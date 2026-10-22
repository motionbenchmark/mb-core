//! Many reports → a human-normalized leaderboard.
//!
//! * HNS (human-normalized score) = (S_agent − S_floor) / (S_human_median − S_floor),
//!   where S_floor is the median baseline-agent score if the data has baseline runs,
//!   else the task's fixed floor.
//! * MBI (MotionBench Index) = 0.5·clip(HNS_success) + 0.2·clip(HNS_efficiency)
//!   + 0.15·Consistency + 0.15·Safety, with HNS clipped to [0, 2].
//!
//! Weights are frozen per major spec version (see RFC-0001).

use crate::score::Report;
use crate::stats::{bootstrap_ci95, fnv1a, mean, median, percentile, std_dev};
use crate::task::Task;
use crate::trace::AgentClass;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const W_SUCCESS: f64 = 0.5;
pub const W_EFFICIENCY: f64 = 0.2;
pub const W_CONSISTENCY: f64 = 0.15;
pub const W_SAFETY: f64 = 0.15;
pub const HNS_CLIP: f64 = 2.0;
pub const BOOTSTRAP_ITERS: usize = 2000;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Leaderboard {
    pub spec_version: String,
    pub scorer_version: String,
    pub weights: Weights,
    pub n_runs: usize,
    pub tasks: Vec<TaskBoard>,
    pub overall: Vec<OverallRow>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Weights {
    pub success: f64,
    pub efficiency: f64,
    pub consistency: f64,
    pub safety: f64,
    pub hns_clip: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HumanReference {
    /// Median of per-driver mean success.
    pub median: f64,
    /// 90th percentile of per-driver mean success.
    pub expert: f64,
    pub median_path_efficiency: f64,
    pub n_drivers: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TaskBoard {
    pub task: String,
    pub title: String,
    pub kind: String,
    pub domain: String,
    pub n_runs: usize,
    pub floor: f64,
    /// `baseline` (median of baseline-class agents in the data) or `task` (fixed in the task spec).
    pub floor_source: String,
    pub max_score: f64,
    pub human_reference: Option<HumanReference>,
    pub rows: Vec<AgentRow>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AgentRow {
    /// Shared rank: agents whose HNS confidence intervals overlap share a rank.
    pub rank: usize,
    pub agent_id: String,
    pub agent_name: String,
    pub class: AgentClass,
    pub n_trials: usize,
    pub success_mean: f64,
    pub success_median: f64,
    pub success_std: f64,
    pub success_worst: f64,
    pub success_best: f64,
    pub success_ci95: [f64; 2],
    pub completion_rate: f64,
    pub time_to_goal_mean_s: Option<f64>,
    pub path_efficiency: f64,
    pub violations_mean: f64,
    pub hns: Option<f64>,
    pub hns_ci95: Option<[f64; 2]>,
    pub hns_expert: Option<f64>,
    pub consistency: f64,
    pub safety: f64,
    pub mbi: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OverallRow {
    pub rank: usize,
    pub agent_id: String,
    pub agent_name: String,
    pub class: AgentClass,
    pub n_tasks: usize,
    pub mbi_mean: f64,
    pub hns_mean: f64,
}

fn clip(x: f64) -> f64 {
    x.clamp(0.0, HNS_CLIP)
}

/// Aggregate reports. `tasks` supplies titles and floors; reports for unknown
/// tasks are still aggregated with a floor of 0.
pub fn aggregate(reports: &[Report], tasks: &BTreeMap<String, Task>) -> Leaderboard {
    let mut by_task: BTreeMap<&str, Vec<&Report>> = BTreeMap::new();
    for r in reports {
        by_task.entry(&r.task).or_default().push(r);
    }

    let mut boards = vec![];
    for (key, runs) in by_task {
        let task = tasks.get(key);
        let max_score = runs.iter().map(|r| r.success.max_score).fold(0.0, f64::max);

        let mut by_agent: BTreeMap<&str, Vec<&Report>> = BTreeMap::new();
        for r in &runs {
            by_agent.entry(&r.agent_id).or_default().push(r);
        }

        // Floor: median of baseline agents when the dataset has any (a random walk
        // covers half a room by accident, so 0 would flatter every agent);
        // otherwise the task's fixed floor.
        let baselines: Vec<f64> = by_agent
            .values()
            .filter(|rs| rs[0].agent_class == AgentClass::Baseline)
            .map(|rs| mean(&rs.iter().map(|r| r.success.score).collect::<Vec<_>>()))
            .collect();
        let (floor, floor_source) = if baselines.is_empty() {
            (task.map(|t| t.reference.floor).unwrap_or(0.0), "task")
        } else {
            (median(&baselines), "baseline")
        };

        // Human reference from per-driver means.
        let humans: Vec<(f64, f64)> = by_agent
            .values()
            .filter(|rs| rs[0].agent_class == AgentClass::Human)
            .map(|rs| {
                let s: Vec<f64> = rs.iter().map(|r| r.success.score).collect();
                let e: Vec<f64> = rs.iter().map(|r| r.efficiency.path_efficiency).collect();
                (mean(&s), mean(&e))
            })
            .collect();
        let human_reference = (!humans.is_empty()).then(|| {
            let s: Vec<f64> = humans.iter().map(|h| h.0).collect();
            let e: Vec<f64> = humans.iter().map(|h| h.1).collect();
            HumanReference {
                median: median(&s),
                expert: percentile(&s, 0.9),
                median_path_efficiency: median(&e),
                n_drivers: humans.len(),
            }
        });

        let mut rows: Vec<AgentRow> = by_agent
            .iter()
            .map(|(id, rs)| {
                let s: Vec<f64> = rs.iter().map(|r| r.success.score).collect();
                let eff: Vec<f64> = rs.iter().map(|r| r.efficiency.path_efficiency).collect();
                let viol: Vec<f64> = rs.iter().map(|r| r.safety.total as f64).collect();
                let ttg: Vec<f64> = rs
                    .iter()
                    .filter_map(|r| r.efficiency.time_to_goal_s)
                    .collect();
                let m = mean(&s);
                let sd = std_dev(&s);
                let ci = bootstrap_ci95(&s, BOOTSTRAP_ITERS, fnv1a(&format!("{key}/{id}")));
                let consistency = if m > 1e-9 {
                    (1.0 - sd / m).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let safety = 1.0 / (1.0 + mean(&viol));
                let eff_m = mean(&eff);

                let (hns, hns_ci, hns_expert, mbi) = match &human_reference {
                    Some(h) if (h.median - floor).abs() > 1e-9 => {
                        let norm = |x: f64| (x - floor) / (h.median - floor);
                        let hns = norm(m);
                        let hns_e = if (h.expert - floor).abs() > 1e-9 {
                            Some((m - floor) / (h.expert - floor))
                        } else {
                            None
                        };
                        let hns_eff = if h.median_path_efficiency > 1e-9 {
                            eff_m / h.median_path_efficiency
                        } else {
                            0.0
                        };
                        let mbi = W_SUCCESS * clip(hns)
                            + W_EFFICIENCY * clip(hns_eff)
                            + W_CONSISTENCY * consistency
                            + W_SAFETY * safety;
                        (Some(hns), Some([norm(ci.0), norm(ci.1)]), hns_e, Some(mbi))
                    }
                    _ => (None, None, None, None),
                };

                AgentRow {
                    rank: 0,
                    agent_id: id.to_string(),
                    agent_name: rs[0].agent_name.clone(),
                    class: rs[0].agent_class,
                    n_trials: rs.len(),
                    success_mean: m,
                    success_median: median(&s),
                    success_std: sd,
                    success_worst: s.iter().cloned().fold(f64::INFINITY, f64::min),
                    success_best: s.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
                    success_ci95: [ci.0, ci.1],
                    completion_rate: rs.iter().filter(|r| r.success.completed).count() as f64
                        / rs.len() as f64,
                    time_to_goal_mean_s: (!ttg.is_empty()).then(|| mean(&ttg)),
                    path_efficiency: eff_m,
                    violations_mean: mean(&viol),
                    hns,
                    hns_ci95: hns_ci,
                    hns_expert,
                    consistency,
                    safety,
                    mbi,
                }
            })
            .collect();

        rows.sort_by(|a, b| {
            b.success_mean
                .partial_cmp(&a.success_mean)
                .unwrap()
                .then(a.agent_id.cmp(&b.agent_id))
        });
        assign_shared_ranks(&mut rows);

        boards.push(TaskBoard {
            task: key.to_string(),
            title: task.map(|t| t.title.clone()).unwrap_or_default(),
            kind: runs[0].task_kind.clone(),
            domain: task.map(|t| t.domain.clone()).unwrap_or_default(),
            n_runs: runs.len(),
            floor,
            floor_source: floor_source.to_string(),
            max_score,
            human_reference,
            rows,
        });
    }

    let overall = overall(&boards);
    Leaderboard {
        spec_version: crate::SPEC_VERSION.into(),
        scorer_version: crate::SCORER_VERSION.into(),
        weights: Weights {
            success: W_SUCCESS,
            efficiency: W_EFFICIENCY,
            consistency: W_CONSISTENCY,
            safety: W_SAFETY,
            hns_clip: HNS_CLIP,
        },
        n_runs: reports.len(),
        tasks: boards,
        overall,
    }
}

/// Rows must be sorted best-first. A new rank starts only when an agent's CI
/// upper bound falls below the lower bound of the current group's leader.
fn assign_shared_ranks(rows: &mut [AgentRow]) {
    let mut rank = 0;
    let mut leader_lo = f64::INFINITY;
    for (i, r) in rows.iter_mut().enumerate() {
        if i == 0 || r.success_ci95[1] < leader_lo {
            rank = i + 1;
            leader_lo = r.success_ci95[0];
        }
        r.rank = rank;
    }
}

fn overall(boards: &[TaskBoard]) -> Vec<OverallRow> {
    let mut acc: BTreeMap<String, (String, AgentClass, Vec<f64>, Vec<f64>)> = BTreeMap::new();
    for b in boards {
        for r in &b.rows {
            if let (Some(mbi), Some(hns)) = (r.mbi, r.hns) {
                let e = acc.entry(r.agent_id.clone()).or_insert((
                    r.agent_name.clone(),
                    r.class,
                    vec![],
                    vec![],
                ));
                e.2.push(mbi);
                e.3.push(hns);
            }
        }
    }
    let mut rows: Vec<OverallRow> = acc
        .into_iter()
        .map(|(id, (name, class, mbi, hns))| OverallRow {
            rank: 0,
            agent_id: id,
            agent_name: name,
            class,
            n_tasks: mbi.len(),
            mbi_mean: mean(&mbi),
            hns_mean: mean(&hns),
        })
        .collect();
    rows.sort_by(|a, b| {
        b.mbi_mean
            .partial_cmp(&a.mbi_mean)
            .unwrap()
            .then(a.agent_id.cmp(&b.agent_id))
    });
    for (i, r) in rows.iter_mut().enumerate() {
        r.rank = i + 1;
    }
    rows
}
