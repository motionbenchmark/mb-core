//! Many reports → a per-task board, scored against the task's goals.
//!
//! * Goal score = share of the task's goal a run achieved (success / max score),
//!   averaged over an agent's trials, with a 95% bootstrap interval. Every agent
//!   class (people, team-written code, AI-written code, assisted, commercial) is
//!   scored this way, against the same goals.
//! * Chance level = the median goal score of baseline agents (a random walk) when
//!   the data has any, else the task's fixed floor. Reported, not subtracted.
//! * MotionBench Index = 0.5·goal + 0.2·path efficiency + 0.15·consistency +
//!   0.15·safety. Every term is already on [0, 1], so no reference group is
//!   needed. The index ranks agents within one task and is never averaged across
//!   tasks.
//! * People comparison (optional) = (S_agent − S_chance) / (S_people_median − S_chance),
//!   computed only when the task declares `reference.people_comparable` (a person
//!   can do exactly the same job with the same means, e.g. driving the same robot)
//!   and the data contains people.
//!
//! Spec 0.2 (RFC-0002). Weights are frozen per spec version.

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
pub const BOOTSTRAP_ITERS: usize = 2000;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Leaderboard {
    pub spec_version: String,
    pub scorer_version: String,
    pub weights: Weights,
    pub n_runs: usize,
    pub tasks: Vec<TaskBoard>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Weights {
    pub success: f64,
    pub efficiency: f64,
    pub consistency: f64,
    pub safety: f64,
}

/// People who did the same job with the same means, on tasks that allow it.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PeopleReference {
    /// Median of per-person mean success (task units).
    pub median: f64,
    /// 90th percentile of per-person mean success (task units).
    pub expert: f64,
    pub n_people: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TaskBoard {
    pub task: String,
    pub title: String,
    pub kind: String,
    pub domain: String,
    pub n_runs: usize,
    pub max_score: f64,
    /// Chance level as a goal score in [0, 1]: what a random walk achieves.
    pub chance: f64,
    /// `baseline` (median of baseline-class agents in the data) or `task` (fixed in the task spec).
    pub chance_source: String,
    /// Whether the task allows a like-for-like comparison with people.
    pub people_comparable: bool,
    /// Present only when `people_comparable` and the data contains people.
    pub people: Option<PeopleReference>,
    pub rows: Vec<AgentRow>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AgentRow {
    /// Shared rank: agents whose goal-score intervals overlap the group leader's share a rank.
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
    /// Mean share of the goal achieved, in [0, 1].
    pub goal: f64,
    pub goal_ci95: [f64; 2],
    pub goal_worst: f64,
    pub completion_rate: f64,
    pub time_to_goal_mean_s: Option<f64>,
    pub path_efficiency: f64,
    pub violations_mean: f64,
    pub consistency: f64,
    pub safety: f64,
    pub mbi: f64,
    /// Compared with the median person (1.0) and chance (0); only on people-comparable tasks.
    pub vs_people: Option<f64>,
    pub vs_people_ci95: Option<[f64; 2]>,
}

/// Aggregate reports. `tasks` supplies titles, floors and whether people are a
/// valid comparison; reports for unknown tasks are aggregated with a floor of 0
/// and no people comparison.
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

        // Chance: median of baseline agents when the dataset has any (a random walk
        // covers half a room by accident); otherwise the task's fixed floor.
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

        let goal_of = |x: f64| if max_score > 1e-9 { x / max_score } else { 0.0 };
        let people_comparable = task.map(|t| t.reference.people_comparable).unwrap_or(false);

        // People reference from per-person means, only where the task allows it.
        let people_means: Vec<f64> = by_agent
            .values()
            .filter(|rs| rs[0].agent_class == AgentClass::Human)
            .map(|rs| mean(&rs.iter().map(|r| r.success.score).collect::<Vec<_>>()))
            .collect();
        let people = (people_comparable && !people_means.is_empty()).then(|| PeopleReference {
            median: median(&people_means),
            expert: percentile(&people_means, 0.9),
            n_people: people_means.len(),
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

                let goal = goal_of(m);
                let mbi = W_SUCCESS * goal
                    + W_EFFICIENCY * eff_m.clamp(0.0, 1.0)
                    + W_CONSISTENCY * consistency
                    + W_SAFETY * safety;
                let (vs_people, vs_people_ci) = match &people {
                    Some(p) if (p.median - floor).abs() > 1e-9 => {
                        let norm = |x: f64| (x - floor) / (p.median - floor);
                        (Some(norm(m)), Some([norm(ci.0), norm(ci.1)]))
                    }
                    _ => (None, None),
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
                    goal,
                    goal_ci95: [goal_of(ci.0), goal_of(ci.1)],
                    goal_worst: goal_of(s.iter().cloned().fold(f64::INFINITY, f64::min)),
                    completion_rate: rs.iter().filter(|r| r.success.completed).count() as f64
                        / rs.len() as f64,
                    time_to_goal_mean_s: (!ttg.is_empty()).then(|| mean(&ttg)),
                    path_efficiency: eff_m,
                    violations_mean: mean(&viol),
                    consistency,
                    safety,
                    mbi,
                    vs_people,
                    vs_people_ci95: vs_people_ci,
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
            max_score,
            chance: goal_of(floor),
            chance_source: floor_source.to_string(),
            people_comparable,
            people,
            rows,
        });
    }

    Leaderboard {
        spec_version: crate::SPEC_VERSION.into(),
        scorer_version: crate::SCORER_VERSION.into(),
        weights: Weights {
            success: W_SUCCESS,
            efficiency: W_EFFICIENCY,
            consistency: W_CONSISTENCY,
            safety: W_SAFETY,
        },
        n_runs: reports.len(),
        tasks: boards,
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
