//! Turns one `(task, trace)` pair into a [`Report`] with six metric families:
//! success, efficiency, precision, coverage, safety and process.

use crate::error::{Error, Result};
use crate::geometry::{angle_diff, dist, Rect};
use crate::task::{Goal, Layout, Task};
use crate::trace::{AgentClass, Event, PoseSample, Trace};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Tracking noise tolerated before a boundary or obstacle contact counts (m).
pub const CONTACT_TOLERANCE_M: f64 = 0.03;
/// Below this speed the robot counts as hesitating (m/s).
pub const HESITATION_SPEED_MPS: f64 = 0.02;
/// Coverage tasks count as "completed" at this fraction of free area.
pub const COVERAGE_COMPLETE: f64 = 0.9;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    pub spec_version: String,
    pub scorer_version: String,
    pub task: String,
    pub task_kind: String,
    pub seed: String,
    pub agent_id: String,
    pub agent_name: String,
    pub agent_class: AgentClass,
    pub trial: u32,
    pub tier: String,
    pub capture: String,
    pub duration_s: f64,
    pub success: Success,
    pub efficiency: Efficiency,
    pub precision: Precision,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coverage: Option<Coverage>,
    pub safety: Safety,
    pub process: Process,
    #[serde(default)]
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Success {
    /// Task points (waypoints hit, objects delivered, % area covered).
    pub score: f64,
    pub max_score: f64,
    /// score / max_score, in [0, 1].
    pub completion: f64,
    pub completed: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Efficiency {
    pub time_to_goal_s: Option<f64>,
    pub path_length_m: f64,
    /// Length of an ideal path for the part of the goal actually achieved.
    pub oracle_length_m: f64,
    /// oracle / actual, in [0, 1]. Zero when nothing was achieved.
    pub path_efficiency: f64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Precision {
    pub final_pose_error_m: Option<f64>,
    pub mean_target_error_m: Option<f64>,
    pub mean_heading_error_rad: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Coverage {
    pub covered_pct: f64,
    pub redundant_pct: f64,
    pub time_to_90_s: Option<f64>,
    pub free_area_m2: f64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Safety {
    pub collisions: u32,
    pub boundary_violations: u32,
    pub obstacle_intrusions: u32,
    pub rule_flags: u32,
    pub total: u32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Process {
    pub mean_speed_mps: f64,
    pub p99_speed_mps: f64,
    pub hesitation_s: f64,
    pub jerk_rms: f64,
}

/// Score one run. Pure and deterministic.
pub fn score(task: &Task, trace: &Trace) -> Result<Report> {
    let m = &trace.manifest;
    if m.task != task.key() {
        return Err(Error::InvalidTrace(format!(
            "trace is for task {} but scorer was given {}",
            m.task,
            task.key()
        )));
    }
    let layout = task.layout(&m.seed)?;
    let mut warnings = vec![];

    // Re-base time at zero and enforce the budget.
    let t0 = trace.poses[0].t_s;
    let limit = task.budget.time_s + 1e-9;
    let poses: Vec<PoseSample> = trace
        .poses
        .iter()
        .map(|p| PoseSample {
            t_s: p.t_s - t0,
            ..*p
        })
        .filter(|p| p.t_s <= limit)
        .collect();
    if poses.len() < trace.poses.len() {
        warnings.push(format!(
            "{} pose samples after the {:.0} s budget were ignored",
            trace.poses.len() - poses.len(),
            task.budget.time_s
        ));
    }
    if poses.len() < 2 {
        return Err(Error::InvalidTrace(
            "fewer than 2 pose samples inside the budget".into(),
        ));
    }
    let events: Vec<Event> = trace
        .events
        .iter()
        .map(|e| Event {
            t_s: e.t_s - t0,
            ..e.clone()
        })
        .filter(|e| e.t_s <= limit)
        .collect();

    let cum = cumulative_length(&poses);
    let path_length = *cum.last().unwrap();
    let duration = poses.last().unwrap().t_s;
    let safety = safety(task, &poses, &events);
    let process = process(&poses, path_length);

    let (success, efficiency, precision, coverage) = match &task.goal {
        Goal::Waypoints {
            pos_tol_m,
            heading_tol_rad,
        } => waypoints(layout, &poses, &cum, *pos_tol_m, *heading_tol_rad),
        Goal::Delivery { .. } => {
            delivery(task, layout, &poses, &cum, &events, trace, &mut warnings)
        }
        Goal::Coverage {
            cell_m,
            tool_width_m,
        } => coverage(task, &poses, path_length, *cell_m, *tool_width_m),
    };

    Ok(Report {
        spec_version: crate::SPEC_VERSION.into(),
        scorer_version: crate::SCORER_VERSION.into(),
        task: task.key(),
        task_kind: task.goal.kind().into(),
        seed: m.seed.clone(),
        agent_id: m.agent.id.clone(),
        agent_name: if m.agent.name.is_empty() {
            m.agent.id.clone()
        } else {
            m.agent.name.clone()
        },
        agent_class: m.agent.class,
        trial: m.trial,
        tier: m.tier.clone(),
        capture: m.capture.clone(),
        duration_s: duration,
        success,
        efficiency,
        precision,
        coverage,
        safety,
        process,
        warnings,
    })
}

fn cumulative_length(p: &[PoseSample]) -> Vec<f64> {
    let mut out = Vec::with_capacity(p.len());
    let mut acc = 0.0;
    out.push(0.0);
    for w in p.windows(2) {
        acc += dist(w[0].x_m, w[0].y_m, w[1].x_m, w[1].y_m);
        out.push(acc);
    }
    out
}

/// Count episodes (entries into a bad state), not samples, so a slow scrape counts once.
fn episodes(flags: impl Iterator<Item = bool>) -> u32 {
    let mut n = 0;
    let mut prev = false;
    for f in flags {
        if f && !prev {
            n += 1;
        }
        prev = f;
    }
    n
}

fn safety(task: &Task, poses: &[PoseSample], events: &[Event]) -> Safety {
    let r = (task.robot.radius_m - CONTACT_TOLERANCE_M).max(0.0);
    let (w, h) = (task.arena.width_m, task.arena.height_m);
    let boundary = episodes(
        poses
            .iter()
            .map(|p| p.x_m < r || p.y_m < r || p.x_m > w - r || p.y_m > h - r),
    );
    let obstacle: u32 = task
        .arena
        .obstacles
        .iter()
        .map(|o| episodes(poses.iter().map(|p| o.overlaps_circle(p.x_m, p.y_m, r))))
        .sum();
    let collisions = events.iter().filter(|e| e.kind == "collision").count() as u32;
    let rule_flags = events.iter().filter(|e| e.kind == "rule_flag").count() as u32;
    Safety {
        collisions,
        boundary_violations: boundary,
        obstacle_intrusions: obstacle,
        rule_flags,
        total: collisions + boundary + obstacle + rule_flags,
    }
}

fn process(poses: &[PoseSample], path_length: f64) -> Process {
    let duration = poses.last().unwrap().t_s.max(1e-9);
    let mut speeds = vec![];
    let mut times = vec![];
    let mut hesitation = 0.0;
    for w in poses.windows(2) {
        let dt = w[1].t_s - w[0].t_s;
        let v = dist(w[0].x_m, w[0].y_m, w[1].x_m, w[1].y_m) / dt;
        if v < HESITATION_SPEED_MPS {
            hesitation += dt;
        }
        speeds.push(v);
        times.push((w[0].t_s + w[1].t_s) / 2.0);
    }
    // Smooth speed (window 5) before differentiating twice: raw tracking noise
    // would otherwise dominate jerk.
    let k = 2usize;
    let smooth: Vec<f64> = (0..speeds.len())
        .map(|i| {
            let lo = i.saturating_sub(k);
            let hi = (i + k + 1).min(speeds.len());
            crate::stats::mean(&speeds[lo..hi])
        })
        .collect();
    let deriv = |v: &[f64], t: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let mut d = vec![];
        let mut tt = vec![];
        for i in 1..v.len() {
            let dt = (t[i] - t[i - 1]).max(1e-9);
            d.push((v[i] - v[i - 1]) / dt);
            tt.push((t[i] + t[i - 1]) / 2.0);
        }
        (d, tt)
    };
    let (acc, ta) = deriv(&smooth, &times);
    let (jerk, _) = deriv(&acc, &ta);
    let jerk_rms = if jerk.is_empty() {
        0.0
    } else {
        (jerk.iter().map(|j| j * j).sum::<f64>() / jerk.len() as f64).sqrt()
    };
    Process {
        mean_speed_mps: path_length / duration,
        p99_speed_mps: crate::stats::percentile(&speeds, 0.99),
        hesitation_s: hesitation,
        jerk_rms,
    }
}

type Scored = (Success, Efficiency, Precision, Option<Coverage>);

fn ratio(oracle: f64, actual: f64) -> f64 {
    if oracle <= 0.0 || actual <= 0.0 {
        0.0
    } else {
        (oracle / actual).min(1.0)
    }
}

fn waypoints(
    layout: &Layout,
    poses: &[PoseSample],
    cum: &[f64],
    pos_tol: f64,
    head_tol: f64,
) -> Scored {
    let wps = &layout.waypoints;
    let mut k = 0;
    let mut hit_idx = vec![];
    let mut pos_err = vec![];
    let mut head_err = vec![];
    for (i, p) in poses.iter().enumerate() {
        if k >= wps.len() {
            break;
        }
        let w = wps[k];
        let d = dist(p.x_m, p.y_m, w.x, w.y);
        let a = angle_diff(p.theta_rad, w.theta);
        if d <= pos_tol && a <= head_tol {
            hit_idx.push(i);
            pos_err.push(d);
            head_err.push(a);
            k += 1;
        }
    }
    let n = wps.len().max(1) as f64;
    let hits = hit_idx.len();
    let completed = hits == wps.len() && !wps.is_empty();

    let mut oracle = 0.0;
    let (mut px, mut py) = (layout.start.x, layout.start.y);
    for w in wps.iter().take(hits) {
        oracle += dist(px, py, w.x, w.y);
        px = w.x;
        py = w.y;
    }
    let actual = hit_idx.last().map(|&i| cum[i]).unwrap_or(0.0);
    let last = poses.last().unwrap();
    let final_err = wps.last().map(|w| dist(last.x_m, last.y_m, w.x, w.y));

    (
        Success {
            score: hits as f64,
            max_score: wps.len() as f64,
            completion: hits as f64 / n,
            completed,
        },
        Efficiency {
            time_to_goal_s: if completed {
                hit_idx.last().map(|&i| poses[i].t_s)
            } else {
                None
            },
            path_length_m: *cum.last().unwrap(),
            oracle_length_m: oracle,
            path_efficiency: ratio(oracle, actual),
        },
        Precision {
            final_pose_error_m: final_err,
            mean_target_error_m: (!pos_err.is_empty()).then(|| crate::stats::mean(&pos_err)),
            mean_heading_error_rad: (!head_err.is_empty()).then(|| crate::stats::mean(&head_err)),
        },
        None,
    )
}

/// Greedy single-carry tour: from the current position go to the nearest
/// remaining object, carry it to its zone centre, repeat. Returns the cumulative
/// oracle length after each delivery.
fn greedy_delivery_oracle(task: &Task, layout: &Layout) -> Vec<f64> {
    let mut remaining: Vec<(f64, f64, (f64, f64))> = layout
        .objects
        .iter()
        .filter_map(|o| {
            let z = task.target_zone(o)?;
            (!z.contains(o.x, o.y)).then(|| (o.x, o.y, z.center()))
        })
        .collect();
    let (mut px, mut py) = (layout.start.x, layout.start.y);
    let mut acc = 0.0;
    let mut out = vec![];
    while !remaining.is_empty() {
        let (j, _) = remaining
            .iter()
            .enumerate()
            .map(|(j, &(ox, oy, _))| (j, dist(px, py, ox, oy)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        let (ox, oy, (zx, zy)) = remaining.remove(j);
        acc += dist(px, py, ox, oy) + dist(ox, oy, zx, zy);
        out.push(acc);
        px = zx;
        py = zy;
    }
    out
}

fn delivery(
    task: &Task,
    layout: &Layout,
    poses: &[PoseSample],
    cum: &[f64],
    events: &[Event],
    trace: &Trace,
    warnings: &mut Vec<String>,
) -> Scored {
    let targets: BTreeMap<&str, &Rect> = layout
        .objects
        .iter()
        .filter_map(|o| task.target_zone(o).map(|z| (o.id.as_str(), z)))
        .collect();
    let mut errs = vec![];
    let delivered: usize = match &trace.world_final {
        Some(w) => {
            let mut finals: BTreeMap<&str, (f64, f64)> = BTreeMap::new();
            for o in &w.objects {
                if targets.contains_key(o.id.as_str()) {
                    finals.insert(&o.id, (o.x, o.y));
                } else {
                    warnings.push(format!("world_final lists unknown object '{}'", o.id));
                }
            }
            finals
                .iter()
                .filter(|(id, (x, y))| targets[*id].contains(*x, *y))
                .inspect(|(id, (x, y))| {
                    let (zx, zy) = targets[*id].center();
                    errs.push(dist(*x, *y, zx, zy))
                })
                .count()
        }
        None => {
            warnings.push("no world_final.json; using deliver events (lower confidence)".into());
            events
                .iter()
                .filter(|e| e.kind == "deliver" && targets.contains_key(e.object_id.as_str()))
                .map(|e| e.object_id.as_str())
                .collect::<BTreeSet<_>>()
                .len()
        }
    };
    let n = layout.objects.len();
    let completed = delivered == n && n > 0;
    let last_deliver = events
        .iter()
        .filter(|e| e.kind == "deliver")
        .map(|e| e.t_s)
        .fold(None, |a: Option<f64>, t| Some(a.map_or(t, |a| a.max(t))));
    let actual = match last_deliver {
        Some(t) => {
            let i = poses
                .iter()
                .position(|p| p.t_s >= t)
                .unwrap_or(poses.len() - 1);
            cum[i]
        }
        None => *cum.last().unwrap(),
    };
    let oracle_steps = greedy_delivery_oracle(task, layout);
    let oracle = if delivered == 0 {
        0.0
    } else {
        oracle_steps[(delivered.min(oracle_steps.len())).max(1) - 1]
    };
    (
        Success {
            score: delivered as f64,
            max_score: n as f64,
            completion: delivered as f64 / n.max(1) as f64,
            completed,
        },
        Efficiency {
            time_to_goal_s: if completed { last_deliver } else { None },
            path_length_m: *cum.last().unwrap(),
            oracle_length_m: oracle,
            path_efficiency: ratio(oracle, actual),
        },
        Precision {
            final_pose_error_m: None,
            mean_target_error_m: (!errs.is_empty()).then(|| crate::stats::mean(&errs)),
            mean_heading_error_rad: None,
        },
        None,
    )
}

fn coverage(
    task: &Task,
    poses: &[PoseSample],
    path_length: f64,
    cell: f64,
    tool_width: f64,
) -> Scored {
    let (w, h) = (task.arena.width_m, task.arena.height_m);
    let nx = (w / cell).ceil() as usize;
    let ny = (h / cell).ceil() as usize;
    let center = |i: usize, j: usize| ((i as f64 + 0.5) * cell, (j as f64 + 0.5) * cell);
    let mut free = vec![false; nx * ny];
    for j in 0..ny {
        for i in 0..nx {
            let (cx, cy) = center(i, j);
            free[j * nx + i] =
                cx <= w && cy <= h && !task.arena.obstacles.iter().any(|o| o.contains(cx, cy));
        }
    }
    let n_free = free.iter().filter(|f| **f).count().max(1);
    let mut covered = vec![false; nx * ny];
    let mut n_cov = 0usize;
    let rt = tool_width / 2.0;
    let mut t90 = None;

    let stamp = |x: f64, y: f64, covered: &mut Vec<bool>, n_cov: &mut usize| {
        let i0 = ((x - rt) / cell).floor().max(0.0) as usize;
        let i1 = (((x + rt) / cell).ceil() as usize).min(nx);
        let j0 = ((y - rt) / cell).floor().max(0.0) as usize;
        let j1 = (((y + rt) / cell).ceil() as usize).min(ny);
        for j in j0..j1 {
            for i in i0..i1 {
                let k = j * nx + i;
                if free[k] && !covered[k] {
                    let (cx, cy) = center(i, j);
                    if dist(x, y, cx, cy) <= rt {
                        covered[k] = true;
                        *n_cov += 1;
                    }
                }
            }
        }
    };

    stamp(poses[0].x_m, poses[0].y_m, &mut covered, &mut n_cov);
    for win in poses.windows(2) {
        let (a, b) = (win[0], win[1]);
        let seg = dist(a.x_m, a.y_m, b.x_m, b.y_m);
        let steps = ((seg / (cell / 2.0)).ceil() as usize).max(1);
        for s in 1..=steps {
            let f = s as f64 / steps as f64;
            stamp(
                a.x_m + (b.x_m - a.x_m) * f,
                a.y_m + (b.y_m - a.y_m) * f,
                &mut covered,
                &mut n_cov,
            );
        }
        if t90.is_none() && n_cov as f64 >= COVERAGE_COMPLETE * n_free as f64 {
            t90 = Some(b.t_s);
        }
    }

    let frac = n_cov as f64 / n_free as f64;
    let covered_area = n_cov as f64 * cell * cell;
    let swept = path_length * tool_width;
    let redundant = if swept > 0.0 {
        ((swept - covered_area).max(0.0) / swept) * 100.0
    } else {
        0.0
    };
    let oracle = covered_area / tool_width;
    (
        Success {
            score: frac * 100.0,
            max_score: 100.0,
            completion: frac,
            completed: frac >= COVERAGE_COMPLETE,
        },
        Efficiency {
            time_to_goal_s: t90,
            path_length_m: path_length,
            oracle_length_m: oracle,
            path_efficiency: ratio(oracle, path_length),
        },
        Precision::default(),
        Some(Coverage {
            covered_pct: frac * 100.0,
            redundant_pct: redundant,
            time_to_90_s: t90,
            free_area_m2: n_free as f64 * cell * cell,
        }),
    )
}
