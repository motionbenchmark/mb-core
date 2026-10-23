//! End-to-end scorer tests on hand-built traces with known answers.

use motionbench_core::trace::{Agent, Event, Manifest, PoseSample, WorldFinal};
use motionbench_core::{aggregate, find_task, score, AgentClass, Trace};
use std::collections::BTreeMap;

fn manifest(task: &str, seed: &str, id: &str, class: AgentClass, trial: u32) -> Manifest {
    Manifest {
        spec_version: "0.1".into(),
        task: task.into(),
        seed: seed.into(),
        agent: Agent {
            id: id.into(),
            class,
            name: id.into(),
            meta: serde_json::Value::Null,
        },
        trial,
        tier: "S".into(),
        capture: "test".into(),
        notes: String::new(),
        files: BTreeMap::new(),
    }
}

/// Straight-line interpolation through poses at 20 Hz, `speed` m/s, holding each target 0.2 s.
fn drive(points: &[(f64, f64, f64)], speed: f64) -> Vec<PoseSample> {
    let dt = 0.05;
    let mut t = 0.0;
    let mut out = vec![PoseSample {
        t_s: 0.0,
        x_m: points[0].0,
        y_m: points[0].1,
        theta_rad: points[0].2,
    }];
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let d = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let steps = ((d / (speed * dt)).ceil() as usize).max(1);
        for s in 1..=steps {
            let f = s as f64 / steps as f64;
            t += dt;
            out.push(PoseSample {
                t_s: t,
                x_m: a.0 + (b.0 - a.0) * f,
                y_m: a.1 + (b.1 - a.1) * f,
                theta_rad: b.2,
            });
        }
        for _ in 0..4 {
            t += dt;
            out.push(PoseSample {
                t_s: t,
                x_m: b.0,
                y_m: b.1,
                theta_rad: b.2,
            });
        }
    }
    out
}

#[test]
fn perfect_waypoint_run_scores_full() {
    let task = find_task("MB-VEX-NAV@1.0", None).unwrap();
    let l = task.layout("dev-01").unwrap();
    let mut pts = vec![(l.start.x, l.start.y, l.start.theta)];
    pts.extend(l.waypoints.iter().map(|w| (w.x, w.y, w.theta)));
    let tr = Trace {
        manifest: manifest(
            "MB-VEX-NAV@1.0",
            "dev-01",
            "ideal",
            AgentClass::HumanAuton,
            0,
        ),
        poses: drive(&pts, 1.0),
        events: vec![],
        world_final: None,
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    assert_eq!(r.success.score, 6.0);
    assert!(r.success.completed);
    assert!(
        r.efficiency.path_efficiency > 0.99,
        "{}",
        r.efficiency.path_efficiency
    );
    assert!(r.efficiency.time_to_goal_s.unwrap() < 30.0);
}

#[test]
fn skipping_a_waypoint_stops_the_sequence() {
    let task = find_task("MB-VEX-NAV@1.0", None).unwrap();
    let l = task.layout("dev-01").unwrap();
    // Visit waypoints 1, 3, 4, 5, 6 — skipping #2 means only #1 counts (order matters).
    let mut pts = vec![(l.start.x, l.start.y, l.start.theta)];
    for (i, w) in l.waypoints.iter().enumerate() {
        if i != 1 {
            pts.push((w.x, w.y, w.theta));
        }
    }
    let tr = Trace {
        manifest: manifest("MB-VEX-NAV@1.0", "dev-01", "skipper", AgentClass::Ai, 0),
        poses: drive(&pts, 1.0),
        events: vec![],
        world_final: None,
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    assert_eq!(r.success.score, 1.0);
    assert!(!r.success.completed);
}

#[test]
fn delivery_counts_world_final_and_flags_unknown_objects() {
    let task = find_task("MB-VEX-PICK@1.0", None).unwrap();
    let l = task.layout("dev-01").unwrap();
    let mut objs = l.objects.clone();
    for o in objs.iter_mut().take(4) {
        o.x = 3.3;
        o.y = 3.3;
    }
    let tr = Trace {
        manifest: manifest("MB-VEX-PICK@1.0", "dev-01", "p", AgentClass::Human, 0),
        poses: drive(&[(0.45, 0.45, 0.0), (3.3, 3.3, 0.78)], 1.0),
        events: vec![Event {
            t_s: 5.0,
            kind: "deliver".into(),
            object_id: "o1".into(),
            detail: String::new(),
        }],
        world_final: Some(WorldFinal { objects: objs }),
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    assert_eq!(r.success.score, 4.0);
    assert_eq!(r.success.max_score, 6.0);
}

#[test]
fn sorting_counts_only_the_matching_zone() {
    let task = find_task("MB-AIM-SORT@0.1", None).unwrap();
    let l = task.layout("dev-01").unwrap();
    let mut objs = l.objects.clone();
    // o1, o2 -> orange zone (correct); b1 -> orange zone (wrong colour); b2 -> blue (correct)
    for o in objs.iter_mut() {
        match o.id.as_str() {
            "o1" | "o2" | "b1" => (o.x, o.y) = (0.15, 0.61),
            "b2" => (o.x, o.y) = (1.68, 0.61),
            _ => {}
        }
    }
    let tr = Trace {
        manifest: manifest("MB-AIM-SORT@0.1", "dev-01", "s", AgentClass::Human, 0),
        poses: drive(&[(0.91, 0.12, 1.57), (0.15, 0.61, 3.0)], 0.3),
        events: vec![],
        world_final: Some(WorldFinal { objects: objs }),
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    assert_eq!(r.success.score, 3.0);
    assert_eq!(r.success.max_score, 6.0);
}

#[test]
fn coverage_of_a_lawnmower_sweep() {
    let task = find_task("MB-COVER@0.1", None).unwrap();
    // Boustrophedon rows across the bottom 0.9 m strip (below the table).
    let mut pts = vec![];
    let mut y = 0.2;
    let mut left = true;
    while y < 0.85 {
        let (a, b) = if left { (0.2, 3.8) } else { (3.8, 0.2) };
        pts.push((a, y, 0.0));
        pts.push((b, y, 0.0));
        y += 0.28;
        left = !left;
    }
    let tr = Trace {
        manifest: manifest("MB-COVER@0.1", "dev-01", "mower", AgentClass::Product, 0),
        poses: drive(&pts, 0.4),
        events: vec![],
        world_final: None,
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    let c = r.coverage.unwrap();
    // ~3.6 m x ~0.86 m swept out of ~9.4 m² free → roughly a third.
    assert!(
        c.covered_pct > 25.0 && c.covered_pct < 40.0,
        "{}",
        c.covered_pct
    );
    assert!(r.efficiency.path_efficiency > 0.7);
}

#[test]
fn budget_is_enforced_and_boundary_counted() {
    let task = find_task("MB-VEX-NAV@1.0", None).unwrap();
    let mut poses = drive(
        &[(0.45, 0.45, 0.0), (0.05, 0.45, 0.0), (0.45, 0.45, 0.0)],
        0.2,
    );
    let last = *poses.last().unwrap();
    poses.push(PoseSample { t_s: 45.0, ..last });
    let tr = Trace {
        manifest: manifest("MB-VEX-NAV@1.0", "dev-01", "wall", AgentClass::Baseline, 0),
        poses,
        events: vec![],
        world_final: None,
        path: None,
    };
    let r = score(&task, &tr).unwrap();
    assert_eq!(r.safety.boundary_violations, 1);
    assert!(r.warnings.iter().any(|w| w.contains("budget")));
}

#[test]
fn aggregate_normalizes_to_human_median() {
    let task = find_task("MB-VEX-NAV@1.0", None).unwrap();
    let l = task.layout("dev-01").unwrap();
    let mut reports = vec![];
    // Humans reach 2, 3, 4 waypoints → median 3. A bot reaching all 6 → HNS 2.
    for (id, n, class) in [
        ("h1", 2, AgentClass::Human),
        ("h2", 3, AgentClass::Human),
        ("h3", 4, AgentClass::Human),
        ("bot", 6, AgentClass::Ai),
    ] {
        for trial in 0..3 {
            let mut pts = vec![(l.start.x, l.start.y, l.start.theta)];
            pts.extend(l.waypoints.iter().take(n).map(|w| (w.x, w.y, w.theta)));
            let tr = Trace {
                manifest: manifest("MB-VEX-NAV@1.0", "dev-01", id, class, trial),
                poses: drive(&pts, 1.0),
                events: vec![],
                world_final: None,
                path: None,
            };
            reports.push(score(&task, &tr).unwrap());
        }
    }
    let tasks = BTreeMap::from([(task.key(), task)]);
    let lb = aggregate(&reports, &tasks);
    let b = &lb.tasks[0];
    assert_eq!(b.human_reference.as_ref().unwrap().median, 3.0);
    let bot = b.rows.iter().find(|r| r.agent_id == "bot").unwrap();
    assert!((bot.hns.unwrap() - 2.0).abs() < 1e-9);
    assert_eq!(bot.rank, 1);
    assert_eq!(lb.overall[0].agent_id, "bot");
}

fn small_room(extra: &str) -> motionbench_core::Task {
    let json = format!(
        r#"{{
        "id": "T-ROOM", "version": "0.1", "title": "test room", "domain": "indoor-floor",
        "arena": {{ "width_m": 1.0, "height_m": 1.0, "obstacles": [], "zones": {{}} {extra} }},
        "robot": {{ "radius_m": 0.1 }},
        "budget": {{ "time_s": 60 }},
        "goal": {{ "kind": "coverage", "cell_m": 0.05, "tool_width_m": 0.2 }},
        "layouts": {{ "s": {{ "start": {{ "x": 0.5, "y": 0.5 }} }} }}
    }}"#
    );
    motionbench_core::Task::from_json(&json, std::path::Path::new("t.json")).unwrap()
}

fn sweep_middle() -> Trace {
    Trace {
        manifest: manifest("T-ROOM@0.1", "s", "r", AgentClass::Product, 0),
        poses: drive(&[(0.1, 0.5, 0.0), (0.9, 0.5, 0.0)], 0.3),
        events: vec![],
        world_final: None,
        path: None,
    }
}

#[test]
fn unobserved_cells_leave_the_denominator() {
    let full = score(&small_room(""), &sweep_middle()).unwrap();
    // Hide the top half of the room: the same sweep now covers a larger share of what was seen.
    let hidden = small_room(r#", "unobserved": [{ "x": 0.0, "y": 0.75, "w": 1.0, "h": 0.25 }]"#);
    let part = score(&hidden, &sweep_middle()).unwrap();
    assert!(
        part.success.score > full.success.score + 5.0,
        "{} vs {}",
        part.success.score,
        full.success.score
    );
    // ...and touching that region is not a safety event.
    assert_eq!(part.safety.obstacle_intrusions, 0);
}

#[test]
fn contact_tolerance_and_edge_reach() {
    let mut t = small_room("");
    // drive along the wall: centre 0.055 m from it with a 0.10 m radius
    let tr = Trace {
        manifest: manifest("T-ROOM@0.1", "s", "r", AgentClass::Product, 0),
        poses: drive(&[(0.2, 0.055, 0.0), (0.8, 0.055, 0.0)], 0.3),
        events: vec![],
        world_final: None,
        path: None,
    };
    assert!(score(&t, &tr).unwrap().safety.boundary_violations > 0);
    t.robot.contact_tolerance_m = Some(0.05);
    assert_eq!(score(&t, &tr).unwrap().safety.boundary_violations, 0);

    let plain = score(&small_room(""), &sweep_middle())
        .unwrap()
        .success
        .score;
    let mut brush = small_room("");
    if let motionbench_core::task::Goal::Coverage { edge_reach_m, .. } = &mut brush.goal {
        *edge_reach_m = Some(0.2);
    }
    let with_brush = score(&brush, &sweep_middle()).unwrap().success.score;
    assert!(with_brush > plain, "{with_brush} vs {plain}");
}
