use clap::{Parser, Subcommand};
use motionbench_core::{aggregate, builtin_tasks, find_task, score, trace::discover, Task, Trace};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

/// MotionBench: score goal-directed robot runs in confined spaces against human references.
#[derive(Parser)]
#[command(name = "mb", version, about)]
struct Cli {
    /// Directory of task specs (`<ID@VERSION>.json`). Defaults to the built-in tasks.
    #[arg(long, global = true)]
    tasks: Option<PathBuf>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// List available tasks.
    Tasks,
    /// Check a .mbtrace directory for structural problems.
    Validate { trace: PathBuf },
    /// Score one .mbtrace directory.
    Score {
        trace: PathBuf,
        /// Print the full JSON report instead of a summary.
        #[arg(long)]
        json: bool,
    },
    /// Score every .mbtrace under a directory and build a leaderboard.
    Aggregate {
        root: PathBuf,
        /// Write the leaderboard JSON here (default: stdout).
        #[arg(short, long)]
        out: Option<PathBuf>,
        /// Also write every per-run report to this JSON-lines file.
        #[arg(long)]
        reports: Option<PathBuf>,
    },
}

fn task_map(dir: Option<&Path>) -> BTreeMap<String, Task> {
    let mut m: BTreeMap<String, Task> = builtin_tasks().into_iter().map(|t| (t.key(), t)).collect();
    if let Some(d) = dir {
        if let Ok(rd) = std::fs::read_dir(d) {
            for e in rd.flatten() {
                if e.path().extension().is_some_and(|x| x == "json") {
                    match Task::load(&e.path()) {
                        Ok(t) => {
                            m.insert(t.key(), t);
                        }
                        Err(err) => eprintln!("warning: skipping {}: {err}", e.path().display()),
                    }
                }
            }
        }
    }
    m
}

fn run(cli: Cli) -> Result<(), String> {
    let tdir = cli.tasks.as_deref();
    match cli.cmd {
        Cmd::Tasks => {
            for t in task_map(tdir).values() {
                println!(
                    "{:<18} {:<11} {:<14} {:>5.0}s  {}  (seeds: {})",
                    t.key(),
                    t.goal.kind(),
                    t.domain,
                    t.budget.time_s,
                    t.title,
                    t.layouts.keys().cloned().collect::<Vec<_>>().join(", ")
                );
            }
        }
        Cmd::Validate { trace } => {
            let tr = Trace::open(&trace).map_err(|e| e.to_string())?;
            let task = find_task(&tr.manifest.task, tdir).map_err(|e| e.to_string())?;
            task.layout(&tr.manifest.seed).map_err(|e| e.to_string())?;
            println!(
                "ok: {} samples, {} events, task {}, seed {}, agent {} ({})",
                tr.poses.len(),
                tr.events.len(),
                tr.manifest.task,
                tr.manifest.seed,
                tr.manifest.agent.id,
                tr.manifest.agent.class
            );
        }
        Cmd::Score { trace, json } => {
            let tr = Trace::open(&trace).map_err(|e| e.to_string())?;
            let task = find_task(&tr.manifest.task, tdir).map_err(|e| e.to_string())?;
            let r = score(&task, &tr).map_err(|e| e.to_string())?;
            if json {
                println!("{}", serde_json::to_string_pretty(&r).unwrap());
            } else {
                println!(
                    "{}  seed {}  agent {} ({})",
                    r.task, r.seed, r.agent_name, r.agent_class
                );
                println!(
                    "  success     {:.2} / {:.0}  ({:.0}%){}",
                    r.success.score,
                    r.success.max_score,
                    r.success.completion * 100.0,
                    if r.success.completed {
                        "  completed"
                    } else {
                        ""
                    }
                );
                println!(
                    "  efficiency  path {:.2} m, oracle {:.2} m, efficiency {:.2}, time-to-goal {}",
                    r.efficiency.path_length_m,
                    r.efficiency.oracle_length_m,
                    r.efficiency.path_efficiency,
                    r.efficiency
                        .time_to_goal_s
                        .map(|t| format!("{t:.2} s"))
                        .unwrap_or("-".into())
                );
                if let Some(e) = r.precision.mean_target_error_m {
                    println!("  precision   mean target error {:.3} m", e);
                }
                if let Some(c) = &r.coverage {
                    println!(
                        "  coverage    {:.1}% covered, {:.1}% redundant",
                        c.covered_pct, c.redundant_pct
                    );
                }
                println!(
                    "  safety      {} violations (boundary {}, obstacle {}, collision {}, rule {})",
                    r.safety.total,
                    r.safety.boundary_violations,
                    r.safety.obstacle_intrusions,
                    r.safety.collisions,
                    r.safety.rule_flags
                );
                println!(
                    "  process     mean speed {:.2} m/s, hesitation {:.1} s",
                    r.process.mean_speed_mps, r.process.hesitation_s
                );
                for w in &r.warnings {
                    println!("  warning: {w}");
                }
            }
        }
        Cmd::Aggregate { root, out, reports } => {
            let tasks = task_map(tdir);
            let dirs = discover(&root);
            if dirs.is_empty() {
                return Err(format!("no .mbtrace directories under {}", root.display()));
            }
            let mut rs = vec![];
            let mut failed = 0;
            for d in &dirs {
                let res = Trace::open(d).and_then(|tr| {
                    let task = tasks.get(&tr.manifest.task).cloned().ok_or_else(|| {
                        motionbench_core::Error::UnknownTask(tr.manifest.task.clone())
                    })?;
                    score(&task, &tr)
                });
                match res {
                    Ok(r) => rs.push(r),
                    Err(e) => {
                        failed += 1;
                        eprintln!("warning: {}: {e}", d.display());
                    }
                }
            }
            eprintln!("scored {} runs ({} failed)", rs.len(), failed);
            if let Some(p) = reports {
                let lines: Vec<String> = rs
                    .iter()
                    .map(|r| serde_json::to_string(r).unwrap())
                    .collect();
                std::fs::write(&p, lines.join("\n") + "\n").map_err(|e| e.to_string())?;
            }
            let lb = aggregate(&rs, &tasks);
            let s = serde_json::to_string_pretty(&lb).unwrap();
            match out {
                Some(p) => std::fs::write(&p, s).map_err(|e| e.to_string())?,
                None => println!("{s}"),
            }
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
