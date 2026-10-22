//! PyO3 bindings. Everything crosses the boundary as JSON strings so the Python
//! layer stays dependency-free and the Rust core stays the single source of truth.

use motionbench_core::{self as core, trace::discover, Report, Task, Trace};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn err<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(e.to_string())
}

fn task_map(dir: Option<&Path>) -> BTreeMap<String, Task> {
    let mut m: BTreeMap<String, Task> = core::builtin_tasks()
        .into_iter()
        .map(|t| (t.key(), t))
        .collect();
    if let Some(d) = dir {
        if let Ok(rd) = std::fs::read_dir(d) {
            for e in rd.flatten() {
                if e.path().extension().is_some_and(|x| x == "json") {
                    if let Ok(t) = Task::load(&e.path()) {
                        m.insert(t.key(), t);
                    }
                }
            }
        }
    }
    m
}

/// Version of the Rust scorer.
#[pyfunction]
fn scorer_version() -> &'static str {
    core::SCORER_VERSION
}

/// Version of the MotionBench spec implemented.
#[pyfunction]
fn spec_version() -> &'static str {
    core::SPEC_VERSION
}

/// JSON list of all tasks (built-in plus any in `tasks_dir`).
#[pyfunction]
#[pyo3(signature = (tasks_dir=None))]
fn list_tasks(tasks_dir: Option<PathBuf>) -> PyResult<String> {
    let m = task_map(tasks_dir.as_deref());
    serde_json::to_string(&m.values().collect::<Vec<_>>()).map_err(err)
}

/// Score one `.mbtrace` directory; returns the report as JSON.
#[pyfunction]
#[pyo3(signature = (trace_dir, tasks_dir=None))]
fn score_trace(py: Python<'_>, trace_dir: PathBuf, tasks_dir: Option<PathBuf>) -> PyResult<String> {
    py.allow_threads(|| {
        let tr = Trace::open(&trace_dir).map_err(err)?;
        let task = core::find_task(&tr.manifest.task, tasks_dir.as_deref()).map_err(err)?;
        let r = core::score(&task, &tr).map_err(err)?;
        serde_json::to_string(&r).map_err(err)
    })
}

/// Score every trace under `root`. Returns `(reports_json, errors_json)`.
#[pyfunction]
#[pyo3(signature = (root, tasks_dir=None))]
fn score_dir(
    py: Python<'_>,
    root: PathBuf,
    tasks_dir: Option<PathBuf>,
) -> PyResult<(String, String)> {
    py.allow_threads(|| {
        let tasks = task_map(tasks_dir.as_deref());
        let mut reports = vec![];
        let mut errors = vec![];
        for d in discover(&root) {
            let res = Trace::open(&d).and_then(|tr| {
                let t = tasks
                    .get(&tr.manifest.task)
                    .ok_or_else(|| core::Error::UnknownTask(tr.manifest.task.clone()))?;
                core::score(t, &tr)
            });
            match res {
                Ok(r) => reports.push(r),
                Err(e) => errors.push(serde_json::json!({"path": d, "error": e.to_string()})),
            }
        }
        Ok((
            serde_json::to_string(&reports).map_err(err)?,
            serde_json::to_string(&errors).map_err(err)?,
        ))
    })
}

/// Aggregate a JSON list of reports into a leaderboard (JSON).
#[pyfunction]
#[pyo3(signature = (reports_json, tasks_dir=None))]
fn aggregate(reports_json: &str, tasks_dir: Option<PathBuf>) -> PyResult<String> {
    let reports: Vec<Report> = serde_json::from_str(reports_json).map_err(err)?;
    let lb = core::aggregate(&reports, &task_map(tasks_dir.as_deref()));
    serde_json::to_string(&lb).map_err(err)
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(scorer_version, m)?)?;
    m.add_function(wrap_pyfunction!(spec_version, m)?)?;
    m.add_function(wrap_pyfunction!(list_tasks, m)?)?;
    m.add_function(wrap_pyfunction!(score_trace, m)?)?;
    m.add_function(wrap_pyfunction!(score_dir, m)?)?;
    m.add_function(wrap_pyfunction!(aggregate, m)?)?;
    Ok(())
}
