use crate::error::{Error, Result};
use crate::geometry::Rect;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

/// A benchmark task: a bounded arena, a goal, constraints and a budget,
/// plus one layout per public seed. Identified as `ID@VERSION`, e.g. `MB-VEX-NAV@1.0`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Task {
    pub id: String,
    pub version: String,
    pub title: String,
    /// Domain adapter, e.g. `vex`, `indoor-floor`, `warehouse`.
    pub domain: String,
    #[serde(default)]
    pub description: String,
    pub arena: Arena,
    pub robot: RobotSpec,
    pub budget: Budget,
    pub goal: Goal,
    #[serde(default)]
    pub reference: Reference,
    /// Public layouts keyed by seed name. Hidden test seeds live server-side only.
    pub layouts: BTreeMap<String, Layout>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Arena {
    pub width_m: f64,
    pub height_m: f64,
    #[serde(default)]
    pub obstacles: Vec<Rect>,
    #[serde(default)]
    pub zones: BTreeMap<String, Rect>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotSpec {
    /// Radius of the robot's footprint circle, used for boundary and obstacle checks.
    pub radius_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Budget {
    pub time_s: f64,
}

/// What "done" means. Each kind has its own scorer in [`crate::score`].
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Goal {
    /// Reach each waypoint in order, within position and heading tolerance.
    Waypoints {
        pos_tol_m: f64,
        heading_tol_rad: f64,
    },
    /// Move objects into a named zone; scored on the final world state.
    Delivery { zone: String },
    /// Sweep the free floor area with a tool of the given width.
    Coverage { cell_m: f64, tool_width_m: f64 },
}

impl Goal {
    pub fn kind(&self) -> &'static str {
        match self {
            Goal::Waypoints { .. } => "waypoints",
            Goal::Delivery { .. } => "delivery",
            Goal::Coverage { .. } => "coverage",
        }
    }
}

/// Reference points for normalization. Human references are computed from human
/// runs in the dataset at aggregation time; only the floor is fixed per task.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Reference {
    #[serde(default)]
    pub floor: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Layout {
    pub start: Pose,
    #[serde(default)]
    pub waypoints: Vec<Pose>,
    #[serde(default)]
    pub objects: Vec<Object>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Object {
    pub id: String,
    pub x: f64,
    pub y: f64,
}

impl Task {
    /// `ID@VERSION`
    pub fn key(&self) -> String {
        format!("{}@{}", self.id, self.version)
    }

    pub fn from_json(s: &str, origin: &Path) -> Result<Task> {
        let t: Task = serde_json::from_str(s).map_err(|e| Error::Json {
            path: origin.to_path_buf(),
            source: e,
        })?;
        t.validate()?;
        Ok(t)
    }

    pub fn load(path: &Path) -> Result<Task> {
        let s = std::fs::read_to_string(path).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        Task::from_json(&s, path)
    }

    pub fn layout(&self, seed: &str) -> Result<&Layout> {
        self.layouts.get(seed).ok_or_else(|| Error::UnknownSeed {
            task: self.key(),
            seed: seed.to_string(),
        })
    }

    /// Area of the arena not covered by obstacles (m²).
    pub fn free_area(&self) -> f64 {
        let obst: f64 = self.arena.obstacles.iter().map(|o| o.area()).sum();
        self.arena.width_m * self.arena.height_m - obst
    }

    pub fn validate(&self) -> Result<()> {
        if self.arena.width_m <= 0.0 || self.arena.height_m <= 0.0 {
            return Err(Error::InvalidTask(format!(
                "{}: arena must have positive size",
                self.key()
            )));
        }
        if self.budget.time_s <= 0.0 {
            return Err(Error::InvalidTask(format!(
                "{}: budget must be positive",
                self.key()
            )));
        }
        if let Goal::Delivery { zone } = &self.goal {
            if !self.arena.zones.contains_key(zone) {
                return Err(Error::InvalidTask(format!(
                    "{}: goal zone '{zone}' not defined",
                    self.key()
                )));
            }
        }
        if self.layouts.is_empty() {
            return Err(Error::InvalidTask(format!(
                "{}: needs at least one public layout",
                self.key()
            )));
        }
        Ok(())
    }
}

const BUILTIN: &[(&str, &str)] = &[
    (
        "MB-VEX-NAV@1.0",
        include_str!("../../../tasks/MB-VEX-NAV@1.0.json"),
    ),
    (
        "MB-VEX-PICK@1.0",
        include_str!("../../../tasks/MB-VEX-PICK@1.0.json"),
    ),
    (
        "MB-COVER@0.1",
        include_str!("../../../tasks/MB-COVER@0.1.json"),
    ),
];

/// Tasks compiled into this build of the scorer.
pub fn builtin_tasks() -> Vec<Task> {
    BUILTIN
        .iter()
        .map(|(k, s)| Task::from_json(s, Path::new(k)).expect("builtin task must parse"))
        .collect()
}

/// Find a task by `ID@VERSION`, first in `tasks_dir` (as `<key>.json`) if given,
/// then among the built-in tasks.
pub fn find_task(key: &str, tasks_dir: Option<&Path>) -> Result<Task> {
    if let Some(dir) = tasks_dir {
        let p = dir.join(format!("{key}.json"));
        if p.exists() {
            return Task::load(&p);
        }
    }
    builtin_tasks()
        .into_iter()
        .find(|t| t.key() == key)
        .ok_or_else(|| Error::UnknownTask(key.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtins_parse() {
        let ts = builtin_tasks();
        assert_eq!(ts.len(), 3);
        for t in ts {
            assert!(t.free_area() > 0.0);
        }
    }
}
