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
    /// Parts of the arena the capture could not see (e.g. outside a phone's frame).
    /// Excluded from coverage and ignored by the safety checks.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unobserved: Vec<Rect>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotSpec {
    /// Radius of the robot's footprint circle, used for boundary and obstacle checks.
    pub radius_m: f64,
    /// How far the footprint may overlap a wall or obstacle before it counts as a
    /// violation. Defaults to [`crate::score::CONTACT_TOLERANCE_M`]. Raise it for
    /// robots that touch walls by design (robot vacuums) or for coarse captures.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact_tolerance_m: Option<f64>,
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
    /// Move objects into zones; scored on the final world state. `zone` is the
    /// default target; an object may name its own (e.g. sorting by colour).
    Delivery {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        zone: Option<String>,
    },
    /// Sweep the free floor area with a tool of the given width. `edge_reach_m`
    /// models a side brush: free cells next to a wall or obstacle also count as
    /// swept when they are within this distance of the robot's centre.
    Coverage {
        cell_m: f64,
        tool_width_m: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        edge_reach_m: Option<f64>,
    },
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

/// Reference points. Every run is scored against the task's goal; these add context.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Reference {
    /// Chance level in task units, used when the data has no baseline runs.
    #[serde(default)]
    pub floor: f64,
    /// True when a person can do exactly the same job with the same means (for example
    /// driving the same robot on the same field), so a comparison with people is fair.
    /// False when the means differ (a robot vacuum and a person with a stick vacuum).
    #[serde(default)]
    pub people_comparable: bool,
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
    /// Target zone for this object; falls back to the goal's `zone`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zone: Option<String>,
    /// Free-form object type, e.g. `barrel-orange` (display only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<String>,
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
    /// The zone an object must end in (delivery tasks).
    pub fn target_zone(&self, o: &Object) -> Option<&Rect> {
        let default = match &self.goal {
            Goal::Delivery { zone } => zone.as_ref(),
            _ => None,
        };
        o.zone
            .as_ref()
            .or(default)
            .and_then(|z| self.arena.zones.get(z))
    }

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
            if let Some(z) = zone {
                if !self.arena.zones.contains_key(z) {
                    return Err(Error::InvalidTask(format!(
                        "{}: goal zone '{z}' not defined",
                        self.key()
                    )));
                }
            }
            for (seed, layout) in &self.layouts {
                for o in &layout.objects {
                    match o.zone.as_ref().or(zone.as_ref()) {
                        None => {
                            return Err(Error::InvalidTask(format!(
                                "{}: object '{}' in layout '{seed}' has no target zone",
                                self.key(),
                                o.id
                            )))
                        }
                        Some(z) if !self.arena.zones.contains_key(z) => {
                            return Err(Error::InvalidTask(format!(
                                "{}: object '{}' targets undefined zone '{z}'",
                                self.key(),
                                o.id
                            )))
                        }
                        _ => {}
                    }
                }
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
    (
        "MB-AIM-NAV@0.1",
        include_str!("../../../tasks/MB-AIM-NAV@0.1.json"),
    ),
    (
        "MB-AIM-SORT@0.1",
        include_str!("../../../tasks/MB-AIM-SORT@0.1.json"),
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
        assert_eq!(ts.len(), 5);
        for t in ts {
            assert!(t.free_area() > 0.0);
        }
    }
}
