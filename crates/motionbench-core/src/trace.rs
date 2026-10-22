use crate::error::{Error, Result};
use crate::task::Object;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

/// Who (or what) produced the run. Results are always reported per class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AgentClass {
    /// Teleoperated by a person. The reference class.
    Human,
    /// Autonomous code written by people (e.g. a team's auton routine).
    HumanAuton,
    /// Program or policy produced by an AI system.
    Ai,
    /// Human driving with machine assistance.
    Hybrid,
    /// A closed commercial robot (e.g. a robot vacuum) measured as-is.
    Product,
    /// Naive floor agent (idle, random walk). Defines S_floor.
    Baseline,
}

impl fmt::Display for AgentClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            AgentClass::Human => "human",
            AgentClass::HumanAuton => "human_auton",
            AgentClass::Ai => "ai",
            AgentClass::Hybrid => "hybrid",
            AgentClass::Product => "product",
            AgentClass::Baseline => "baseline",
        };
        f.write_str(s)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Agent {
    pub id: String,
    pub class: AgentClass,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub meta: serde_json::Value,
}

/// `manifest.json` at the root of a `.mbtrace` directory.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Manifest {
    #[serde(default = "default_spec")]
    pub spec_version: String,
    /// `ID@VERSION`
    pub task: String,
    pub seed: String,
    pub agent: Agent,
    #[serde(default)]
    pub trial: u32,
    /// `S` (simulation), `F` (field, self-reported) or `V` (verified).
    #[serde(default = "default_tier")]
    pub tier: String,
    /// `sim`, `video`, `onboard`, `synthetic`, …
    #[serde(default)]
    pub capture: String,
    #[serde(default)]
    pub notes: String,
    /// Optional SHA-256 of each data file, keyed by file name.
    #[serde(default)]
    pub files: BTreeMap<String, String>,
}

fn default_spec() -> String {
    crate::SPEC_VERSION.to_string()
}
fn default_tier() -> String {
    "S".to_string()
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct PoseSample {
    pub t_s: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub theta_rad: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Event {
    pub t_s: f64,
    /// `pickup`, `deliver`, `collision`, `rule_flag`, `note`, …
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub object_id: String,
    #[serde(default)]
    pub detail: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct WorldFinal {
    #[serde(default)]
    pub objects: Vec<Object>,
}

/// One run, loaded from a `.mbtrace` directory.
#[derive(Clone, Debug)]
pub struct Trace {
    pub manifest: Manifest,
    pub poses: Vec<PoseSample>,
    pub events: Vec<Event>,
    pub world_final: Option<WorldFinal>,
    pub path: Option<PathBuf>,
}

fn io(path: &Path) -> impl FnOnce(std::io::Error) -> Error + '_ {
    move |e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    }
}

fn read_csv<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<Vec<T>> {
    let mut rdr = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .comment(Some(b'#'))
        .from_path(path)
        .map_err(|e| Error::Csv {
            path: path.to_path_buf(),
            source: e,
        })?;
    rdr.deserialize()
        .collect::<std::result::Result<Vec<T>, _>>()
        .map_err(|e| Error::Csv {
            path: path.to_path_buf(),
            source: e,
        })
}

impl Trace {
    /// Open a `.mbtrace` directory: `manifest.json`, `poses.csv`, and optionally
    /// `events.csv` and `world_final.json`.
    pub fn open(dir: &Path) -> Result<Trace> {
        let mpath = dir.join("manifest.json");
        let ms = std::fs::read_to_string(&mpath).map_err(io(&mpath))?;
        let manifest: Manifest = serde_json::from_str(&ms).map_err(|e| Error::Json {
            path: mpath.clone(),
            source: e,
        })?;

        let poses = read_csv::<PoseSample>(&dir.join("poses.csv"))?;
        let epath = dir.join("events.csv");
        let events = if epath.exists() {
            read_csv::<Event>(&epath)?
        } else {
            vec![]
        };
        let wpath = dir.join("world_final.json");
        let world_final = if wpath.exists() {
            let s = std::fs::read_to_string(&wpath).map_err(io(&wpath))?;
            Some(serde_json::from_str(&s).map_err(|e| Error::Json {
                path: wpath.clone(),
                source: e,
            })?)
        } else {
            None
        };
        let t = Trace {
            manifest,
            poses,
            events,
            world_final,
            path: Some(dir.to_path_buf()),
        };
        t.check_hashes()?;
        t.validate()?;
        Ok(t)
    }

    /// Structural checks: non-empty, finite, strictly increasing time.
    pub fn validate(&self) -> Result<()> {
        if self.poses.len() < 2 {
            return Err(Error::InvalidTrace(
                "poses.csv needs at least 2 samples".into(),
            ));
        }
        for (i, p) in self.poses.iter().enumerate() {
            if !(p.t_s.is_finite()
                && p.x_m.is_finite()
                && p.y_m.is_finite()
                && p.theta_rad.is_finite())
            {
                return Err(Error::InvalidTrace(format!(
                    "non-finite value in pose row {i}"
                )));
            }
            if i > 0 && p.t_s <= self.poses[i - 1].t_s {
                return Err(Error::InvalidTrace(format!(
                    "time not strictly increasing at pose row {i}"
                )));
            }
        }
        if !self.manifest.task.contains('@') {
            return Err(Error::InvalidTrace(format!(
                "manifest.task '{}' must be ID@VERSION",
                self.manifest.task
            )));
        }
        Ok(())
    }

    /// If the manifest lists file hashes, verify them.
    pub fn check_hashes(&self) -> Result<()> {
        let Some(dir) = &self.path else { return Ok(()) };
        for (name, want) in &self.manifest.files {
            let p = dir.join(name);
            let bytes = std::fs::read(&p).map_err(io(&p))?;
            let got = format!("{:x}", Sha256::digest(&bytes));
            if !got.eq_ignore_ascii_case(want) {
                return Err(Error::InvalidTrace(format!("sha256 mismatch for {name}")));
            }
        }
        Ok(())
    }
}

/// Recursively find `.mbtrace` directories (any directory holding a `manifest.json`).
pub fn discover(root: &Path) -> Vec<PathBuf> {
    let mut out = vec![];
    fn walk(p: &Path, out: &mut Vec<PathBuf>) {
        if p.join("manifest.json").is_file() {
            out.push(p.to_path_buf());
            return;
        }
        if let Ok(rd) = std::fs::read_dir(p) {
            let mut kids: Vec<_> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            kids.sort();
            for k in kids {
                walk(&k, out);
            }
        }
    }
    walk(root, &mut out);
    out
}
