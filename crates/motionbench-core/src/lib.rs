//! # motionbench-core
//!
//! Deterministic scoring for MotionBench, an open benchmark for goal-directed
//! agents (people, robots running team-written or AI-written code, and people
//! assisted by robots) in **bounded spaces** (competition fields, rooms,
//! warehouse cells, yards), each scored against the task's goal.
//!
//! The model is simple:
//!
//! * a [`Task`] = arena + goal + constraints + budget, with seeded layouts;
//! * a [`Trace`] = one agent's run on one task: poses, events, final world state;
//! * [`score`] turns `(task, trace)` into a [`Report`];
//! * [`aggregate`] turns many reports into per-task boards scored against each task's goal ([`Leaderboard`]).
//!
//! Every binding (Python, WASM, …) calls into this crate so that a run scores
//! identically everywhere.

pub mod aggregate;
pub mod error;
pub mod geometry;
pub mod score;
pub mod stats;
pub mod task;
pub mod trace;

pub use aggregate::{aggregate, AgentRow, Leaderboard, PeopleReference, TaskBoard};
pub use error::{Error, Result};
pub use score::{score, Report};
pub use task::{builtin_tasks, find_task, Goal, Task};
pub use trace::{AgentClass, Manifest, Trace};

/// Version of the MotionBench spec this crate implements (task, trace, report schemas).
pub const SPEC_VERSION: &str = "0.2";

/// Version of this scorer. Reports record it so results can be traced to exact code.
pub const SCORER_VERSION: &str = env!("CARGO_PKG_VERSION");
