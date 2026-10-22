use std::path::PathBuf;

/// Errors raised while loading tasks/traces or scoring.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("I/O error on {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("JSON error in {path}: {source}")]
    Json {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("CSV error in {path}: {source}")]
    Csv {
        path: PathBuf,
        #[source]
        source: csv::Error,
    },
    #[error("unknown task '{0}' (expected an id like MB-VEX-NAV@1.0)")]
    UnknownTask(String),
    #[error("task '{task}' has no layout for seed '{seed}' (hidden seeds are only available from the leaderboard server)")]
    UnknownSeed { task: String, seed: String },
    #[error("invalid trace: {0}")]
    InvalidTrace(String),
    #[error("invalid task: {0}")]
    InvalidTask(String),
}

pub type Result<T> = std::result::Result<T, Error>;
