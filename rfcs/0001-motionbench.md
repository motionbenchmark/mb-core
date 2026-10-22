# RFC-0001: MotionBench — an open benchmark for goal-directed robots in confined spaces

| Field | Value |
| --- | --- |
| RFC | 0001 |
| Title | MotionBench project proposal and core specification v0.1 |
| Status | **Draft — open for comment** |
| Type | Process + Standards Track |
| Created | 2026-10-22 |
| Author | Shekhar (23v) |
| Discussion | GitHub Discussions, category "RFCs" |
| Supersedes | — |
| License | CC BY 4.0 (this document); Apache-2.0 (code it describes) |

The key words MUST, MUST NOT, REQUIRED, SHOULD, SHOULD NOT and MAY are to be
interpreted as described in RFC 2119.

This RFC follows the shape of an Apache Software Foundation incubation proposal
(Part I) followed by a technical specification (Part II). MotionBench is not an ASF
project; we adopt the ASF's proven structure for proposing, governing and growing
an open-source community, and leave the door open to a foundation home later.

---

## Part I — Proposal

### Abstract

MotionBench is an open benchmark, dataset and scoring library that measures how
well goal-directed robots perform tasks inside bounded, confined spaces, and
reports every result relative to human reference runs on the same task. It starts
with VEX robotics fields and generalizes to any arena-bounded robot: robot
vacuums, lawn mowers, warehouse robots, classroom arms and simulated agents.

### Proposal

We propose to build and maintain, in the open:

1. **A specification** for tasks (arena, goal, constraints, budget, seeded
   layouts), traces (what happened in a run) and reports (how it scored).
2. **A reference scorer** in Rust (`motionbench-core`) with bindings for Python
   (`motionbench` on PyPI), and later WebAssembly, so that a run scores
   identically in every language.
3. **Capture tools**: an overhead-camera tracker using printed AprilTags, onboard
   logging snippets for robot controllers, and a simulator for synthetic runs.
4. **Task suites**, starting with game-agnostic VEX tasks and a room-coverage task.
5. **A public dataset** of human and machine runs, and a leaderboard.

### Background

Robotics evaluation splits into two worlds. Simulation benchmarks are
reproducible but rarely touch real hardware or real people. Real-world
competitions involve thousands of real robots and drivers but produce match
scores that cannot be compared across events, methods or years. Consumer robots
are judged by marketing claims or by lab standards that test a component (for
example, debris pickup on a straight line) rather than a whole job.

Meanwhile AI systems now write robot programs. Students, coaches and researchers
are all asking the same question: is the AI-written routine better than the
team's own code, or than a good human driver? There is no shared, cheap, honest
way to answer it.

Text-model benchmarks such as LoCoMo show what a good open benchmark looks like:
a fixed task set, a versioned public dataset, scripts anyone can run, and results
that cite an exact data hash. MotionBench brings that discipline to embodied
agents in confined spaces.

### Rationale

**Why confined spaces.** A bounded arena with an explicit goal is closed enough
that runs can be reproduced, a human can perform the same task as a reference,
and participation needs only a phone and printed tags. Self-driving cars, road
and sidewalk vehicles, and open-airspace drones are explicitly out of scope: they
are open-world, life-safety problems with their own regulators and benchmarks,
and mixing them in would dilute both the method and the message. See
[MANIFESTO.md](../MANIFESTO.md) for the full scope test.

**Why human-normalized.** Absolute scores mean nothing across tasks. "1.0 = the
median human on this task" is understandable by a student, a parent, a researcher
and a product reviewer alike.

**Why VEX first.** The VEX V5 programme has thousands of teams worldwide playing
on an identical 12 ft x 12 ft field, and its matches already split into an
autonomous period and a driver-controlled period: the human-vs-machine comparison
is built into the sport. Our VEX tasks are game-agnostic primitives (navigate,
collect, deliver) so they do not reproduce any season's game.

**Why Rust core plus bindings.** A single implementation of the scoring rules
avoids the "Python says 0.81, JavaScript says 0.80" problem. Rust gives a small,
fast, dependency-light core that compiles to native, Python and WebAssembly.

### Initial goals (first 90 days)

- Spec v0.1 (this document) reviewed and accepted.
- `motionbench-core` 0.1 and `motionbench` 0.1 published to crates.io and PyPI.
- Three tasks: `MB-VEX-NAV@1.0`, `MB-VEX-PICK@1.0`, `MB-COVER@0.1`.
- A synthetic dataset (330 runs) that exercises the full pipeline.
- Human baselines from 20+ real drivers across 3-5 pilot teams.
- A first report: human drivers vs team-written vs AI-written routines, with
  confidence intervals, reviewed by the pilot teams before publication.

### Current status

**Meritocracy.** Contributors who make sustained, high-quality contributions
(code, tasks, data collection, documentation, reviews, community support) are
invited to become committers, and committers to join the Project Management
Committee (PMC). Contribution is recognized in every form, not only code.

**Community.** The initial community is the founding team, pilot VEX teams and
their coaches. We will grow it through the VEX community forums and team
Discords, robotics research venues, and robot-product reviewers.

**Core developers.** Initially a small founding team. Our explicit goal is to
have committers from at least three unaffiliated organizations within the first
year, including at least one coach and one student-team alumnus.

**Alignment.** MotionBench is complementary to official competition scoring and
to existing component-level product standards; it does not replace either.

### Known risks

| Risk | Mitigation |
| --- | --- |
| **Orphaned project.** Founders move on. | Apache-2.0 code, CC BY data, docs-first culture, no single-vendor infrastructure; static site deployable by anyone. |
| **Inexperience with open source governance.** | Adopt this ASF-style process from day one: public RFCs, lazy consensus, recorded votes. |
| **Homogeneous contributors.** | Steering seats reserved for a coach, a student representative, a researcher and an industry representative. |
| **Reliance on salaried developers.** | Keep the core small; recruit maintainers from pilot teams and universities. |
| **Intellectual property of competition owners.** | Game-agnostic tasks only; no copied rules, field art or trademarks; season-specific suites only with written permission from the competition owner. |
| **Privacy of minors.** | Trace-only public data by default, guardian consent for video, no faces required. |
| **Gaming the leaderboard.** | Hidden seeds revealed at run time, video for verified tier, worst-of-N reporting, review of outliers. |
| **Name confusion.** An unrelated 2025 video-understanding benchmark also uses the name "MotionBench". | Always present as "MotionBench — robots in confined spaces"; package names, domain and DOI disambiguate. Revisit before 1.0. |
| **Over-claiming.** "AI beats humans" headlines from small samples. | Results publish sample sizes and CIs; synthetic data always labelled; the manifesto's promises are binding on maintainers. |

### Documentation

`README.md`, `MANIFESTO.md`, `docs/spec/*`, `docs/capture-guide.md`, this RFC,
and API docs generated from the Rust and Python sources.

### Initial source

The repository at the time of this RFC contains: the Rust scorer and `mb` CLI,
PyO3 bindings, the trace writer, the synthetic generator, the AprilTag tracker
and its synthetic-video renderer, a PROS (VEX V5) logging header, three task
specs, the synthetic dataset and its leaderboard, and CI.

### Source and IP submission status

All initial code was written for this project and is contributed under Apache-2.0.
Task specifications and data are CC BY 4.0. Contributors sign off commits under
the Developer Certificate of Origin (DCO); no CLA is required.

### External dependencies

Rust: serde, serde_json, csv, thiserror, sha2, clap, pyo3 (all MIT/Apache-2.0).
Python: numpy (BSD), optional OpenCV contrib (Apache-2.0) for the tracker.

### Cryptography

SHA-256 is used only for integrity hashes of trace files. No encryption.

### Required resources

- Git repository: `github.com/motionbenchmark/mb-core` (code, tasks, data).
- Issue tracker and Discussions on the same repository.
- Project site in a separate repository (`motionbenchmark/mb-web`) for the leaderboard, downloads and sign-ups.
- Release artifacts on GitHub Releases, PyPI and crates.io.
- A community chat (Discord) and a low-volume announcements list.

### Initial committers

- Shekhar (23v) — founder
- *Open: pilot team mentors, a research lead, a tracker maintainer*

### Sponsors

To be sought: a university robotics lab or education nonprofit as fiscal and
academic home; competition owners as optional partners for season suites.

---

## Part II — Specification v0.1

### 1. Concepts

| Object | Definition |
| --- | --- |
| **Task** | Arena + Goal + Constraints + Budget, identified as `ID@VERSION` (e.g. `MB-VEX-NAV@1.0`), with one layout per seed. |
| **Seed** | Names a layout (start pose, waypoints, object positions). `dev-*` seeds are public; `test-*` seeds are hidden and issued by the leaderboard server at run time. |
| **Agent** | Whatever produced the run, tagged with exactly one class: `human`, `human_auton`, `ai`, `hybrid`, `product`, `baseline`. |
| **Trace** | One agent's run on one task and seed (`.mbtrace` directory). |
| **Report** | The scorer's output for one trace. |
| **Leaderboard** | Aggregation of reports, normalized against human runs. |

### 2. Task format

A task is a JSON document (see `tasks/` and `docs/spec/task-format.md`):

- `arena`: `width_m`, `height_m`, `obstacles` (axis-aligned rectangles), `zones`.
- `robot.radius_m`: footprint radius used for boundary and obstacle checks.
- `budget.time_s`: hard time limit; samples after it MUST be ignored.
- `goal.kind`: one of `waypoints`, `delivery`, `coverage` (v0.1).
- `reference.floor`: fixed floor score, used when no baseline runs exist.
- `layouts`: map of seed name to layout.

A published task version MUST NOT change. Any change to geometry, goal, budget or
layouts creates a new version.

### 3. Trace format

A `.mbtrace` is a directory containing:

| File | Required | Content |
| --- | --- | --- |
| `manifest.json` | yes | `spec_version`, `task`, `seed`, `agent {id, class, name, meta}`, `trial`, `tier` (`S`/`F`/`V`), `capture`, `notes`, `files` (SHA-256 per data file) |
| `poses.csv` | yes | `t_s,x_m,y_m,theta_rad`; metres and radians in the arena frame; time strictly increasing; extra columns ignored |
| `events.csv` | no | `t_s,type,object_id,detail`; types include `pickup`, `deliver`, `collision`, `rule_flag`, `note` |
| `world_final.json` | no | `{"objects":[{"id","x","y"}]}` final positions; REQUIRED for verified delivery runs |

If `files` lists hashes, the scorer MUST reject a trace whose files do not match.

### 4. Scoring

Time is re-based to the first pose sample. Samples after the budget are dropped.

**Success** by goal kind:

- *waypoints*: waypoints must be reached **in order**; a waypoint counts on the
  first sample within `pos_tol_m` and `heading_tol_rad`. Score = waypoints reached.
- *delivery*: an object counts if its final position lies inside the goal zone
  (from `world_final.json`; falling back to `deliver` events with a warning).
- *coverage*: the arena is rasterized at `cell_m`; a free cell is covered when the
  tool disc (diameter `tool_width_m`) passes over its centre. Score = % of free
  cells covered. "Completed" means at least 90%.

**Efficiency**: path length; oracle length (straight-line chain for waypoints;
greedy single-carry tour for delivery; covered area / tool width for coverage);
`path_efficiency = min(1, oracle / actual)` over the achieved part of the goal;
time-to-goal.

**Precision**: final-pose error, mean target error, mean heading error.

**Safety**: boundary contacts, obstacle intrusions (robot disc overlapping an
obstacle by more than 3 cm, counted as episodes), `collision` and `rule_flag`
events.

**Process**: mean and p99 speed, hesitation time (speed < 2 cm/s), RMS jerk.

### 5. Aggregation

Per task and agent, over all trials and public seeds:

- mean, median, standard deviation, worst and best success score;
- 95% percentile-bootstrap CI of the mean (2,000 resamples, SplitMix64 seeded by
  FNV-1a of `task/agent`, so every binding reproduces it exactly);
- completion rate, mean path efficiency, mean violations.

**Human reference**: the median (and 90th percentile, "expert") of per-driver mean
scores across agents of class `human`.

**Floor**: the median of per-agent mean scores across `baseline` agents if any are
present, otherwise `task.reference.floor`.

**Human-normalized score**:

```
HNS = (S_agent − S_floor) / (S_human_median − S_floor)
```

**MotionBench Index** (weights frozen for spec 0.x):

```
MBI = 0.50·clip(HNS) + 0.20·clip(HNS_eff) + 0.15·Consistency + 0.15·Safety
clip(x) = min(max(x, 0), 2)
HNS_eff = path_efficiency / median human path_efficiency
Consistency = clamp(1 − std/mean, 0, 1)
Safety = 1 / (1 + mean violations)
```

**Ranks are shared**: rows are sorted by mean success, and a new rank starts only
when an agent's CI upper bound falls below the lower bound of the current group's
leader.

### 6. Tiers

| Tier | Meaning | Leaderboard |
| --- | --- | --- |
| S | Simulation or synthetic | Separate board |
| F | Real hardware, self-reported trace and video | Main board, marked unverified until reviewed |
| V | Real hardware, hidden seed issued at run time, uncut video with on-screen code, reviewed | Main board |

### 7. Conformance

An implementation is conformant when, for every trace in the published
conformance set, its report and leaderboard JSON equal the reference
implementation's after parsing (the reference Python and Rust paths are
byte-identical today). Reports MUST record `spec_version` and `scorer_version`.

### 8. Versioning

- Spec: `MAJOR.MINOR`. Weights and normalization change only with MAJOR.
- Tasks: `ID@MAJOR.MINOR`; old versions remain scorable forever.
- Datasets: named with a version and published with SHA-256 checksums.

### 9. Alternatives considered

- **Score source code or robot designs**: rejected; ties the benchmark to
  platforms and invites IP problems. Traces are universal.
- **Absolute scores only**: rejected; not comparable across tasks or audiences.
- **Python-only scorer**: rejected; a single Rust core is the only way to keep
  web, Python and CLI results identical.
- **Include road vehicles and open-world drones**: rejected; see Rationale.

### 10. Unresolved questions

1. Should AI agents be allowed to train on public human traces, and if so is
   that a separate track?
2. How should hardware differences be handled: per-robot boards, or a "standard
   robot" simulator tier?
3. What is the minimum number of human drivers for a stable reference (proposal: 10)?
4. Which organization should become the long-term home?

---

## Process

This RFC is open for comment for **21 days** from its creation date. It is
accepted by lazy consensus of the initial committers unless a committer objects;
an objection triggers a vote (+1 / 0 / −1, majority of committers, at least three
binding +1 votes). Accepted RFCs are numbered, merged and never edited except for
typo fixes; later changes need a new RFC that supersedes this one.
