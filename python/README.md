# MotionBench

**An open benchmark for goal-directed robots in confined spaces.**
Score any run — a student driving a VEX robot, a team's autonomous routine, an
AI-written program, a robot vacuum — against human reference runs on the same task.

[![License](https://img.shields.io/badge/code-Apache--2.0-blue)](LICENSE)
[![Data](https://img.shields.io/badge/data-CC%20BY%204.0-green)](data/synthetic-v0.1/README.md)
[![Spec](https://img.shields.io/badge/spec-v0.1%20draft-orange)](rfcs/0001-motionbench.md)

> **Scope.** MotionBench is for machines that move inside a *bounded* space towards
> an *explicit* goal that a person could also do: competition fields, rooms,
> warehouse cells, yards. It is **not** a benchmark for self-driving cars, road or
> sidewalk vehicles, or open-airspace drones, and nothing here is a safety
> certification. Read the [manifesto](MANIFESTO.md) for why that line matters.

## How it works

```
 phone video / robot log / simulator
            │
            ▼
   .mbtrace  (poses.csv, events.csv, world_final.json, manifest.json)
            │
            ▼
   motionbench-core (Rust)  ──►  report: success · efficiency · precision
            │                           coverage · safety · process
            ▼
   aggregate ──► leaderboard: human-normalized score (HNS), MotionBench Index (MBI),
                 95% CIs, worst-of-N, shared ranks
```

* **HNS** = (agent − floor) / (median human − floor). 1.0 means "as good as the
  median human on this task"; the floor is a random-walk baseline.
* **MBI** = 0.5·HNS + 0.2·efficiency HNS + 0.15·consistency + 0.15·safety
  (HNS clipped to [0, 2]; weights frozen per spec major version).

Full rules: [RFC-0001](rfcs/0001-motionbench.md) and [docs/spec](docs/spec).

## Install

```bash
pip install motionbench                 # scorer + synthetic generator
pip install "motionbench[tracker]"      # + overhead-camera AprilTag tracker
cargo install motionbench-cli           # the `mb` command (Rust)
```

From source:

```bash
cargo test                               # Rust core + CLI
pip install maturin && cd python && maturin develop --release
pytest python/tests
```

## Quickstart

```bash
motionbench tasks                                   # list tasks
motionbench synth --out data/synthetic-v0.1         # regenerate the synthetic dataset
motionbench score data/synthetic-v0.1/MB-VEX-NAV@1.0/dev-01/driver-01/trial-0
motionbench aggregate data/synthetic-v0.1 -o results/leaderboard.json
motionbench viz data/synthetic-v0.1 -o explorer.html     # open in a browser to inspect and verify
```

`motionbench viz` writes one self-contained HTML page for any dataset folder (synthetic
or real). It replays every run on its arena, overlays trials or agents, and shows the
scorer's report. It also re-counts waypoints, deliveries and coverage from the raw poses
in the browser and runs physical sanity checks (time gaps, speed, leaving the arena,
budget overruns), flagging any run where the two disagree.

```python
import motionbench as mb

report = mb.score("data/synthetic-v0.1/MB-VEX-PICK@1.0/dev-01/llm-auton-r10/trial-0")
print(report["success"]["score"], report["efficiency"]["path_efficiency"])

reports, errors = mb.score_dir("data/synthetic-v0.1")
board = mb.aggregate(reports)
```

### Score a real run from video (VEX)

```bash
motionbench field --vex -o field.json        # tag layout for a 12 ft field
motionbench tags --config field.json --out tags/   # print these at 100% scale
# tape tags 0-3 in the field corners, tag 10 on the robot, film from a tripod
motionbench track run.mp4 --config field.json --task MB-VEX-NAV@1.0 --seed dev-01 \
    --agent-id driver-07 --agent-class human -o runs/driver-07-t0
motionbench score runs/driver-07-t0
```

See [docs/capture-guide.md](docs/capture-guide.md). On rendered test footage the
tracker recovers the robot in 100% of frames with sub-centimetre mean position error.
No tags? Score an ordinary phone video:

```bash
pip install "motionbench[tracker]"
motionbench calibrate run.mov -o calib.html      # tap the floor's corners, download calib.json
motionbench track run.mov --untagged --calib calib.json -o out/ --overlay
```

Use it from an AI assistant with the MCP server (`pip install "motionbench[tracker,mcp]"`,
command `motionbench-mcp`): it can inspect a video, build the calibration, convert,
score and submit a run to the project site for review.

```json
{"mcpServers": {"motionbench": {"command": "motionbench-mcp"}}}
```

Onboard logging for VEX V5 (PROS): [integrations/vex/pros](integrations/vex/pros).

## Tasks (v0.1)

| Task | Domain | Goal | Budget | Public seeds |
| --- | --- | --- | --- | --- |
| `MB-VEX-NAV@1.0` | VEX 12 ft field | 6 waypoints in order, ±5 cm / ±5° | 20 s | dev-01..03 |
| `MB-VEX-PICK@1.0` | VEX 12 ft field | Deliver 6 objects to a corner zone | 60 s | dev-01..02 |
| `MB-COVER@0.1` | 4 m × 3 m room | Sweep free floor with a 30 cm tool | 180 s | dev-01..02 |
| `MB-AIM-NAV@0.1` | 6 × 4 ft table, VEX AIM | 5 waypoints in order, ±3 cm / ±10° | 20 s | dev-01..02 |
| `MB-AIM-SORT@0.1` | 6 × 4 ft table, VEX AIM | Sort 3 orange + 3 blue barrels into their colour zones | 45 s | dev-01..02 |

VEX tasks are game-agnostic primitives; they do not reproduce any season's game.
The VEX AIM tasks use a MotionBench-defined 6 × 4 ft arena (tape it out on a table or
floor), not an official VEX field, and the barrels that come with the AIM kit.

## First results — synthetic data only

> These numbers come from **simulated** agents (`motionbench.synth`) and exist to
> exercise the pipeline. They are not evidence about real robots or real people.

**MB-VEX-NAV@1.0** (median of 12 simulated drivers = 5.5 waypoints):

| Agent | Class | Mean | Worst of 10 | HNS | MBI |
| --- | --- | --- | --- | --- | --- |
| LLM-written auton, round 10 | ai | 6.0 | 6 | 1.09 | 1.11 |
| Team auton A (odometry + GPS) | human_auton | 6.0 | 6 | 1.09 | 1.10 |
| Driver with auto-align | hybrid | 6.0 | 6 | 1.09 | 1.08 |
| Team auton B (odometry only) | human_auton | 4.9 | 1 | 0.89 | 0.95 |
| LLM-written auton, round 1 | ai | 2.1 | 0 | 0.38 | 0.49 |
| Team auton C (timed, open loop) | human_auton | 1.2 | 0 | 0.22 | 0.42 |

**MB-COVER@0.1** (median of 6 simulated people = 86.9% covered; random-walk floor 49.2%):

| Agent | Class | Covered | HNS | MBI |
| --- | --- | --- | --- | --- |
| Robot vacuum, lane-planning | product | 94.7% | 1.21 | 1.03 |
| LLM-written planner, round 3 | ai | 89.7% | 1.08 | 1.04 |
| Robot vacuum, bump-and-turn | product | 59.0% | 0.26 | 0.40 |

**MB-AIM-SORT@0.1** (VEX AIM classroom roster; median of 10 simulated students = 4.7 of 6 barrels sorted):

| Agent | Class | Mean | Worst of 5 | HNS | MBI |
| --- | --- | --- | --- | --- | --- |
| Class code B (Python, AI Vision) | human_auton | 6.0 | 6 | 1.28 | 1.17 |
| LLM-written Python, round 3 (AI Vision) | ai | 6.0 | 6 | 1.28 | 1.17 |
| Student with AI Vision auto-aim | hybrid | 6.0 | 6 | 1.28 | 1.16 |
| Class code A (Blocks, gyro and encoders) | human_auton | 2.6 | 2 | 0.55 | 0.76 |
| LLM-written Python, round 1 | ai | 1.4 | 1 | 0.30 | 0.63 |

The AIM agents' speeds and errors are assumptions for a small omni-drive robot,
not measurements of VEX AIM.

Full board: [results/leaderboard-synthetic-v0.1.json](results/leaderboard-synthetic-v0.1.json).
Two method lessons already visible: success saturates on NAV for strong agents (so
efficiency and time decide), and Team auton B's worst run (1 of 6) shows why
worst-of-N is reported next to the mean. On AIM, the open-loop Blocks program is
near the top for navigation but falls behind on sorting, where small drift makes it
miss barrels: the task, not the robot, decides what matters.

## Repository layout

```
crates/motionbench-core   Rust scorer (tasks, traces, scoring, statistics, aggregation)
crates/motionbench-cli    `mb` command
python/                   PyO3 bindings + motionbench Python package
  motionbench/synth.py      synthetic dataset generator
  motionbench/tracker/      AprilTag overhead-camera tracker, renderer, tag sheets
tasks/                    task specs (ID@VERSION.json)
data/synthetic-v0.1/      570 synthetic traces (CC BY 4.0)
results/                  leaderboards and per-run reports
integrations/vex/pros/    onboard pose logger for VEX V5 (PROS)
rfcs/                     RFC process and RFC-0001
docs/                     spec and capture guide
```

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) and [GOVERNANCE.md](GOVERNANCE.md). New tasks
and scoring changes go through an [RFC](rfcs/README.md). Pilot teams, coaches and
researchers: reach out via the project site.

## Citation

See [CITATION.cff](CITATION.cff).

## License

Code: [Apache-2.0](LICENSE). Tasks, data and documentation: CC BY 4.0. See [NOTICE](NOTICE).
VEX and V5 are trademarks of their owners; MotionBench is an independent project
and is not affiliated with or endorsed by any competition organizer or manufacturer.
