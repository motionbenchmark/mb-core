# MotionBench

**An open benchmark for how well agents complete goals in bounded spaces.**
Score any run — a student driving a VEX robot, a team's autonomous routine, an
AI-written program, a driver with robot assistance, a robot vacuum — against the
task's goals, measured the same way every time.

[![License](https://img.shields.io/badge/code-Apache--2.0-blue)](LICENSE)
[![Data](https://img.shields.io/badge/data-CC%20BY%204.0-green)](data/synthetic-v0.1/README.md)
[![Spec](https://img.shields.io/badge/spec-v0.2%20draft-orange)](rfcs/0002-goal-referenced-scoring.md)

> **Scope.** MotionBench is for agents that move inside a *bounded* space towards
> an *explicit*, measurable goal: competition fields, classroom tables, rooms,
> warehouse cells. It is **not** a benchmark for self-driving cars, road or
> sidewalk vehicles, or open-airspace drones, and nothing here is a safety
> certification. Read the [manifesto](MANIFESTO.md) for why that line matters.

> **Paper (draft for comment):** [paper/](paper/README.md) - methodology and preliminary results. **Protocol:** [docs/protocol.md](docs/protocol.md).

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
   aggregate ──► per-task board: goal score, MotionBench Index (MBI), chance level,
                 95% CIs, worst-of-N, shared ranks
```

* **Goal score** = the share of the task's goal a run achieved (waypoints reached,
  objects delivered, floor covered), from 0 to 100%. Every agent — a person, team-written
  code, AI-written code, a person with robot assistance, a commercial robot — is scored
  against the same goal.
* **Chance level** = what a random walk achieves on the same task, shown beside every
  board so a score can be read against luck.
* **MBI** = 0.5·goal + 0.2·path efficiency + 0.15·consistency + 0.15·safety, all on
  [0, 1]. It ranks agents within one task; MotionBench never ranks across tasks.
* **People comparison (optional).** Where a person can do exactly the same job with the
  same means (students driving the same VEX robot their code drives), the board also
  reports each agent against the median person. Where the means differ (a robot vacuum
  and a person with a stick vacuum), it doesn't.

Full rules: [RFC-0001](rfcs/0001-motionbench.md) (spec v0.1),
[RFC-0002](rfcs/0002-goal-referenced-scoring.md) (scoring in spec v0.2) and
[docs/spec](docs/spec).

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
tracker finds the robot in every frame, with about 1 cm median position error (90% within 1.6 cm).
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

### Code vs drivers, in simulation

Define a game once (`games/<name>/game.json`), then run your VEXcode AIM Python routine
against it hundreds of times, play the same game with simulated drivers (reaction time,
steering, precision, decisions, mistakes), and get a verdict before the code ever runs on
the robot. See [docs/games.md](docs/games.md).

```bash
motionbench game simulate games/aim-sort --code my_routine.py --layout dev-01 -o runs/try1 --video try1.mp4
motionbench game verdict games/aim-sort --layout dev-01 --code v1.py --code v2.py
```

### Multi-robot matches

Four robots, two alliances, one fixed broadcast camera: simulate matches with full
ground truth, render them as event-style footage, report each robot by period, and
benchmark trackers against the ground truth. See [docs/match.md](docs/match.md).

```bash
motionbench match synth -o data/match-sim-v0.1 --seeds 1-5     # synthetic matches (traces + events)
motionbench match render data/match-sim-v0.1/m0001 -o m0001.mp4  # broadcast-style video + m0001.gt.csv
motionbench match report data/match-sim-v0.1/m0001 --png paths.png
motionbench match track m0001.mp4 -o tracks.csv                  # baseline tracker, no tags
motionbench match eval data/match-sim-v0.1/m0001 tracks.csv      # recall, error, ID switches, IDF1
```

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

**MB-VEX-NAV@1.0** (goal: 6 waypoints; drivers use the same robot, so people are compared too: median of 12 simulated drivers = 5.5):

| Agent | Class | Goal | Worst of 10 | MBI | vs median driver |
| --- | --- | --- | --- | --- | --- |
| LLM-written auton, round 10 | ai | 100% | 100% | 0.99 | 1.09 |
| Team auton A (odometry + GPS) | human_auton | 100% | 100% | 0.99 | 1.09 |
| Driver with auto-align | hybrid | 100% | 100% | 0.97 | 1.09 |
| Team auton B (odometry only) | human_auton | 82% | 17% | 0.84 | 0.89 |
| LLM-written auton, round 1 | ai | 35% | 0% | 0.42 | 0.38 |
| Team auton C (timed, open loop) | human_auton | 20% | 0% | 0.36 | 0.22 |

**MB-COVER@0.1** (goal: the whole free floor; chance, a random walk, covers 49%. No robot-versus-person comparison: the means differ):

| Agent | Class | Goal (floor covered) | MBI |
| --- | --- | --- | --- |
| Robot vacuum, lane-planning | product | 95% | 0.86 |
| LLM-written planner, round 3 | ai | 90% | 0.90 |
| Robot vacuum, bump-and-turn | product | 59% | 0.53 |

**MB-AIM-SORT@0.1** (goal: 6 barrels in their colour zones; median of 10 simulated students = 4.7):

| Agent | Class | Goal | Worst of 5 | MBI | vs median student |
| --- | --- | --- | --- | --- | --- |
| Class code B (Python, AI Vision) | human_auton | 100% | 100% | 0.99 | 1.28 |
| LLM-written Python, round 3 (AI Vision) | ai | 100% | 100% | 0.99 | 1.28 |
| Student with AI Vision auto-aim | hybrid | 100% | 100% | 0.98 | 1.28 |
| Class code A (Blocks, gyro and encoders) | human_auton | 43% | 33% | 0.65 | 0.55 |
| LLM-written Python, round 1 | ai | 23% | 17% | 0.55 | 0.30 |

The AIM agents' speeds and errors are assumptions for a small omni-drive robot,
not measurements of VEX AIM.

Full board: [results/leaderboard-synthetic-v0.1.json](results/leaderboard-synthetic-v0.1.json).
Two method lessons already visible: the goal saturates on NAV for strong agents (so
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
  motionbench/tracker/      AprilTag and tag-free trackers, renderer, tag sheets
  motionbench/match/        multi-robot matches: simulator, broadcast renderer, report, tracker, benchmark
  motionbench/game/         games: VEX AIM simulator running VEXcode Python, simulated drivers, verdicts, video
games/                    game definitions (game.json) and sample routines
tasks/                    task specs (ID@VERSION.json)
data/synthetic-v0.1/      570 synthetic traces (CC BY 4.0)
data/match-sim-v0.1/      5 synthetic 2 v 2 matches with ground truth (CC BY 4.0)
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
