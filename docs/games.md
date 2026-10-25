# Games: code vs drivers, in simulation

Before a routine runs on the real robot, run it a few hundred times in
simulation, play the same game with simulated drivers, and get a verdict: is
this code better than the people who would otherwise drive?

A **game** is defined once, then simulated and scored as often as you like:

```
games/aim-sort/
  game.json        field, zones, AprilTags, objects, layouts, match length, scoring rules
  routines/        sample VEXcode AIM Python routines
```

| command | what it does |
|---|---|
| `motionbench game create games/my-game --id MY-GAME --title "..."` | start a definition to fill in |
| `motionbench game validate games/aim-sort` | check a definition |
| `motionbench game drivers` | list the simulated driver characters |
| `motionbench game simulate games/aim-sort --code my.py -o runs/try1 --video try1.mp4` | one run of a routine (or `--driver pro-priya`) |
| `motionbench game score games/aim-sort runs/try1` | game points for a run folder (simulated or tracked) |
| `motionbench game verdict games/aim-sort --code v1.py --code v2.py -o verdict/` | routines vs simulated drivers, many runs each |
| `motionbench game render games/aim-sort runs/try1 -o try1.mp4` | video of a run |

Everything this produces is **simulated** and labelled that way. A verdict
tells you which version to try on the real robot first; it is not a
measurement of a robot or a person.

## Creating a game

`game.json` (format `motionbench-game` 0.1) holds:

- `field`: size in metres, obstacles, scoring `zones` and wall-mounted AprilTags
  (`id`, position, which way it faces). Coordinates are metres from the near-left
  corner; headings are degrees clockwise from +y (the far wall).
- `objects`: the kinds of game objects (shape, radius, height, colour).
- `layouts`: starting positions of the robot and every object.
- `match`: length and format.
- `scoring.rules`: which object scores how many points in which zone. Objects
  count at rest after the match, centre inside the zone, not touching the robot.
- `motionbench_task` (optional): a MotionBench task with the same layouts, so
  every simulated run is also scored by the standard scorer.

For a team's own game, the definition is worked out from their code and a short
description of the game (what the field looks like, what scores), then checked
with `validate` and by simulating their routine. It is written once; versions of
the code are then compared against it.

`games/aim-sort` is the first game: a VEX AIM robot sorts three orange and three
blue barrels into their colour zones in 45 seconds. It uses the layouts of
MotionBench task MB-AIM-SORT@0.1, and the game score and the standard scorer
agree on every run.

## The VEX AIM simulator

Routines run unchanged: the VEXcode header (`from vex import *`, `robot=Robot()`)
gets the simulator's stand-ins. Covered:

- motion: `move_at`, `move_for`, `move_with_vectors`, `turn`, `turn_for`, `turn_to`,
  `stop_all_movement`, `set_move_velocity`, `set_turn_velocity`, `set_xy_position`,
  `get_x_position`, `get_y_position`, `is_move_active`, `is_turn_active`, `is_stopped`
- `robot.inertial`: `get_heading`, `get_rotation`, `get_yaw`, `set_heading`
- `robot.kicker`: `kick(SOFT|MEDIUM|HARD)`, `place()`
- sensing: `has_sports_ball`, `has_orange_barrel`, `has_blue_barrel`, `has_any_barrel`
- `robot.vision.get_data(...)` for `ORANGE_BARREL`, `BLUE_BARREL`, `SPORTS_BALL`,
  `ALL_TAGS`, `TAG0`-`TAG37`: `exists`, `bearing`, `width`, `height`, `centerX`, `centerY`, `id`
- `wait(t, MSEC|SECONDS)`, `robot.timer`; screen, sound and LEDs are accepted and ignored

Not yet: threads and controller events.

Speeds follow the API (100% = 200 mm/s, turns up to 150 deg/s). Each run draws
its own imperfections: wheel slip (the robot goes 2.5% more or less than its
encoders say, typically), sideways drift, turn overshoot, kick scatter, and vision
noise and missed detections. Code that only counts wheel turns accumulates those
errors; code that looks at the field can correct them. The robot's footprint,
accelerations, the camera's field of view (60 degrees) and friction are
assumptions, not AIM specifications.

The match clock is enforced: when time runs out the routine is stopped wherever
it is. Every robot call costs simulated time (getters about a millisecond, a
camera frame 33 ms), so busy loops burn the clock like they do on the robot. A
routine that crashes stops the robot, and the error is reported.

## Simulated drivers

Each driver character has a reaction time (they react to what the field looked
like a moment ago, partly anticipating), a share of full speed they dare to use,
steering shake, precision when lining up and placing, decision quality (choosing
a sensible next object), a mistake rate (letting go early) and a hesitation rate.

| character | reaction | speed | shake | precision | decisions | mistakes |
|---|---|---|---|---|---|---|
| Rookie Riley | 0.55 s | 55% | 9 deg | 0.30 | 0.55 | 16% |
| Nervous Noah | 0.45 s | 60% | 5 deg | 0.60 | 0.70 | 7% |
| Average Alex | 0.40 s | 70% | 5 deg | 0.55 | 0.75 | 8% |
| Speedy Sloane | 0.30 s | 100% | 7 deg | 0.40 | 0.70 | 13% |
| Steady Sam | 0.35 s | 55% | 3 deg | 0.85 | 0.85 | 3% |
| Pro Priya | 0.22 s | 95% | 2 deg | 0.85 | 0.95 | 2% |

A made-up class of 20 drivers, with traits spread around these, gives the
"typical student": the median of the class. These are models chosen to span a
classroom, not measurements of anyone. Calibrating them against real drivers (a
few filmed runs per student) is the obvious next step.

## The verdict

The drivers are a **parameterized, uncalibrated baseline**: a verdict says the code beats
(or doesn't beat) these simulated drivers, not real people.

- **Paired runs.** Run *i* of every agent uses the same seed: the same layout and the same
  draw of robot imperfections. Comparisons are means of paired differences.
- **Public and unseen layouts.** 20 runs on the public layout, and 20 spread over 10
  generated layouts (official verdicts keep that seed private).
- **Resampling unit.** Public: paired runs. Unseen: layouts first, then runs within them,
  so the range covers variation across layouts.
- **Four outcomes**, with an equivalence margin of 0.5 points: **better** or **worse** when
  the 95% range excludes zero; **equivalent** when the range lies inside ±0.5; **no
  difference found** otherwise. No difference found is not the same as equivalent.
- A routine's **change from public to unseen** is a comparison with itself and does not
  depend on the drivers at all.

The four sample routines (45 s, 6 points possible; `results/aim-sort-verdict-v0.1.json`):

| routine | public | vs typical driver | unseen | vs typical driver | change |
|---|---|---|---|---|---|
| `deadreckon_v1.py` | 1.70 | worse (-1.20) | 0.65 | worse (-2.20) | worse (-1.05) |
| `vision_v2.py` | 2.45 | worse (-0.45) | 2.40 | no difference found (-0.45) | equivalent (-0.05) |
| `vision_tags_v3.py` | 2.15 | worse (-0.75) | 1.85 | worse (-1.00) | no difference found (-0.30) |
| `planner_v4.py` | 4.00 | better (+1.10) | 2.80 | no difference found (-0.05) | worse (-1.20) |

The typical simulated driver scores 2.90 on the public layout and 2.85 on unseen
layouts; the six characters range from 2.00 to 3.75 (public) and 2.10 to 4.15 (unseen).
The planner beats every simulated character on the public layout and none of the two
best on unseen layouts: its plan encodes where the barrels are. The routine that searches
with the camera holds its score. Two other findings a real run would also show: the
tag-homing routine wastes time searching for the tag, and routines that drop every barrel
on the same spot pick up the one already there. At AIM's top speed of 200 mm/s no agent
sorts all six barrels in 45 s; the best runs score 4 or 5.

Simulated agents are also less variable than real ones (some score the same in every
run), so ranges here are likely narrower than they will be on real robots.

## Programs for the AIM WebSocket client (robo-bench)

`motionbench.game.aim_websocket` runs programs written for the VEX AIM **WebSocket Python
client** (the one that drives the robot from a laptop over Wi-Fi), unchanged: the client's own
code runs, and only its connection is replaced by a simulated robot that answers its commands and
sends it status packets. Time, `sleep`, `wait` and threads run on simulated time.

```python
from motionbench.game.aim_websocket import run_program
r = run_program(code, World(Game.load("games/rb-tag1"), seed=0), client_dir="path/to/robo-bench")
```

Four games reproduce robo-bench's fields (`games/rb-tag1`, `rb-tag2`, `rb-tag2-barrels`,
`rb-sort`), with AprilTag **signs** standing on the field: a sign falls when something hits it, and
the game records what did (a rolling ball, the robot, a carried ball). Knock-down games use the
`targets` goal (RFC-0003). The simulated robot was calibrated against a recorded real session;
replaying its four programs reproduces each real outcome. robo-bench's `sim/` runs its whole
generate → run → feedback loop on this.

## Next

- VEX IQ Level Up as a game (IQ drivetrain model, bean bags, pyramid tiers) once
  the team's code is ready.
- Calibrate driver characters from real students' filmed runs.
- Calibrate the robot model from real AIM runs (speed, slip, kick distances).
