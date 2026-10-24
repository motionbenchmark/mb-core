# Multi-robot matches

A VEX match is four robots on one field for two minutes: two alliances, a
15-second autonomous period, then driver control. Scoring one robot in a match
means first knowing where every robot was, from ordinary event footage, with
no tags and with people, blocks and structures in the way.

`motionbench match` is the toolkit for that problem:

| command | what it does |
|---|---|
| `motionbench match synth -o DIR --seeds 1-5` | simulate 2 v 2 matches with full ground truth |
| `motionbench match render MATCH -o clip.mp4` | broadcast-style video of a match, plus per-frame ground truth |
| `motionbench match report MATCH [--png paths.png]` | per-robot metrics, split by period |
| `motionbench match track VIDEO --calib calib.json -o tracks.csv` | baseline tracker for fixed-camera footage (no tags) |
| `motionbench match from-tracks tracks.csv -o MATCH` | stitch tracks into robots and write a match folder for `report` |
| `motionbench match eval MATCH tracks.csv` | score a tracker against ground truth |

Everything here is **synthetic** unless it says otherwise. Simulated matches
are for building and testing trackers; they are not measurements of real
teams.

## What real footage looks like

The camera model comes from one real event broadcast (a screen recording of a
match stream). Only these numbers were kept: the footage is not in the
repository, because it shows students and belongs to the event.

- The broadcast opens with ~9 s of intro shots, then cuts to a **fixed overhead
  camera**. Over a minute the image moves by at most ~1 px, so the per-pixel
  median of the frames gives a clean empty field.
- **Camera:** fitted from the four inside corners of the 3.566 m field alone:
  focal length ≈ 1139 px at 1020 × 696, about 3.7 m above the floor, tilted
  about 44° down. Structures land within a few centimetres of where the field
  drawings put them, so one view of the field is enough to measure it.
- **Robots:** four, about 46 cm square and up to ~45 cm tall, so they lean
  noticeably in perspective: a robot's top can sit 40–60 px above its footprint.
  Alliances are told apart by small red or blue plates, and one robot may be
  painted red itself.
- **Clutter:** red and blue game blocks (the same colours as the plates), raised
  bars robots park beside and under, loaders in the corners, people standing
  round the field with hands reaching in, and a broadcast overlay (title, logo,
  score bars) on the edges.

## Simulated matches

`motionbench match synth` produces, per match:

```
m0001/
  match.json          field, periods, robots (alliance, size, look), camera, seed, "synthetic": true
  objects.csv         t_s,id,state,x_m,y_m,z_m,ref     a row whenever a block changes
  events.csv          t_s,robot,type,other,detail      pickups, deliveries, contacts, parking
  robots/red1/        one .mbtrace per robot (20 Hz), scorable with `motionbench score`
  tasks/MB-MATCH-FIELD@0.1.json
```

Field coordinates are metres on the floor inside the perimeter: origin at the
near-left corner as the broadcast camera sees it, +x right, +y away from the
camera. Red plays from the left.

The simulator is simple on purpose, but it creates what trackers find hard:
alliance partners that look alike, pushing and shoving (robots in contact), long
stops beside structures, blocks picked up, carried and pushed, a pause between
periods with nothing moving, and robots parked side by side at the end. Robots
plan around structures, give way unless they are defending, keep to their own
half in autonomous, and one robot per alliance parks at the end. Each robot has
its own speed, acceleration, turning, capacity and habits, drawn from the seed.

The field is game-neutral. Its structures resemble a recent season's field
closely enough to be realistic for tracking, but there is no rules model and no
game score.

## Rendering

`motionbench match render` draws a match from the broadcast camera: tiles and
mat, perimeter walls, raised bars, loaders, blocks, robots as stacks of boxes
with alliance plates, people round the field (legs, bodies, arms reaching in
to the loaders), the overlay, lighting variation, noise and JPEG artefacts. An
optional `--intro` adds title-card frames before the overhead shot.

Next to the video, `clip.gt.csv` gives, for every frame and robot, the floor
position, heading, the image point of the footprint centre and the image box.

A different camera is a `match.json` edit away (`camera`: frame size, focal
length, height, the four floor corners in pixels and the field size).

## Scoring a real match

1. Film from a fixed camera that sees all four field corners for the whole match.
2. `motionbench calibrate match.mp4 --vex-field --width 1020 --start 9 -o calib.html`: open the
   page, tap the field's four inside corners (far left, far right, near right, near left) and
   download `calib.json`. `--start` skips a broadcast intro; use `--field W L` for other fields.
3. `motionbench match track match.mp4 --calib calib.json --start 9 --match-start 12 -o tracks.csv --overlay check.mp4`
   (`--match-start` is the video time when the match clock starts). Watch `check.mp4`.
4. `motionbench match from-tracks tracks.csv -o my-match/ --name red1=1234A`: tracks that broke
   briefly are stitched back together and named by alliance.
5. `motionbench match report my-match/ --png paths.png`.

Ask the teams and the event before filming or using a recording: matches show students. The
project publishes robot paths, never the footage. The same guide is on the website at
https://motionbench.org/guides/matches.

## Report

`motionbench match report` works on any match folder, simulated or tracked,
because it uses only the poses: distance, mean and top speed, idle time, share of
the field visited, time in each half, on the centre mat and in the park zone,
robot-robot and wall contacts (from distances), each split into autonomous and
driver periods. Simulated matches also have ground-truth pickup and delivery
counts. `--png` draws everyone's path.

## Tracking benchmark

Tracker output is a CSV, one row per track per frame:

```
t_s,track_id,x_m,y_m,alliance
```

in match time and field metres. `motionbench match eval` matches tracks to
robots in each frame (one-to-one, within 0.45 m) and reports:

- **recall** and **precision**: robot-frames found, and track-frames that were a robot
- **position error**: median, mean and 90th percentile
- **ID switches**: a robot's track changes from one frame to the next
- **IDF1**: how much of the match each robot keeps one identity
- **MOTA**: 1 − (misses + false tracks + ID switches) / robot-frames
- **alliance accuracy**, when tracks say their colour

broken down per robot and per period. The pause between periods is left out.

### Baseline tracker

`motionbench match track` is a reference point, not a finished tracker:

1. empty field = per-pixel median of frames across the clip;
2. foreground = pixels that differ from it;
3. for each floor position on a 5 cm grid, the camera says which image box a
   robot standing there would fill; a robot is where that box is full of
   foreground and the ring round it is not (an occupancy map, so leaning robots
   and robots in contact are handled the same way);
4. peaks become detections; tracks follow them with a constant-velocity guess
   and one-to-one assignment, coasting through short occlusions;
5. alliance = the majority of saturated red or blue pixels low on the robot,
   where the plates are, which also steers assignment once it is known.

Baseline on three simulated matches (`data/match-sim-v0.1` m0001–m0003, rendered at 10 fps with
`--seed` equal to the match number; full numbers in `results/match-baseline-v0.1.json`):

| match | recall | precision | median error | 90% error | ID switches | IDF1 | MOTA | alliance |
|---|---|---|---|---|---|---|---|---|
| m0001 | 97.1% | 98.6% | 8.0 cm | 17.7 cm | 25 | 65.1% | 95.2% | 68.4% |
| m0002 | 89.5% | 95.8% | 8.9 cm | 19.1 cm | 33 | 65.6% | 84.9% | 90.6% |
| m0003 | 90.4% | 94.3% | 7.0 cm | 15.8 cm | 43 | 48.3% | 84.0% | 61.5% |

It finds the robots and puts them in the right place to within a robot's bumper, but keeping
each robot's identity for the whole match is where it falls down: that is the main target.

On the real broadcast clip (a minute of the overhead shot; no ground truth yet) the baseline
keeps a track on all four robots for the whole minute. One robot changes identity once, when
a pause symbol from the screen recording appears over the field and the tracker follows it;
four of its five tracks get the right alliance colour.

Known weaknesses, which are also the next things to build: robots that stand
still for much of the clip leak into the empty-field picture; two robots pushing
each other can merge into one detection and swap identities; a red-painted robot
can read as red alliance; people leaning far in can make short false tracks.

## Next steps

- Hand-labelled frames from real event footage (with permission), so real
  accuracy can be measured, not only simulated.
- A learned robot detector trained on rendered frames (the renderer gives
  unlimited labelled data), compared against the baseline on the same benchmark.
- Match tasks for the leaderboard built on tracked paths: autonomous routes,
  driver efficiency, contact and defence statistics.
