# Real people, scored: THÖR-MAGNI walks

A first check of the scorer on real human motion. [THÖR-MAGNI](http://thor.oru.se/magni.html)
(Örebro University) recorded 40 people moving through a 20 m × 9 m room with motion
capture, walking between seven goal points around obstacles, sometimes carrying objects
or sharing the room with a teleoperated robot. Its motion-prediction challenge
repository publishes the processed walks, so no large download is needed.

```bash
git clone --depth 1 https://github.com/tmralmeida/lhmp-thor-magni-challenge-extras
python examples/thor-magni/thor_walks.py lhmp-thor-magni-challenge-extras/data out/ --from 3 --to 7
```

`thor_walks.py` chains the 8-second pieces back into continuous walks, cuts out every
goal-to-goal walk, builds an experimental task for one route (`MB-THOR-WALK@0.0`), writes
each walk as a tier-F trace, adds random-walk runs for the floor, and scores everything.

## What it found (goal 3 → goal 7, the most-walked route)

| | |
| --- | --- |
| Goal-to-goal walks in the dataset | 1,984 |
| Walks on this route | 120, by 60 people |
| Completed | 120 of 120 |
| Median time | 6.8 s (about 7 m, 1.0 m/s) |
| Slowest | 33.2 s: a pair of visitors detoured for 33 m |
| Random-walk success within 60 s | 40% |

Four lessons for the benchmark:

1. **Success alone cannot separate people on easy point-to-point jobs.** All 60 people
   score 1.0 and 60 of 61 agents share rank 1. Time, path efficiency and consistency do
   the separating, the same ceiling effect the synthetic VEX AIM board shows.
2. **The time limit should follow the people.** With 60 s, a random walk reaches the goal
   40% of the time, which raises the floor. A limit of about three times the median human
   time (about 20 s here) would push chance success towards zero.
3. **Path efficiency needs the arrival radius.** The oracle is measured centre to centre,
   but a walk ends on entering the 0.6 m radius, so many walks come out shorter than the
   oracle and are capped at 1.0. The oracle should subtract the tolerance.
4. **Shared ranks need a minimum number of trials.** One person with two walks, one
   failed, had an interval wide enough to overlap rank 1.

## Caveats

- This is an experimental task, not part of the benchmark's task set. Heading is not
  scored and the room's obstacles are not modelled, so it cannot count bumps.
- Several recordings per day reuse helmet numbers and restart their clocks. Pieces are
  chained only when one starts 0.4 s after another ends and within 1 m of it; grouping by
  day and helmet alone mixes different people and produces jumps of up to 14 m.
- The data is 2.5 Hz, coarser than MotionBench captures.

## Credit and licences

THÖR-MAGNI is released under CC BY 4.0; the challenge files are MIT-licensed. Please cite
the THÖR-MAGNI paper by T. Schreiter et al.
([arXiv:2403.09285](https://arxiv.org/abs/2403.09285)) and the
[dataset record](https://zenodo.org/records/13865754). No THÖR-MAGNI data is stored in
this repository.
