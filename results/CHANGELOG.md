# Results changelog

Every change to a published board or result, newest first. Boards carry the spec and
scorer version they were produced with; results under an older spec stay valid as
results under that spec. See [docs/protocol.md](../docs/protocol.md).

## 2026-10-26
- **`aim-sort-verdict-v0.1.json` regenerated with the current simulator.** The published file
  had been produced before the simulator changes of 2026-10-24 (AprilTag signs, camera
  model), so it did not reproduce from the repository. Dead reckoning, the planner and every
  driver are unchanged. Vision: unseen 2.40 -> 2.35, change from public -0.05 -> -0.10.
  Vision + tags: public 2.15 -> 2.25, unseen 1.85 -> 1.80.
- **Equivalence is strict.** A comparison is "equivalent" only when its 95% range lies
  strictly inside the margin. The vision routine's change from public to unseen is now
  "no difference found" (-0.10, range -0.55 to +0.40; before: "equivalent").
- Per-layout results added (`paper/results/aim-sort-per-layout.json`, the same runs grouped
  by layout).

## 2026-10-24 (later)
- **Game verdicts are paired** (same seeds for every agent), resampled over layouts then runs
  on unseen layouts, and use four outcomes with a 0.5-point equivalence margin
  (better / worse / equivalent / no difference found). `aim-sort-verdict-v0.1.json`
  regenerated; the drivers now run as many paired runs as the code (20).
- **Capture accuracy corrected.** Re-measured over three render seeds with medians, 90th
  percentiles and missing frames (`paper/results/capture-accuracy.json`): about 1 cm with
  tags, about 3 cm without (earlier text said about 2 cm and about 6 cm, from single runs).
- **People comparison guard:** `vs_people` is reported only when the median person beats
  chance by at least 10% of the task's maximum. No published board changes.

## 2026-10-24
- **Match tracking baseline** (`match-baseline-v0.1.json`, synthetic): three simulated
  matches, baseline tracker re-run after the alliance-vote change (one vote per frame).
- **Known issue, synthetic VEX AIM boards**: some simulated AIM agents drive faster than the
  AIM robot's published top speed (200 mm/s), so their times on MB-AIM-NAV and MB-AIM-SORT
  are optimistic. The game simulator (`motionbench game`) uses the real limit; the boards
  will be regenerated with it in the next data release.

## 2026-10-23
- **Spec 0.2** (RFC-0002): boards report the share of the goal (`goal`, `goal_ci95`,
  `goal_worst`) and chance level; comparisons with people only where
  `people_comparable` is set. Ranks unchanged. Synthetic boards regenerated.

## 2026-10-22
- **Field runs v0.1**: first real run (robot vacuum, home hallway, tag-free phone video).
- **VEX AIM tasks** MB-AIM-NAV@0.1 and MB-AIM-SORT@0.1 with synthetic data.
- **Initial release**: spec 0.1, synthetic dataset v0.1 (570 runs).
