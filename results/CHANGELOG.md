# Results changelog

Every change to a published board or result, newest first. Boards carry the spec and
scorer version they were produced with; results under an older spec stay valid as
results under that spec. See [docs/protocol.md](../docs/protocol.md).

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
