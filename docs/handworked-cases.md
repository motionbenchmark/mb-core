# Hand-worked scoring cases

Seventeen small cases whose expected results are worked out by hand from the task rules,
not by running the scorer. They run as `python/tests/test_handworked.py`, where each
case's arithmetic is written out in a comment so it can be checked with a pencil.

Two tasks on a 2 m × 2 m arena with a 10 s budget, and one coverage task:

- **HW-WAY:** start (0.2, 0.2, heading 0); W1 = (1.0, 0.2, heading 0), W2 = (1.0, 1.0, heading 90°);
  tolerance 5 cm and 5°.
- **HW-DEL:** start (0.2, 0.2); objects o1 = (0.5, 0.5) and o2 = (1.0, 0.5); goal zone
  x ∈ [1.5, 2.0], y ∈ [1.5, 2.0].
- **HW-COV:** a 1 m × 1 m floor in 0.1 m cells, a 30 cm tool, 30 s budget.

The robot's radius is 0.10 m with the default 3 cm contact tolerance, so it touches a wall
when its centre is within 0.07 m of it.

| # | Case | Rule | Expected |
|---|---|---|---|
| 1 | Both waypoints reached, straight lines | in order, within tolerance | 2/2; reference 1.6 m; efficiency 1.0 |
| 2 | Stops 4.9 cm from W1 | distance ≤ 5 cm | 1 |
| 3 | Stops 5.1 cm from W1 | distance ≤ 5 cm | 0 |
| 4 | At W1 with heading 0.2 rad | heading error ≤ 0.0873 rad | 0 |
| 5 | Visits W2, then W1 | waypoints count only in order | 1 |
| 6 | Reaches W2 at exactly 10.0 s | samples with t ≤ budget count | 2 |
| 7 | Same path ending at 10.5 s | samples after the budget are ignored, with a warning | 1 |
| 8 | No sample within 5 cm of W1 (gap in the record) | judged on recorded samples only; no interpolation | 0 |
| 9 | o1 at the zone centre, o2 not moved | object's final centre inside the zone | 1/2 |
| 10 | o2 exactly on the zone edge | the zone is a closed rectangle | 2/2 |
| 11 | o2 1 cm outside the edge | | 1/2 |
| 12 | Reference length for deliveries | greedy nearest-first, single carry, to zone centre | 2.19203 m after one delivery; efficiency 1.0 on that path, 1/3 on a path three times as long |
| 13 | Consistency | 1 − sample sd / mean, clamped; 0 when mean is 0 | 1.0 for 1,1,1,1,1; 0.75155 for 2,2,1,2,2; 0 for all zeros |
| 14 | Chance from a baseline agent | median of baseline means, shown as a goal share, not subtracted | 0.4 waypoints = 0.2 goal; an agent at 2/2 keeps goal 1.0 |
| 15 | Comparison with people | only when people's median − chance ≥ 10% of the maximum | (2 − 0.4)/(1 − 0.4) = 2.667 when people score 1; not reported when people score 0 |
| 16 | One straight coverage pass at y = 0.52 m from x = 0.25 to 0.75 m | a cell counts when its centre comes within half the tool width of the path | 8 + 8 + 6 = 22 of 100 cells: 22% |
| 17 | Wall contact: in, out, in twice in a row | contact episodes, not samples | 2 |

When these cases were first written, two of the hand derivations were wrong, and the
scorer was right both times. In case 12, the longer path ran past the 10 s budget, so the
delivery event was outside the budget. In case 14, chance is reported as a goal share,
not in task units. Both cases were corrected and their derivations rewritten. No case has
found a scorer error so far. The test file was later rewritten, with every derivation
redone, and cases 16 and 17 were added; all seventeen pass.
