# MotionBench specification v0.1

The normative text is [RFC-0001, Part II](../../rfcs/0001-motionbench.md#part-ii--specification-v01).
This folder holds machine-readable schemas that implement it:

| File | Validates |
| --- | --- |
| [task.schema.json](task.schema.json) | `tasks/<ID@VERSION>.json` |
| [manifest.schema.json](manifest.schema.json) | `manifest.json` inside a `.mbtrace` |

`poses.csv` columns: `t_s,x_m,y_m,theta_rad` (metres, radians, arena frame,
origin at the lower-left corner, +x right, +y away from the viewer, heading
counter-clockwise from +x). `events.csv` columns: `t_s,type,object_id,detail`.

## Optional task fields for field captures (v0.1, additive)

| Field | Meaning |
| --- | --- |
| `arena.unobserved` | Rectangles the camera could not see. Left out of coverage (they are neither covered nor missed) and ignored by the safety checks. |
| `robot.contact_tolerance_m` | How far the footprint may overlap a wall or obstacle before it counts as a violation. Default 0.03 m. Raise it for robots that touch walls by design (robot vacuums) or coarse captures. |
| `goal.edge_reach_m` (coverage) | Side-brush model: free cells next to a wall or obstacle also count as swept when they are within this distance of the robot's centre. |
| `layouts.*.objects[].zone` (delivery) | Per-object target zone, e.g. sorting by colour; `goal.zone` becomes the default. |

All are optional; tasks without them score exactly as before.
