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
