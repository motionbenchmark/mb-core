"""Figure: the robot-vacuum capture as the scorer sees it (top-down path and swept floor).

    python paper/make_vacuum_fig.py
"""
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.image import imread
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
T = json.loads((ROOT / "data/field-v0.1/tasks/MB-FIELD-HALLWAY@0.1.json").read_text())
TR = ROOT / "data/field-v0.1/MB-FIELD-HALLWAY@0.1/field-01/robot-vacuum-hallway/trial-0"
P = np.array([[float(v) for v in r] for r in list(csv.reader(open(TR / "poses.csv")))[1:]])
W, H = T["arena"]["width_m"], T["arena"]["height_m"]
cell, half = T["goal"].get("cell_m", 0.05), T["goal"]["tool_width_m"] / 2
# The scorer's coverage rule (crates/motionbench-core/src/score.rs, fn coverage), redrawn here:
# cells of `cell` metres; a free cell is swept when its centre comes within half the tool width
# of the path (or within edge_reach_m for cells next to a wall), with the path stamped every
# half cell. The share must match the scorer's report.
import math
import motionbench as mb
re_ = max(T["goal"].get("edge_reach_m") or 0.0, half)
nx, ny = math.ceil(W / cell), math.ceil(H / cell)
cy, cx = (np.mgrid[0:ny, 0:nx] + 0.5) * cell
inside = lambda o: (cx >= o["x"]) & (cx <= o["x"] + o["w"]) & (cy >= o["y"]) & (cy <= o["y"] + o["h"])
wall = (cx > W) | (cy > H)
for o in T["arena"]["obstacles"]:
    wall |= inside(o)
unseen = np.zeros_like(wall)
for o in T["arena"].get("unobserved", []):
    unseen |= inside(o)
blocked = wall | unseen
free = ~blocked
pad = np.pad(wall, 1, constant_values=True)
edge = free & np.any([pad[1 + j:ny + 1 + j, 1 + i:nx + 1 + i] for j in (-1, 0, 1) for i in (-1, 0, 1)], axis=0)
pts = [P[0, 1:3]]
for p0, p1 in zip(P[:-1, 1:3], P[1:, 1:3]):
    n = max(1, math.ceil(np.hypot(*(p1 - p0)) / (cell / 2)))
    pts += [p0 + (p1 - p0) * k / n for k in range(1, n + 1)]
swept = np.zeros_like(free)
for x, y in pts:
    d = np.hypot(cx - x, cy - y)
    swept |= free & ((d <= half) | (edge & (d <= re_)))
pct = 100 * (swept & free).sum() / free.sum()
ref = mb.score(TR, ROOT / "data/field-v0.1/tasks")["success"]["score"]
assert abs(pct - ref) < 0.05, (pct, ref)
print(f"coverage redrawn {pct:.2f}%, scorer {ref:.2f}%")

fig, (a, b) = plt.subplots(1, 2, figsize=(6.6, 4.4), dpi=200, gridspec_kw={"width_ratios": [1.05, 1]})
a.imshow(imread(ROOT / "paper/figs/hallway-tracked.jpg"))
a.set_axis_off()
a.set_title("phone video with the tracked path", fontsize=8)
img = np.zeros(blocked.shape + (3,))
img[free] = (0.90, 0.90, 0.90)
img[swept & free] = (0.55, 0.78, 0.95)
img[blocked] = (0.45, 0.45, 0.45)
b.imshow(img, origin="lower", extent=(0, W, 0, H))
b.plot(P[:, 1], P[:, 2], color="#C0262D", lw=0.8)
b.plot(*P[0, 1:3], "o", color="#C0262D", ms=3)
b.set_xlabel("m", fontsize=7)
b.set_ylabel("m", fontsize=7)
b.tick_params(labelsize=6)
b.set_title(f"top-down: swept (blue, {pct:.0f}% of visible floor),\nnot swept (light grey), not visible (dark grey)", fontsize=8)
fig.tight_layout()
fig.savefig(ROOT / "paper/figs/vacuum-capture.pdf")
print(f"wrote paper/figs/vacuum-capture.pdf (swept {pct:.1f}% in this drawing)")
