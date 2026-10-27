"""Figure 1: the same routine on the public layout and on an unseen one (simulated).

Replays two runs of the verdict (no new runs): the planner on the public layout (seed
1000) and on the first unseen layout (gen-5000, seed 6000).

    python paper/make_teaser.py
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

from motionbench.game.run import simulate
from motionbench.game.spec import Game

ROOT = Path(__file__).resolve().parents[1]
G = Game.load(ROOT / "games/aim-sort")
code = (ROOT / "games/aim-sort/routines/planner_v4.py").read_text()
F = G.field
COL = {"orange": "#E07B00", "blue": "#1F5FBF"}

fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.6), dpi=200)
for ax, (layout, seed, title) in zip(axes, [("dev-01", 1000, "public layout"), ("gen-5000", 6000, "unseen layout")]):
    res = simulate(G, code=code, code_name="planner_v4.py", layout=layout, seed=seed)
    _, lay = G.layout(layout)
    ax.add_patch(Rectangle((0, 0), F["width_m"], F["height_m"], fc="#F4F4F2", ec="#555", lw=0.8))
    for name, z in F["zones"].items():
        ax.add_patch(Rectangle((z["x"], z["y"]), z["w"], z["h"], fc=COL[name], alpha=0.18, ec=COL[name], lw=0.8))
    P = res.poses
    ax.plot([p[1] for p in P], [p[2] for p in P], color="#333", lw=0.7, alpha=0.8)
    ax.plot(P[0][1], P[0][2], marker="^", color="#333", ms=4)
    kind = {o["id"]: o["kind"] for o in lay["objects"]}
    for o in lay["objects"]:  # where the barrels started
        c = COL[G.raw["objects"][o["kind"]]["colour"]]
        ax.add_patch(Circle((o["x"], o["y"]), 0.03, fc="none", ec=c, lw=1.0, ls="--"))
    for o in res.objects:  # where they ended up
        c = COL[G.raw["objects"][kind[o["id"]]]["colour"]]
        ax.add_patch(Circle((o["x"], o["y"]), 0.03, fc=c, ec="white", lw=0.4))
    ax.set_xlim(-0.03, F["width_m"] + 0.03); ax.set_ylim(-0.03, F["height_m"] + 0.03)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(f"{title}: {res.points:.0f} of {G.max_points:.0f} sorted", fontsize=8)
fig.text(0.5, 0.02, "dashed: where each barrel started  ·  filled: where it ended  ·  line: robot path  (simulated)",
         ha="center", fontsize=6.5, color="#444")
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig(ROOT / "paper/figs/teaser.pdf")
print("wrote paper/figs/teaser.pdf")
