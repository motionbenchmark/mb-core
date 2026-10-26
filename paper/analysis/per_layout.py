"""Per-layout results of the unseen-layout runs in the AIM sorting study.

Replays exactly the runs of `motionbench game verdict` on unseen layouts (same layouts and
seeds; the simulator is deterministic given both) and reports each agent's mean points per
layout. It adds no new runs. Writes paper/results/aim-sort-per-layout.json.
"""
import json
from pathlib import Path

import numpy as np

from motionbench.game.driver import Driver
from motionbench.game.run import simulate
from motionbench.game.spec import Game

ROOT = Path(__file__).resolve().parents[2]
G = Game.load(ROOT / "games/aim-sort")
V = json.loads((ROOT / "paper/results/aim-sort-verdict.json").read_text())
R = ROOT / "games/aim-sort/routines"
ROUT = ["deadreckon_v1.py", "vision_v2.py", "vision_tags_v3.py", "planner_v4.py"]
d = V["design"]
n, base = d["unseen_layouts"], d["unseen_seed_base"]
settings = [(f"gen-{base + s % n}", 6000 + s) for s in range(d["runs"])]
typ = Driver(**V["class"]["median_student"])

agents = {r: (lambda l, s, r=r: simulate(G, code=(R / r).read_text(), code_name=r, layout=l, seed=s)) for r in ROUT}
agents["typical simulated driver"] = lambda l, s: simulate(G, driver=typ, layout=l, seed=s)
out = {"note": "Simulated. Same runs as the unseen-layout verdict, grouped by layout.", "layouts": [f"gen-{base + i}" for i in range(n)],
       "mean_points": {}, "check_overall_mean": {}}
for name, fn in agents.items():
    pts = [fn(l, s).points for l, s in settings]
    out["mean_points"][name] = {f"gen-{base + i}": float(np.mean([p for p, (l, _) in zip(pts, settings) if l == f"gen-{base + i}"]))
                                for i in range(n)}
    out["check_overall_mean"][name] = float(np.mean(pts))
    print(name, out["check_overall_mean"][name], list(out["mean_points"][name].values()))
(ROOT / "paper/results/aim-sort-per-layout.json").write_text(json.dumps(out, indent=1))
