"""Do rankings depend on the time budget, and on the layouts? (existing runs only)

Replays the runs of the AIM sorting verdict (same layouts and seeds, so no new runs) and
scores each run as if the match had ended earlier, at T = 25, 30, 35, 40 and 45 s, with the
game's own rule applied to the last recorded instant at or before T: a barrel counts if its
centre is in its own colour's zone and it is neither held by nor touching the robot. (Longer budgets would need new runs and are not studied.) At 45 s
this cut-off rule is checked against the game's own score. It then compares the ranking of
the 11 agents (4 routines, 6 driver characters, typical simulated driver) across budgets
and between public and unseen layouts with Kendall's tau.
Writes paper/results/aim-sort-budget.json.
"""
import json
from pathlib import Path

import numpy as np
from scipy.stats import kendalltau

from motionbench.game.driver import CHARACTERS, Driver
from motionbench.game.run import simulate
from motionbench.game.spec import Game
from motionbench.game.world import ROBOT_R


ROOT = Path(__file__).resolve().parents[2]
G = Game.load(ROOT / "games/aim-sort")
V = json.loads((ROOT / "paper/results/aim-sort-verdict.json").read_text())
R = ROOT / "games/aim-sort/routines"
d = V["design"]
SETTINGS = {
    "public": [(V["layout"], 1000 + s) for s in range(d["runs"])],
    "unseen": [(f"gen-{d['unseen_seed_base'] + s % d['unseen_layouts']}", 6000 + s) for s in range(d["runs"])],
}
RADIUS = float(G.raw.get("robot", {}).get("radius_m", ROBOT_R))  # as World does
BUDGETS = [25, 30, 35, 40, 45]
agents = {r: (lambda l, s, r=r: simulate(G, code=(R / r).read_text(), code_name=r, layout=l, seed=s))
          for r in ["deadreckon_v1.py", "vision_v2.py", "vision_tags_v3.py", "planner_v4.py"]}
for c in CHARACTERS:
    agents[c.name] = lambda l, s, c=c: simulate(G, driver=c, layout=l, seed=s)
typ = Driver(**V["class"]["median_student"])
agents["typical simulated driver"] = lambda l, s: simulate(G, driver=typ, layout=l, seed=s)


def score_at(res, T):
    ts = np.array([p[0] for p in res.poses])
    k = int(np.searchsorted(ts, T + 1e-9) - 1)
    _, rx, ry, _ = res.poses[k]
    pts = 0
    for (x, y, held, _), kind in zip(res.object_track[k], [o["kind"] for o in res.objects]):
        touching = held or np.hypot(x - rx, y - ry) <= RADIUS + G.object_kind(kind)["radius_m"] + 0.003
        if not touching and G.best_zone(kind) in G.zone_of(x, y):
            pts += 1
    return pts


out = {"note": "Simulated; existing runs re-scored at earlier cut-offs.", "budgets_s": BUDGETS, "means": {}, "check_45s": {}}
for setting, sets in SETTINGS.items():
    means = {T: {} for T in BUDGETS}
    agree = total = 0
    for name, fn in agents.items():
        rows = []
        for l, s in sets:
            res = fn(l, s)
            rows.append([score_at(res, T) for T in BUDGETS])
            agree += int(rows[-1][-1] == res.points)
            total += 1
        for i, T in enumerate(BUDGETS):
            means[T][name] = float(np.mean([r[i] for r in rows]))
    out["means"][setting] = {str(T): means[T] for T in BUDGETS}
    out["check_45s"][setting] = {"runs": total, "agree_with_game_score": agree}
    names = list(agents)
    ref = [means[45][n] for n in names]
    out.setdefault("tau_vs_45s", {})[setting] = {str(T): round(float(kendalltau([means[T][n] for n in names], ref).statistic), 3)
                                                for T in BUDGETS[:-1]}
    out.setdefault("leader", {})[setting] = {str(T): max(names, key=lambda n: means[T][n]) for T in BUDGETS}
    rout = names[:4]
    out.setdefault("best_routine", {})[setting] = {str(T): max(rout, key=lambda n: means[T][n]) for T in BUDGETS}
names = list(agents)
pub = [out["means"]["public"]["45"][n] for n in names]
uns = [out["means"]["unseen"]["45"][n] for n in names]
out["tau_public_vs_unseen_45s"] = round(float(kendalltau(pub, uns).statistic), 3)
(ROOT / "paper/results/aim-sort-budget.json").write_text(json.dumps(out, indent=1))
print(json.dumps({k: v for k, v in out.items() if k != "means"}, indent=1))
