"""Where the points went: what happened to each barrel in the AIM sorting study.

Replays exactly the runs of the verdict (same layouts and seeds; the simulator is
deterministic given both, so no new runs) and classifies every barrel at the end of
each run from the final world state and the run's events:

  sorted          at rest in its own colour's zone (scores)
  wrong zone      at rest in the other colour's zone
  outside a zone  picked up at least once but at rest outside both zones
  carried         still held by the robot when time ran out
  never picked    never picked up

It also counts "re-grabs": pick-ups of a barrel that was at rest in its own zone (the
robot drove into a barrel it had already sorted). Writes paper/results/aim-sort-failures.json.
"""
import json
from collections import Counter
from pathlib import Path

from motionbench.game.driver import Driver
from motionbench.game.run import simulate
from motionbench.game.spec import Game

ROOT = Path(__file__).resolve().parents[2]
G = Game.load(ROOT / "games/aim-sort")
V = json.loads((ROOT / "paper/results/aim-sort-verdict.json").read_text())
R = ROOT / "games/aim-sort/routines"
ROUT = ["deadreckon_v1.py", "vision_v2.py", "vision_tags_v3.py", "planner_v4.py"]
d = V["design"]
SETTINGS = {
    "public": [(V["layout"], 1000 + s) for s in range(d["runs"])],
    "unseen": [(f"gen-{d['unseen_seed_base'] + s % d['unseen_layouts']}", 6000 + s) for s in range(d["runs"])],
}
typ = Driver(**V["class"]["median_student"])
CATS = ["sorted", "wrong zone", "outside a zone", "carried", "never picked"]


def classify(res):
    own = {o["id"]: G.best_zone(o["kind"]) for o in res.objects}
    picked = {e[2] for e in res.events if e[1] == "pickup"}
    c = Counter()
    for o in res.objects:
        zones = G.zone_of(o["x"], o["y"])
        if o.get("held"):
            c["carried"] += 1
        elif own[o["id"]] in zones:
            c["sorted"] += 1
        elif zones:
            c["wrong zone"] += 1
        elif o["id"] in picked:
            c["outside a zone"] += 1
        else:
            c["never picked"] += 1
    # re-grabs: a pick-up of a barrel whose last placement was in its own zone
    last_place = {}
    regrab = 0
    for t, kind, oid, detail in res.events:
        if kind == "place":
            last_place[oid] = own.get(oid) in detail.split(",")
        elif kind == "pickup" and last_place.get(oid):
            regrab += 1
            last_place[oid] = False
    return c, regrab


agents = {r: (lambda l, s, r=r: simulate(G, code=(R / r).read_text(), code_name=r, layout=l, seed=s)) for r in ROUT}
agents["typical simulated driver"] = lambda l, s: simulate(G, driver=typ, layout=l, seed=s)
out = {"note": "Simulated. Same runs as the verdict; barrels per run (6 each), summed over 20 runs.", "categories": CATS}
for setting, sets in SETTINGS.items():
    out[setting] = {}
    for name, fn in agents.items():
        tot, rg, pts = Counter(), 0, []
        for l, s in sets:
            res = fn(l, s)
            c, r_ = classify(res)
            tot.update(c)
            rg += r_
            pts.append(res.points)
            assert c["sorted"] == res.points, (name, l, s, c, res.points)
        out[setting][name] = {**{k: tot[k] for k in CATS}, "re-grabs": rg, "runs": len(sets)}
        print(setting, name, out[setting][name])
(ROOT / "paper/results/aim-sort-failures.json").write_text(json.dumps(out, indent=1))
