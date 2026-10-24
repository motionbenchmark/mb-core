"""How much do rankings depend on the MotionBench Index weights?

On the synthetic boards whose agents respect their robots' limits (VEX V5 and room
coverage; the VEX AIM boards are excluded, see results/CHANGELOG.md), recompute the
index for random weight vectors and compare rankings with Kendall's tau:

* broad:  weights drawn uniformly from the simplex (Dirichlet(1,1,1,1));
* local:  weights drawn around the defaults (Dirichlet(50 * w_default)).

Also compares the default-index ranking with the ranking by goal score alone, the
primary outcome. Writes paper/results/index-sensitivity.json.
"""
import json
from pathlib import Path

import numpy as np
from scipy.stats import kendalltau

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "paper" / "results" / "index-sensitivity.json"
W0 = np.array([0.50, 0.20, 0.15, 0.15])
TASKS = ["MB-VEX-NAV@1.0", "MB-VEX-PICK@1.0", "MB-COVER@0.1"]


def terms(r):
    return np.array([r["goal"], min(max(r["path_efficiency"], 0.0), 1.0), r["consistency"], r["safety"]])


def main(n=5000, seed=0):
    board = json.loads((ROOT / "results" / "leaderboard-synthetic-v0.1.json").read_text())
    rng = np.random.default_rng(seed)
    out = {"weights_default": W0.tolist(), "samples": n, "tasks": {}}
    for b in board["tasks"]:
        if b["task"] not in TASKS:
            continue
        rows = [r for r in b["rows"] if r["class"] != "baseline"]
        X = np.array([terms(r) for r in rows])
        base = X @ W0
        assert np.allclose(base, [r["mbi"] for r in rows], atol=2e-3), "index recomputation differs from the board"
        res = {}
        for label, draw in (("broad", lambda: rng.dirichlet(np.ones(4))), ("local", lambda: rng.dirichlet(50 * W0))):
            taus, top_same = [], 0
            for _ in range(n):
                s = X @ draw()
                taus.append(kendalltau(base, s).statistic)
                top_same += int(np.argmax(s) == np.argmax(base))
            res[label] = {"tau_median": round(float(np.median(taus)), 3), "tau_p5": round(float(np.percentile(taus, 5)), 3),
                          "top_agent_unchanged": round(top_same / n, 3)}
        res["goal_only"] = {"tau": round(float(kendalltau(base, X[:, 0]).statistic), 3)}
        res["agents"] = len(rows)
        out["tasks"][b["task"]] = res
        print(b["task"], res)
    OUT.write_text(json.dumps(out, indent=1))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
