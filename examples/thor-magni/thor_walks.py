"""Score real people walking between goal points, from the THÖR-MAGNI dataset.

THÖR-MAGNI (Örebro University) records people moving through a 20 m x 9 m room with
motion capture. Its motion-prediction challenge repository ships the processed walks
as 8-second pieces at 2.5 Hz, plus the positions of seven goal points:

    git clone --depth 1 https://github.com/tmralmeida/lhmp-thor-magni-challenge-extras
    python examples/thor-magni/thor_walks.py lhmp-thor-magni-challenge-extras/data out/ --from 3 --to 7

The script
  1. chains the 8-second pieces back into continuous walks,
  2. cuts out every walk from one goal point to another,
  3. writes an experimental task (MB-THOR-WALK@0.0) for the chosen route,
  4. converts each walk into a MotionBench trace (tier F, capture "mocap"),
  5. adds random-walk runs for the floor, and scores and aggregates everything.

Outputs in OUT: tasks/, traces/, reports.json, board.json, walks.csv.
Replay them (the task is custom, so use the Python call, which takes a tasks folder):

    python -c "import motionbench.viz as v; open('walks.html','w').write(v.build('OUT/traces', tasks_dir='OUT/tasks'))"

Data: THÖR-MAGNI is CC BY 4.0; the challenge files are MIT-licensed. Cite the
THÖR-MAGNI paper by Schreiter et al. (arXiv:2403.09285) and https://zenodo.org/records/13865754.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

import motionbench as mb

STEP = 0.4  # seconds between samples in the challenge files
OX, OY, W, H = 10.0, 4.5, 20.0, 9.0  # shift room coordinates so the arena starts at (0, 0)


def chains(df: pd.DataFrame):
    """Yield continuous walks. Pieces are chained only when one starts one step after
    another ends and within 1 m of where it ended. Several recordings per day reuse
    helmet numbers and restart their clocks, so day + helmet alone is not one person."""
    for (m, ag), d in df.groupby(["map_name", "ag_id"]):
        pieces = {k: g.sort_values("Time") for k, g in d.groupby("tracklet_id")}
        ends = {k: (g.Time.iloc[0], g.Time.iloc[-1], g[["x", "y"]].iloc[0].to_numpy(), g[["x", "y"]].iloc[-1].to_numpy())
                for k, g in pieces.items()}
        succ, has_pred = {}, set()
        for a, (_, ea, _, la) in ends.items():
            best = None
            for b, (sb, _, fb, _) in ends.items():
                if b != a and b not in has_pred and abs(sb - ea - STEP) < 0.1:
                    gap = np.linalg.norm(fb - la)
                    if gap < 1.0 and (best is None or gap < best[0]):
                        best = (gap, b)
            if best:
                succ[a] = best[1]
                has_pred.add(best[1])
        for a in pieces:
            if a in has_pred:
                continue
            ids = [a]
            while ids[-1] in succ:
                ids.append(succ[ids[-1]])
            yield m, ag, pd.concat([pieces[k] for k in ids])


def goal_walks(data: Path, radius: float):
    """Every walk that leaves one goal point and next arrives at a different one."""
    goals = pd.read_csv(data / "goals_positions.csv")
    goals[["x", "y"]] /= 1000.0  # millimetres to metres
    for sc in range(1, 6):
        df = pd.read_csv(data / f"Scenario_{sc}.csv")
        for m, ag, ch in chains(df):
            day = int(m[:4])
            G = goals[goals.day == day]
            t, p = ch.Time.to_numpy(), ch[["x", "y"]].to_numpy()
            dist = np.linalg.norm(p[:, None, :] - G[["x", "y"]].to_numpy()[None], axis=2)
            near = np.where(dist.min(1) < radius, G.goal.to_numpy()[dist.argmin(1)], 0)
            last, last_i = 0, 0
            for i, g in enumerate(near):
                if g and last and g != last:
                    yield dict(scenario=sc, day=day, agent=ag, label=ch.data_label.iloc[0],
                               frm=int(last), to=int(g), t0=float(t[last_i]), t1=float(t[i]),
                               poses=[(float(a), float(b), float(c)) for a, (b, c) in zip(t[last_i:i + 1], p[last_i:i + 1])])
                if g:
                    last, last_i = g, i


def with_heading(rows):
    """Re-base time to 0, shift into the arena frame, and add heading from the direction of travel."""
    t0 = rows[0][0]
    out, th = [], 0.0
    for i, (t, x, y) in enumerate(rows):
        a, b = rows[max(i - 1, 0)], rows[min(i + 1, len(rows) - 1)]
        if math.hypot(b[1] - a[1], b[2] - a[2]) > 0.02:
            th = math.atan2(b[2] - a[2], b[1] - a[1])
        out.append((t - t0, x + OX, y + OY, th))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("data", type=Path, help="the challenge repository's data/ folder")
    ap.add_argument("out", type=Path)
    ap.add_argument("--from", dest="frm", type=int, default=3)
    ap.add_argument("--to", type=int, default=7)
    ap.add_argument("--radius", type=float, default=0.6, help="arrival radius around a goal point (m)")
    ap.add_argument("--budget", type=float, default=60.0, help="time limit per walk (s)")
    ap.add_argument("--random-trials", type=int, default=5, help="random-walk runs per day, for the floor")
    a = ap.parse_args()

    walks = list(goal_walks(a.data, a.radius))
    route = [w for w in walks if w["frm"] == a.frm and w["to"] == a.to]
    if not route:
        raise SystemExit(f"no walks from goal {a.frm} to goal {a.to}")
    a.out.mkdir(parents=True, exist_ok=True)
    pd.DataFrame([{k: v for k, v in w.items() if k != "poses"} for w in walks]).to_csv(a.out / "walks.csv", index=False)

    goals = pd.read_csv(a.data / "goals_positions.csv")
    goals[["x", "y"]] /= 1000.0

    def gxy(day, g):
        r = goals[(goals.day == day) & (goals.goal == g)].iloc[0]
        return round(r.x + OX, 3), round(r.y + OY, 3)

    layouts = {}
    for day in sorted({w["day"] for w in route}):
        (sx, sy), (ex, ey) = gxy(day, a.frm), gxy(day, a.to)
        layouts[f"day-{day}"] = {"start": {"x": sx, "y": sy, "theta": 0.0},
                                 "waypoints": [{"x": ex, "y": ey, "theta": 0.0}]}
    task_key = "MB-THOR-WALK@0.0"
    task = {
        "id": "MB-THOR-WALK", "version": "0.0",
        "title": f"Walk from goal {a.frm} to goal {a.to} in the THÖR-MAGNI room (experimental)",
        "domain": "human-motion",
        "description": "Experimental task built from THÖR-MAGNI (CC BY 4.0). A person walks between two goal "
                       "points in a 20 m x 9 m motion-capture room. Heading is not scored and obstacles are "
                       "not modelled. Not part of the benchmark's task set.",
        "arena": {"width_m": W, "height_m": H, "obstacles": [], "zones": {}},
        "robot": {"radius_m": 0.25},
        "budget": {"time_s": a.budget},
        "goal": {"kind": "waypoints", "pos_tol_m": a.radius, "heading_tol_rad": math.pi},
        "reference": {"floor": 0},
        "layouts": layouts,
    }
    tasks = a.out / "tasks"
    tasks.mkdir(exist_ok=True)
    (tasks / f"{task_key}.json").write_text(json.dumps(task, indent=2))

    traces = a.out / "traces"
    shutil.rmtree(traces, ignore_errors=True)
    for i, w in enumerate(route):
        person = f"d{w['day']}-sc{w['scenario']}-{w['agent'].replace('Helmet_', 'h')}"
        mb.write_trace(traces / f"walk-{i:03d}", task=task_key, seed=f"day-{w['day']}", agent_id=person,
                       agent_class="human", agent_name=f"{w['label']} ({person})",
                       agent_meta={"scenario": w["scenario"], "source": "THOR-MAGNI"},
                       poses=with_heading(w["poses"]), trial=i, tier="F", capture="mocap",
                       notes="Converted from THOR-MAGNI challenge files (2.5 Hz).")

    rng = random.Random(2026)
    for seed, lay in layouts.items():
        for k in range(a.random_trials):
            x, y, th = lay["start"]["x"], lay["start"]["y"], rng.uniform(-math.pi, math.pi)
            rows = []
            for n in range(int(a.budget / STEP) + 1):
                rows.append((n * STEP, x, y, math.atan2(math.sin(th), math.cos(th))))
                th += rng.gauss(0, 0.6)
                nx, ny = x + 1.2 * STEP * math.cos(th), y + 1.2 * STEP * math.sin(th)
                if 0.3 < nx < W - 0.3 and 0.3 < ny < H - 0.3:
                    x, y = nx, ny
                else:
                    th += math.pi
            mb.write_trace(traces / f"random-{seed}-{k}", task=task_key, seed=seed, agent_id="random-walk",
                           agent_class="baseline", agent_name="Random walk (floor)", poses=rows, trial=k)

    reports, errors = mb.score_dir(traces, tasks_dir=tasks)
    board = mb.aggregate(reports, tasks_dir=tasks)
    (a.out / "reports.json").write_text(json.dumps(reports))
    (a.out / "board.json").write_text(json.dumps(board, indent=1))

    humans = [r for r in reports if r["agent_class"] == "human"]
    times = [r["efficiency"]["time_to_goal_s"] for r in humans if r["success"]["completed"]]
    effs = [r["efficiency"]["path_efficiency"] for r in humans]
    t = board["tasks"][0]
    print(f"{len(walks)} goal-to-goal walks found; {len(route)} on goal {a.frm} -> {a.to}; {len(errors)} scoring errors")
    print(f"completed {len(times)}/{len(humans)}; median time {np.median(times):.1f} s; "
          f"median path efficiency {np.median(effs):.2f}; floor {t['floor']:.2f}")
    print(f"{sum(r['rank'] == 1 for r in t['rows'])} of {len(t['rows'])} agents share rank 1")


if __name__ == "__main__":
    main()
