"""Score a multi-robot tracker against a match's ground truth.

Tracker output is a CSV with one row per track per frame:

    t_s,track_id,x_m,y_m[,alliance]

in match time and field metres (origin at the near-left corner, +y away from
the camera). Each frame, tracks are matched one-to-one to the robots within a
gate of ``gate_m`` (Hungarian assignment on distance). From that:

  recall / precision    share of robot-frames found / share of track-frames that are a robot
  position error        median, mean and 90th percentile, for matched pairs (metres)
  id switches           a robot's matched track changes from one frame to the next
  IDF1                  the best fixed track-to-robot assignment over the whole match:
                        how much of the time each robot keeps one identity
  MOTA                  1 - (misses + false tracks + id switches) / robot-frames
  alliance accuracy     when tracks say their alliance

The same numbers come per robot and per period (auton, driver).
"""

from __future__ import annotations

import csv
import itertools
import math
from collections import Counter, defaultdict
from typing import Optional

import numpy as np

from .io import MatchData


def _assign(C: np.ndarray) -> list[tuple[int, int]]:
    if C.size == 0:
        return []
    try:
        from scipy.optimize import linear_sum_assignment
        r, c = linear_sum_assignment(C)
        return list(zip(r.tolist(), c.tolist()))
    except ImportError:  # small problems: try every assignment
        n, k = C.shape
        best, pairs = math.inf, []
        if n <= k:
            for perm in itertools.permutations(range(k), n):
                s = sum(C[i, j] for i, j in enumerate(perm))
                if s < best:
                    best, pairs = s, list(enumerate(perm))
        else:
            for perm in itertools.permutations(range(n), k):
                s = sum(C[i, j] for j, i in enumerate(perm))
                if s < best:
                    best, pairs = s, [(i, j) for j, i in enumerate(perm)]
        return pairs


def load_tracks(path: str, offset_s: float = 0.0) -> dict[float, list[dict]]:
    frames: dict[float, list[dict]] = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            t = round(float(row["t_s"]) + offset_s, 3)
            frames[t].append({"id": str(row["track_id"]), "x": float(row["x_m"]), "y": float(row["y_m"]),
                              "alliance": (row.get("alliance") or "").strip().lower() or None})
    return dict(frames)


def evaluate(match_dir, tracks_csv: str, *, gate_m: float = 0.45, offset_s: float = 0.0,
             t_range: Optional[tuple[float, float]] = None, skip_pause: bool = True) -> dict:
    m = MatchData(match_dir)
    ids = [r["id"] for r in m.robots]
    ally = {r["id"]: r["alliance"] for r in m.robots}
    periods = [(p["name"], p["start_s"], p["end_s"]) for p in m.meta.get("periods", [])] or [("match", 0, m.duration)]
    frames = load_tracks(tracks_csv, offset_s)
    times = sorted(t for t in frames if 0 <= t <= m.duration and (t_range is None or t_range[0] <= t <= t_range[1]))
    # frames with no tracks at all still count as misses: evaluate on a regular grid too
    if times:
        dt = float(np.median(np.diff(times))) if len(times) > 1 else 0.1
        lo, hi = (t_range or (times[0], times[-1]))
        grid = np.round(np.arange(lo, hi + 1e-9, dt), 3)
        times = sorted(set(times) | {t for t in grid.tolist() if not any(abs(t - u) < dt / 2 for u in frames)})

    def period(t):
        return next((n for n, a, b in periods if a <= t < b), "end")

    errs: dict[str, list] = defaultdict(list)
    per = defaultdict(lambda: Counter())
    last: dict[str, Optional[str]] = {i: None for i in ids}
    overlap: Counter = Counter()  # (track, robot) matched frames
    track_frames: Counter = Counter()
    all_err: list[float] = []
    alliance_ok = alliance_n = 0
    for t in times:
        pname = period(t)
        if skip_pause and pname == "pause":
            continue
        gt = [(i, *m.pose(i, t)[:2]) for i in ids]
        tr = frames.get(t, [])
        for d in tr:
            track_frames[d["id"]] += 1
        C = np.array([[math.hypot(g[1] - d["x"], g[2] - d["y"]) for d in tr] for g in gt]) if tr else np.zeros((len(gt), 0))
        pairs = [(i, j) for i, j in _assign(np.where(C > gate_m, 1e6, C)) if C[i, j] <= gate_m]
        matched_g = {i for i, _ in pairs}
        for scope in ("all", pname):
            per[scope]["gt"] += len(gt)
            per[scope]["pred"] += len(tr)
            per[scope]["tp"] += len(pairs)
            per[scope]["fn"] += len(gt) - len(pairs)
            per[scope]["fp"] += len(tr) - len(pairs)
        for gi, (rid, *_rest) in enumerate(gt):
            per[rid]["gt"] += 1
            if gi not in matched_g:
                per[rid]["fn"] += 1
        for i, j in pairs:
            rid, tid = gt[i][0], tr[j]["id"]
            e = float(C[i, j])
            errs[rid].append(e)
            errs[pname].append(e)
            all_err.append(e)
            per[rid]["tp"] += 1
            overlap[(tid, rid)] += 1
            if last[rid] is not None and last[rid] != tid:
                for scope in ("all", pname, rid):
                    per[scope]["idsw"] += 1
            last[rid] = tid
            if tr[j]["alliance"]:
                alliance_n += 1
                alliance_ok += tr[j]["alliance"] == ally[rid]

    # IDF1: best one-to-one track->robot mapping over the whole match
    tids = sorted(track_frames)
    if tids:
        M = np.array([[overlap[(tid, rid)] for rid in ids] for tid in tids], float)
        idtp = sum(M[i, j] for i, j in _assign(-M))
    else:
        idtp = 0.0
    gt_total, pred_total = per["all"]["gt"], per["all"]["pred"]

    def summary(c: Counter, e: list) -> dict:
        d = {
            "robot_frames": int(c["gt"]),
            "recall": round(c["tp"] / c["gt"], 4) if c["gt"] else None,
            "id_switches": int(c["idsw"]),
            "median_error_m": round(float(np.median(e)), 4) if e else None,
            "mean_error_m": round(float(np.mean(e)), 4) if e else None,
            "p90_error_m": round(float(np.percentile(e, 90)), 4) if e else None,
        }
        if c["pred"] or "pred" in c:
            d["precision"] = round(c["tp"] / c["pred"], 4) if c["pred"] else None
            d["false_track_frames"] = int(c["fp"])
            d["mota"] = round(1 - (c["fn"] + c["fp"] + c["idsw"]) / c["gt"], 4) if c["gt"] else None
        return d

    out = {
        "match_id": m.meta.get("match_id"),
        "tracks": tracks_csv,
        "gate_m": gate_m,
        "frames_evaluated": sum(1 for t in times if not (skip_pause and period(t) == "pause")),
        "overall": summary(per["all"], all_err),
        "idf1": round(2 * idtp / (gt_total + pred_total), 4) if gt_total + pred_total else None,
        "alliance_accuracy": round(alliance_ok / alliance_n, 4) if alliance_n else None,
        "periods": {n: summary(per[n], errs[n]) for n, _, _ in periods if per[n]["gt"]},
        "robots": {rid: summary(per[rid], errs[rid]) for rid in ids},
        "track_ids": len(tids),
    }
    return out


def print_eval(r: dict) -> None:
    o = r["overall"]
    fmt = lambda v, p=1: "-" if v is None else f"{100 * v:.{p}f}%"
    cm = lambda v: "-" if v is None else f"{100 * v:.1f} cm"
    print(f"match {r['match_id']}: {r['frames_evaluated']} frames, gate {r['gate_m']} m, {r['track_ids']} track ids")
    print(f"  recall {fmt(o['recall'])}  precision {fmt(o.get('precision'))}  MOTA {fmt(o.get('mota'))}  IDF1 {fmt(r['idf1'])}  "
          f"id switches {o['id_switches']}  alliance {fmt(r['alliance_accuracy'])}")
    print(f"  position error: median {cm(o['median_error_m'])}, mean {cm(o['mean_error_m'])}, 90% {cm(o['p90_error_m'])}")
    for name, s in list(r["periods"].items()) + list(r["robots"].items()):
        print(f"  {name:>8}: recall {fmt(s['recall'])}, median error {cm(s['median_error_m'])}, id switches {s['id_switches']}")
