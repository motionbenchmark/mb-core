"""Per-robot match report: how each robot moved, where it spent its time, and how
often it touched other robots and the walls, split by match period.

Works on any match directory, simulated or tracked: everything is computed from
the poses alone (contacts from distances), so a tracker's output can be
reported exactly like ground truth. When the match has an events log (simulated
matches do), pickups and deliveries are added as ground-truth counts. There is
no game score.
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np

from .io import MatchData

TOUCH_M = 0.50  # centres closer than this: the robots are touching (18 in robots)
WALL_M = 0.26  # centre this close to the perimeter: touching the wall
IDLE_MPS = 0.05


def _periods(m: MatchData) -> list[tuple[str, float, float]]:
    return [(p["name"], p["start_s"], p["end_s"]) for p in m.meta.get("periods", [])] or [("match", 0.0, m.duration)]


def _speed(p: np.ndarray) -> np.ndarray:
    if len(p) < 3:
        return np.zeros(len(p))
    dt = np.gradient(p[:, 0])
    v = np.hypot(np.gradient(p[:, 1]), np.gradient(p[:, 2])) / np.maximum(dt, 1e-6)
    k = 5
    return np.convolve(np.pad(v, (k // 2, k // 2), mode="edge"), np.ones(k) / k, mode="valid")


def _episodes(mask: np.ndarray) -> int:
    return int(np.sum(np.diff(np.r_[0, mask.astype(int)]) == 1))


def report(root, periods: Optional[list[str]] = None) -> dict:
    m = MatchData(root)
    F = m.meta["field"]["size_m"]
    zones = m.meta["field"]["zones"]
    ids = [r["id"] for r in m.robots]
    ally = {r["id"]: r["alliance"] for r in m.robots}
    # common time base
    t = m.poses[ids[0]][:, 0]
    P = {i: np.array([m.pose(i, tt)[:2] for tt in t]) if len(m.poses[i]) != len(t) or not np.allclose(m.poses[i][:, 0], t)
         else m.poses[i][:, 1:3] for i in ids}
    dt = np.gradient(t)
    out = {"match_id": m.meta.get("match_id"), "synthetic": m.meta.get("synthetic", False), "duration_s": float(t[-1] - t[0]),
           "robots": {}, "alliances": {}, "pairs": []}
    sel = [p for p in _periods(m) if p[0] != "pause" and (not periods or p[0] in periods)]

    def inside(z, xy):
        return (xy[:, 0] >= z["x"]) & (xy[:, 0] <= z["x"] + z["w"]) & (xy[:, 1] >= z["y"]) & (xy[:, 1] <= z["y"] + z["h"])

    for rid in ids:
        xy = P[rid]
        v = _speed(np.c_[t, xy])
        own = "red-half" if ally[rid] == "red" else "blue-half"
        opp = "blue-half" if ally[rid] == "red" else "red-half"
        wall = (xy[:, 0] < WALL_M) | (xy[:, 0] > F - WALL_M) | (xy[:, 1] < WALL_M) | (xy[:, 1] > F - WALL_M)
        near = {o: np.hypot(*(xy - P[o]).T) < TOUCH_M for o in ids if o != rid}
        rr = {"alliance": ally[rid], "team": next(r.get("team", "") for r in m.robots if r["id"] == rid), "periods": {}}
        for name, a, b in sel:
            k = (t >= a) & (t < b)
            if k.sum() < 2:
                continue
            seg = xy[k]
            dist = float(np.hypot(*np.diff(seg, axis=0).T).sum())
            T = float(dt[k].sum())
            cells = {(int(x / 0.25), int(y / 0.25)) for x, y in seg}
            moving = v[k] > IDLE_MPS
            rr["periods"][name] = {
                "time_s": round(T, 2),
                "distance_m": round(dist, 2),
                "mean_speed_mps": round(dist / T, 3) if T else 0.0,
                "top_speed_mps": round(float(np.percentile(v[k], 98)), 3),
                "idle_pct": round(100 * float(1 - moving.mean()), 1),
                "field_cells_visited_pct": round(100 * len(cells) / int(np.ceil(F / 0.25)) ** 2, 1),
                "own_half_pct": round(100 * float(inside(zones[own], seg).mean()), 1),
                "opponent_half_pct": round(100 * float(inside(zones[opp], seg).mean()), 1),
                "centre_mat_pct": round(100 * float(inside(zones["centre-mat"], seg).mean()), 1),
                "park_zone_pct": round(100 * float(inside(zones[f"park-{ally[rid]}"], seg).mean()), 1),
                "robot_contacts": int(sum(_episodes(near[o][k]) for o in near)),
                "robot_contact_s": round(float(sum((near[o][k] * dt[k]).sum() for o in near)), 2),
                "opponent_contacts": int(sum(_episodes(near[o][k]) for o in near if ally[o] != ally[rid])),
                "wall_contacts": _episodes(wall[k]),
                "wall_contact_s": round(float((wall[k] * dt[k]).sum()), 2),
            }
            if m.events:
                ev = [e for e in m.events if e["robot"] == rid and a <= e["t_s"] < b]
                rr["periods"][name]["pickups"] = sum(e["type"] in ("pickup", "load") for e in ev)
                rr["periods"][name]["deliveries"] = sum(e["type"] == "score" for e in ev)
        # whole match = the selected periods together
        tot: dict = {}
        for pr in rr["periods"].values():
            for kk, vv in pr.items():
                if kk.endswith("_pct") or kk.endswith("_mps"):
                    continue
                tot[kk] = round(tot.get(kk, 0) + vv, 2)
        if tot.get("time_s"):
            tot["mean_speed_mps"] = round(tot["distance_m"] / tot["time_s"], 3)
            ws = [(pr["time_s"], pr) for pr in rr["periods"].values()]
            for kk in ("idle_pct", "own_half_pct", "opponent_half_pct", "centre_mat_pct", "park_zone_pct"):
                tot[kk] = round(sum(w * pr[kk] for w, pr in ws) / tot["time_s"], 1)
            kmask = np.zeros(len(t), bool)
            for _, a, b in sel:
                kmask |= (t >= a) & (t < b)
            tot["field_cells_visited_pct"] = round(100 * len({(int(x / 0.25), int(y / 0.25)) for x, y in xy[kmask]}) / int(np.ceil(F / 0.25)) ** 2, 1)
            tot["top_speed_mps"] = max(pr["top_speed_mps"] for pr in rr["periods"].values())
        rr["match"] = tot
        rr["parked_at_end"] = bool(inside(zones[f"park-{ally[rid]}"], xy[-1:])[0])
        out["robots"][rid] = rr
    for a in ("red", "blue"):
        rs = [out["robots"][i]["match"] for i in ids if ally[i] == a]
        out["alliances"][a] = {"distance_m": round(sum(r.get("distance_m", 0) for r in rs), 2),
                               "robot_contacts": int(sum(r.get("robot_contacts", 0) for r in rs)),
                               "parked": [i for i in ids if ally[i] == a and out["robots"][i]["parked_at_end"]]}
        if m.events:
            out["alliances"][a]["deliveries"] = int(sum(r.get("deliveries", 0) for r in rs))
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            d = np.hypot(*(P[a] - P[b]).T)
            out["pairs"].append({"robots": [a, b], "contacts": _episodes(d < TOUCH_M), "contact_s": round(float(((d < TOUCH_M) * dt).sum()), 2),
                                 "closest_m": round(float(d.min()), 3)})
    return out


def print_report(r: dict) -> None:
    tag = " (synthetic)" if r.get("synthetic") else ""
    print(f"match {r['match_id']}{tag}: {r['duration_s']:.0f} s")
    cols = ["distance_m", "mean_speed_mps", "idle_pct", "field_cells_visited_pct", "opponent_half_pct", "robot_contacts", "wall_contacts"]
    head = ["robot", "period", "dist m", "m/s", "idle %", "cells %", "opp half %", "robot hits", "wall hits"]
    has_ev = any("deliveries" in p for rr in r["robots"].values() for p in rr["periods"].values())
    if has_ev:
        cols.append("deliveries")
        head.append("delivered")
    print("  ".join(f"{h:>10}" for h in head))
    for rid, rr in r["robots"].items():
        for name, p in list(rr["periods"].items()) + [("match", rr["match"])]:
            vals = [f"{p.get(c, ''):>10}" for c in cols]
            print(f"{rid:>10}  {name:>10}  " + "  ".join(vals))
    for a, s in r["alliances"].items():
        extra = f", {s['deliveries']} delivered" if "deliveries" in s else ""
        print(f"{a}: {s['distance_m']} m driven, {s['robot_contacts']} robot contacts{extra}, parked: {', '.join(s['parked']) or 'none'}")


def draw_paths(root, out_png: str, px: int = 720) -> None:
    """A top-down picture of every robot's path (red alliance solid, blue dashed tint)."""
    from ..tracker.track import _cv2
    cv2 = _cv2()
    m = MatchData(root)
    F = m.meta["field"]["size_m"]
    s = px / F
    img = np.full((px, px, 3), (222, 225, 220), np.uint8)
    P = lambda x, y: (int(x * s), int((F - y) * s))
    for i in range(1, 6):
        cv2.line(img, (int(i * px / 6), 0), (int(i * px / 6), px), (200, 203, 198), 1)
        cv2.line(img, (0, int(i * px / 6)), (px, int(i * px / 6)), (200, 203, 198), 1)
    for o in m.meta["field"]["obstacles"]:
        cv2.rectangle(img, P(o["x"], o["y"] + o["h"]), P(o["x"] + o["w"], o["y"]), (70, 75, 80), -1)
    for name, z in m.meta["field"]["zones"].items():
        if name.startswith("park"):
            cv2.rectangle(img, P(z["x"], z["y"] + z["h"]), P(z["x"] + z["w"], z["y"]), (60, 60, 200) if "red" in name else (200, 120, 40), 2)
    shades = {"red1": (40, 40, 200), "red2": (110, 100, 235), "blue1": (190, 100, 20), "blue2": (235, 180, 100)}
    for r in m.robots:
        p = m.poses[r["id"]]
        pts = np.array([P(x, y) for x, y in p[:, 1:3]], np.int32)
        cv2.polylines(img, [pts], False, shades.get(r["id"], (0, 0, 0)), 2, cv2.LINE_AA)
        cv2.circle(img, tuple(pts[0]), 5, shades.get(r["id"], (0, 0, 0)), -1)
        cv2.putText(img, r["id"], (int(pts[-1][0]) + 6, int(pts[-1][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.imwrite(out_png, img)


def main_json(root) -> str:
    return json.dumps(report(root), indent=1)
