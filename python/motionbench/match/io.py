"""Match datasets on disk.

    match-0001/
      match.json            field, periods, robots (alliance, size, look), camera, seed
      objects.csv           t_s,id,state,x_m,y_m,z_m,ref   (one row whenever a block changes)
      events.csv            t_s,robot,type,other,detail    (pickups, scores, contacts, parking)
      robots/red1/ ...      one .mbtrace per robot (poses at 20 Hz), scorable on its own
      tasks/MB-MATCH-FIELD@0.1.json

A block's position at any time is its last ``objects.csv`` row; while it is
``carried`` it travels with the robot named in ``ref``.
"""

from __future__ import annotations

import csv
import math
import json
from pathlib import Path
from typing import Optional

import numpy as np

from ..trace import write_trace
from .field import FIELD_M
from .sim import PERIODS, MatchResult

TASK_ID, TASK_VERSION = "MB-MATCH-FIELD", "0.1"
BROADCAST_CAMERA = {
    "note": "Fitted to a real event broadcast: a fixed camera high above the near wall (only these numbers are kept).",
    "frame_size": [1020, 696],
    "f_px": 1138.7,
    "camera_height_m": 3.72,
    "tilt_deg": 43.7,
    "floor_corners_px": [[200, 125], [800, 125], [984, 656], [15, 659]],
    "field_m": FIELD_M,
}


def camera_from_calib(calib) -> dict:
    """A match camera from a tap-to-calibrate ``calib.json`` (``motionbench calibrate --vex-field``):
    the four field corners plus the field's measured size give focal length and height."""
    from ..tracker.untagged import Calibration, fit_camera

    if not isinstance(calib, Calibration):
        calib = Calibration.load(calib)
    if not (calib.measured_width_m and calib.measured_length_m):
        raise ValueError("calib.json needs the field's measured width and length (use --vex-field or --field)")
    cam, info = fit_camera(calib, [], tuple(calib.frame_size))
    return {
        "note": "Fitted from calib.json (four field corners and the measured field size).",
        "frame_size": list(calib.frame_size),
        "f_px": float(cam.f),
        "camera_height_m": float(cam.h),
        "tilt_deg": round(cam.tilt_deg(), 2),
        "floor_corners_px": [list(map(float, p)) for p in calib.floor_corners_px],
        "field_m": float(calib.measured_width_m),
    }


def match_task(res: MatchResult, match_id: str) -> dict:
    fld = res.field
    return {
        "id": TASK_ID,
        "version": TASK_VERSION,
        "title": "Four-robot match on a VEX V5 field (per-robot view)",
        "domain": "vex-v5",
        "description": "One robot's path through a 2 v 2 match: 15 s autonomous, a short pause, 105 s driver control. "
                       "Scored as field coverage so every robot's trace can be checked with the standard scorer; match "
                       "metrics (contacts, zones, periods) come from `motionbench match report`.",
        "arena": {"width_m": fld.size_m, "height_m": fld.size_m,
                  "obstacles": [{"x": round(o.x, 3), "y": round(o.y, 3), "w": round(o.w, 3), "h": round(o.h, 3), "label": o.label} for o in fld.obstacles],
                  "zones": {}},
        "robot": {"radius_m": 0.229, "contact_tolerance_m": 0.06},
        "budget": {"time_s": PERIODS[-1][2]},
        "goal": {"kind": "coverage", "cell_m": 0.1, "tool_width_m": 0.40},
        "reference": {"floor": 0},
        "layouts": {f"{match_id}-{s.id}": {"start": {"x": float(res.poses[s.id][0, 1]), "y": float(res.poses[s.id][0, 2]),
                                                     "theta": float(res.poses[s.id][0, 3])}} for s in res.specs},
    }


def write_match(res: MatchResult, out: "str | Path", match_id: Optional[str] = None, camera: Optional[dict] = None) -> Path:
    out = Path(out)
    match_id = match_id or f"m{res.seed:04d}"
    (out / "tasks").mkdir(parents=True, exist_ok=True)
    (out / "tasks" / f"{TASK_ID}@{TASK_VERSION}.json").write_text(json.dumps(match_task(res, match_id), indent=1))
    meta = {
        "format": "motionbench-match",
        "version": "0.1",
        "match_id": match_id,
        "synthetic": True,
        "seed": res.seed,
        "periods": [{"name": n, "start_s": a, "end_s": b} for n, a, b in PERIODS],
        "field": res.field.to_dict(),
        "robots": [dict(res.robot_meta(s), trace=f"robots/{s.id}") for s in res.specs],
        "blocks": list(res.blocks.values()),
        "parked": res.parked,
        "camera": camera or BROADCAST_CAMERA,
    }
    (out / "match.json").write_text(json.dumps(meta, indent=1))
    with open(out / "objects.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "id", "state", "x_m", "y_m", "z_m", "ref"])
        w.writerows(res.obj_log)
    with open(out / "events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "robot", "type", "other", "detail"])
        w.writerows(res.events)
    for s in res.specs:
        ev = [(t, kind, other, detail) for t, rid, kind, other, detail in res.events if rid == s.id]
        write_trace(out / "robots" / s.id, task=f"{TASK_ID}@{TASK_VERSION}", seed=f"{match_id}-{s.id}", agent_id=f"team-{s.team}",
                    agent_class="human", agent_name=f"{s.team} ({s.alliance})", poses=res.poses[s.id].tolist(), events=ev,
                    agent_meta={"alliance": s.alliance, "robot": s.id, "auton": "human_auton for 0-15 s", "synthetic": True},
                    tier="S", capture="sim", notes="Synthetic match robot (motionbench match synth).", decimals=4)
    return out


class MatchData:
    """A match read back from disk, with helpers to ask where things were at time t."""

    def __init__(self, root: "str | Path"):
        self.root = Path(root)
        self.meta = json.loads((self.root / "match.json").read_text())
        self.robots = self.meta["robots"]
        self.poses: dict[str, np.ndarray] = {}
        for r in self.robots:
            rows = list(csv.DictReader(open(self.root / r["trace"] / "poses.csv")))
            self.poses[r["id"]] = np.array([[float(x["t_s"]), float(x["x_m"]), float(x["y_m"]), float(x["theta_rad"])] for x in rows])
        self.events = [dict(e, t_s=float(e["t_s"])) for e in csv.DictReader(open(self.root / "events.csv"))] \
            if (self.root / "events.csv").exists() else []
        self.colours = {b["id"]: b["colour"] for b in self.meta.get("blocks", [])}
        self._obj: dict[str, list] = {}
        self._obj_t: dict[str, np.ndarray] = {}
        if (self.root / "objects.csv").exists():
            for o in csv.DictReader(open(self.root / "objects.csv")):
                self._obj.setdefault(o["id"], []).append((float(o["t_s"]), o["state"], float(o["x_m"]), float(o["y_m"]), float(o["z_m"]), o["ref"]))
        self.duration = max(p[-1, 0] for p in self.poses.values())

    def pose(self, rid: str, t: float) -> tuple[float, float, float]:
        p = self.poses[rid]
        i = int(np.clip(np.searchsorted(p[:, 0], t) - 1, 0, len(p) - 2))
        a, b = p[i], p[i + 1]
        s = float(np.clip((t - a[0]) / max(b[0] - a[0], 1e-9), 0, 1))
        dth = (b[3] - a[3] + np.pi) % (2 * np.pi) - np.pi
        return a[1] + (b[1] - a[1]) * s, a[2] + (b[2] - a[2]) * s, a[3] + dth * s

    def objects_at(self, t: float) -> list[dict]:
        out = []
        for oid, rows in self._obj.items():
            ts = self._obj_t.setdefault(oid, np.array([r[0] for r in rows]))
            k = max(0, int(np.searchsorted(ts, t, side="right")) - 1)
            _, state, x, y, z, ref = rows[k]
            if state == "carried" and ref in self.poses:
                x, y, _ = self.pose(ref, t)
            out.append({"id": oid, "colour": self.colours.get(oid, "red"), "state": state, "x": x, "y": y, "z": z, "ref": ref})
        return out


def match_from_tracks(tracks_csv: "str | Path", out: "str | Path", *, camera: Optional[dict] = None, n_robots: int = 4,
                      match_id: str = "tracked", names: Optional[dict] = None, duration_s: Optional[float] = None) -> Path:
    """Turn tracker output into a match folder that ``motionbench match report`` reads.

    Broken tracks are stitched: a track that starts within 3 s and 1 m of where another
    ended (and does not disagree on alliance) continues it. The ``n_robots`` longest
    chains become robots, named red1/red2/blue1/blue2 by their alliance where it is known
    (else r1, r2, ...), or by ``names`` ({track_id or robot: name}).
    """
    from .field import default_field

    rows: dict[str, list] = {}
    ally: dict[str, str] = {}
    with open(tracks_csv) as f:
        for r in csv.DictReader(f):
            rows.setdefault(r["track_id"], []).append((float(r["t_s"]), float(r["x_m"]), float(r["y_m"]), float(r.get("observed") or 1)))
            if r.get("alliance"):
                ally[r["track_id"]] = r["alliance"]
    segs = []
    for tid, v in rows.items():
        a = np.array(sorted(v))
        seen = np.nonzero(a[:, 3] > 0)[0]
        if len(seen) >= 3:  # drop the coasted tail after the tracker last saw it
            segs.append((tid, a[: seen[-1] + 1, :3]))
    segs.sort(key=lambda s: s[1][0, 0])
    chains: list[dict] = []
    for tid, p in segs:
        best, bd = None, 1e9
        for c in chains:
            gap = p[0, 0] - c["p"][-1, 0]
            d = math.hypot(p[0, 1] - c["p"][-1, 1], p[0, 2] - c["p"][-1, 2])
            a, b = ally.get(tid), c["alliance"]
            close = gap <= 1.0 and d <= 0.5  # picked up where it was lost: continuity beats a colour guess
            if 0 <= gap <= 3.0 and d <= 1.0 and (close or not (a and b and a != b)) and gap + d < bd:
                best, bd = c, gap + d
        if best is None:
            chains.append({"ids": [tid], "p": p, "alliance": ally.get(tid), "votes": {ally[tid]: len(p)} if ally.get(tid) else {}})
        else:
            best["ids"].append(tid)
            best["p"] = np.vstack([best["p"], p])
            if ally.get(tid):
                best["votes"][ally[tid]] = best["votes"].get(ally[tid], 0) + len(p)
                best["alliance"] = max(best["votes"], key=best["votes"].get)
    chains.sort(key=lambda c: -(c["p"][-1, 0] - c["p"][0, 0]))
    chains = chains[:n_robots]
    count = {"red": 0, "blue": 0}
    fld = default_field()
    F = fld.size_m
    t_end = duration_s or max(c["p"][-1, 0] for c in chains)
    out = Path(out)
    robots = []
    for k, c in enumerate(chains):
        a = c["alliance"]
        if a in count and count[a] < n_robots // 2:
            count[a] += 1
            rid = f"{a}{count[a]}"
        else:
            rid, a = f"r{k + 1}", a or "unknown"
        nm = (names or {}).get(rid) or next(((names or {}).get(i) for i in c["ids"] if (names or {}).get(i)), None)
        p = c["p"]
        # regular 10 Hz poses; heading from travel direction
        ts = np.arange(max(0.0, p[0, 0]), min(t_end, p[-1, 0]) + 1e-9, 0.1)
        xs, ys = np.interp(ts, p[:, 0], p[:, 1]), np.interp(ts, p[:, 0], p[:, 2])
        th = np.unwrap(np.arctan2(np.gradient(ys), np.gradient(xs)))
        poses = np.c_[ts, np.clip(xs, 0, F), np.clip(ys, 0, F), th]
        write_trace(out / "robots" / rid, task=f"{TASK_ID}@{TASK_VERSION}", seed=f"{match_id}-{rid}", agent_id=nm or rid,
                    agent_class="human", agent_name=nm or rid, poses=poses.tolist(), tier="F", capture="video",
                    agent_meta={"alliance": a, "tracks": c["ids"]}, notes="Tracked from match video (motionbench match track).",
                    decimals=4)
        robots.append({"id": rid, "alliance": a, "team": nm or "", "trace": f"robots/{rid}", "tracks": c["ids"],
                       "size_m": [0.457, 0.457, 0.35], "chassis": "grey", "accent": "none"})
    meta = {"format": "motionbench-match", "version": "0.1", "match_id": match_id, "synthetic": False, "seed": 0,
            "periods": [{"name": n, "start_s": a_, "end_s": b} for n, a_, b in PERIODS], "field": fld.to_dict(),
            "robots": robots, "blocks": [], "camera": camera or BROADCAST_CAMERA, "source": str(tracks_csv)}
    (out / "match.json").write_text(json.dumps(meta, indent=1))
    return out
