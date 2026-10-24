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
