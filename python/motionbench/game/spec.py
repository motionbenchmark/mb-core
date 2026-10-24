"""Game definitions: created once, then simulated and scored many times.

A game lives in ``games/<name>/game.json`` (format ``motionbench-game`` 0.1):
the field (size, obstacles, zones, AprilTags on the walls), the kinds of
objects, one or more starting layouts, the match length, and the scoring rules
("an orange barrel at rest inside the orange zone scores 1 point"). Field
coordinates are metres from the near-left corner; headings are degrees
clockwise from +y (the far wall).

``create_game`` writes a new definition from a description you fill in; the
rules for a team's own game are usually worked out from their code and a short
description of the game, then checked with ``validate``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

FORMAT = "motionbench-game"
PLATFORMS = ("vex-aim",)


@dataclass
class Game:
    raw: dict
    path: Optional[Path] = None

    # -- loading --------------------------------------------------------------------------------
    @classmethod
    def load(cls, p: "str | Path") -> "Game":
        p = Path(p)
        if p.is_dir():
            p = p / "game.json"
        g = cls(json.loads(p.read_text()), p)
        errs = g.validate()
        if errs:
            raise ValueError(f"{p}: invalid game: " + "; ".join(errs))
        return g

    @property
    def key(self) -> str:
        return f"{self.raw['id']}@{self.raw['version']}"

    @property
    def field(self) -> dict:
        return self.raw["field"]

    @property
    def time_s(self) -> float:
        return float(self.raw["match"]["time_s"])

    def layout(self, name: Optional[str] = None) -> tuple[str, dict]:
        lays = self.raw["layouts"]
        name = name or next(iter(lays))
        if name not in lays:
            raise ValueError(f"unknown layout {name!r} (have: {', '.join(lays)})")
        return name, lays[name]

    def object_kind(self, kind: str) -> dict:
        return self.raw["objects"][kind]

    @property
    def max_points(self) -> float:
        _, lay = self.layout()
        return sum(self.points_for(o["kind"], z) for o in lay["objects"] for z in [self.best_zone(o["kind"])] if z)

    def best_zone(self, kind: str) -> Optional[str]:
        rules = [r for r in self.raw["scoring"]["rules"] if r["object"] == kind]
        return max(rules, key=lambda r: r["points"])["zone"] if rules else None

    def points_for(self, kind: str, zone: str) -> float:
        return max((r["points"] for r in self.raw["scoring"]["rules"] if r["object"] == kind and r["zone"] == zone), default=0)

    # -- checks -----------------------------------------------------------------------------------
    def validate(self) -> list[str]:
        g, errs = self.raw, []
        if g.get("format") != FORMAT:
            errs.append(f'"format" must be "{FORMAT}"')
        for k in ("id", "version", "title", "platform", "field", "objects", "layouts", "match", "scoring"):
            if k not in g:
                errs.append(f"missing {k!r}")
        if errs:
            return errs
        if g["platform"] not in PLATFORMS:
            errs.append(f"platform {g['platform']!r} is not simulated yet (have: {', '.join(PLATFORMS)})")
        W, H = g["field"]["width_m"], g["field"]["height_m"]
        zones = g["field"].get("zones", {})
        for name, lay in g["layouts"].items():
            s = lay.get("start", {})
            if not (0 < s.get("x", -1) < W and 0 < s.get("y", -1) < H):
                errs.append(f"layout {name}: start is outside the field")
            for o in lay.get("objects", []):
                if o.get("kind") not in g["objects"]:
                    errs.append(f"layout {name}: object {o.get('id')} has unknown kind {o.get('kind')!r}")
                if not (0 <= o.get("x", -1) <= W and 0 <= o.get("y", -1) <= H):
                    errs.append(f"layout {name}: object {o.get('id')} is outside the field")
        for r in g["scoring"].get("rules", []):
            if r.get("object") not in g["objects"]:
                errs.append(f"scoring rule for unknown object {r.get('object')!r}")
            if r.get("zone") not in zones:
                errs.append(f"scoring rule for unknown zone {r.get('zone')!r}")
        if g["match"].get("time_s", 0) <= 0:
            errs.append("match.time_s must be positive")
        return errs

    # -- scoring ------------------------------------------------------------------------------------
    def zone_of(self, x: float, y: float) -> list[str]:
        out = []
        for name, z in self.field.get("zones", {}).items():
            if z["x"] <= x <= z["x"] + z["w"] and z["y"] <= y <= z["y"] + z["h"]:
                out.append(name)
        return out

    def score_world(self, objects: list[dict]) -> dict:
        """``objects``: [{id, kind, x, y, held?}] at rest after the match."""
        rows, total = [], 0.0
        for o in objects:
            pts, zone = 0.0, None
            if not o.get("held"):
                for z in self.zone_of(o["x"], o["y"]):
                    p = self.points_for(o["kind"], z)
                    if p > pts:
                        pts, zone = p, z
            total += pts
            rows.append({"id": o["id"], "kind": o["kind"], "zone": zone, "points": pts, "held": bool(o.get("held"))})
        return {"points": total, "max_points": self.max_points, "objects": rows}


def heading_to_theta(heading_deg: float) -> float:
    """Field heading (degrees clockwise from +y) to a math angle (radians from +x)."""
    return math.radians(90.0 - heading_deg)


def create_game(out_dir: "str | Path", *, id: str, title: str, platform: str = "vex-aim", width_m: float = 1.8288,
                height_m: float = 1.2192, time_s: float = 60.0, description: str = "") -> Path:
    """Write a skeleton game.json to fill in (field, objects, layouts, scoring)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    g: dict[str, Any] = {
        "format": FORMAT, "format_version": "0.1", "id": id, "version": "0.1", "title": title, "platform": platform,
        "description": description, "source": {"inferred_from": [], "notes": ""},
        "field": {"width_m": width_m, "height_m": height_m, "obstacles": [], "zones": {}, "tags": []},
        "objects": {}, "layouts": {"dev-01": {"start": {"x": width_m / 2, "y": 0.12, "heading_deg": 0}, "objects": []}},
        "match": {"time_s": time_s, "mode": "skills", "robots": 1},
        "scoring": {"rules": [], "counts_when": "at rest after the match, centre inside the zone, not touching the robot"},
    }
    (out / "game.json").write_text(json.dumps(g, indent=2))
    (out / "routines").mkdir(exist_ok=True)
    return out / "game.json"
