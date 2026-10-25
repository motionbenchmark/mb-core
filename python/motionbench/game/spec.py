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
        if name.startswith("gen-") and name not in lays:
            return name, self.generate_layout(int(name[4:]))
        if name not in lays:
            raise ValueError(f"unknown layout {name!r} (have: {', '.join(lays)}, or gen-<number> for a generated one)")
        return name, lays[name]

    def generate_layout(self, seed: int) -> dict:
        """A fresh layout drawn from ``layout_rules``: the same start, the same objects, new places.

        Public layouts can be learned by heart; generated ones test whether a routine
        (or a driver) copes with a field it hasn't seen. Official verdicts use seeds that
        are not published."""
        import numpy as np

        rules = self.raw.get("layout_rules")
        if not rules:
            raise ValueError(f"{self.key} has no layout_rules, so layouts can't be generated")
        rng = np.random.default_rng([seed, 7919])
        F = self.field
        start = dict(self.raw["layouts"][rules.get("start_from", next(iter(self.raw["layouts"])))]["start"])
        margin, spacing = rules.get("margin_m", 0.08), rules.get("min_spacing_m", 0.15)
        clear, pad = rules.get("clear_of_start_m", 0.25), rules.get("clear_of_zones_m", 0.05)
        base = self.raw["layouts"][rules.get("start_from", next(iter(self.raw["layouts"])))]
        objs: list[dict] = [dict(o) for o in base["objects"] if o["id"] in set(rules.get("keep", []))]
        for kind, count in rules["objects"].items():
            for k in range(int(count)):
                for _ in range(2000):
                    x = float(rng.uniform(margin, F["width_m"] - margin))
                    y = float(rng.uniform(margin, F["height_m"] - margin))
                    if math.hypot(x - start["x"], y - start["y"]) < clear:
                        continue
                    if any(z["x"] - pad <= x <= z["x"] + z["w"] + pad and z["y"] - pad <= y <= z["y"] + z["h"] + pad
                           for z in F.get("zones", {}).values()):
                        continue
                    if any(o["x"] - pad <= x <= o["x"] + o["w"] + pad and o["y"] - pad <= y <= o["y"] + o["h"] + pad
                           for o in F.get("obstacles", [])):
                        continue
                    if any(math.hypot(x - o["x"], y - o["y"]) < spacing for o in objs):
                        continue
                    break
                prefix = kind.split("-")[-1][0]
                objs.append({"id": f"{prefix}{k + 1}", "kind": kind, "x": round(x, 3), "y": round(y, 3)})
        return {"start": start, "objects": objs, "generated": {"seed": seed}}

    def object_kind(self, kind: str) -> dict:
        return self.raw["objects"][kind]

    @property
    def max_points(self) -> float:
        _, lay = self.layout()
        zone_pts = sum(self.points_for(o["kind"], z) for o in lay["objects"] for z in [self.best_zone(o["kind"])] if z)
        return zone_pts + self.target_points * len(self.target_ids(lay))

    # -- targets: AprilTag signs to knock down ----------------------------------------------------------
    @property
    def targets_rule(self) -> Optional[dict]:
        return self.raw["scoring"].get("targets")

    @property
    def target_points(self) -> float:
        r = self.targets_rule
        return float(r.get("points", 1)) if r else 0.0

    def target_ids(self, lay: dict) -> list[str]:
        r = self.targets_rule
        if not r:
            return []
        kinds = set(r.get("kinds", []))
        return [o["id"] for o in lay["objects"] if o["kind"] in kinds]

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
        tr = g["scoring"].get("targets")
        if tr:
            for k in tr.get("kinds", []):
                if k not in g["objects"]:
                    errs.append(f"targets rule names unknown object kind {k!r}")
                elif g["objects"][k].get("shape") != "tag":
                    errs.append(f"targets rule: object kind {k!r} must have shape 'tag'")
        for name, lay in g["layouts"].items():
            for o in lay.get("objects", []):
                if g["objects"].get(o.get("kind"), {}).get("shape") == "tag" and "tag_id" not in o:
                    errs.append(f"layout {name}: AprilTag sign {o.get('id')} needs a tag_id")
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

    def score_world(self, objects: list[dict], events: Optional[list] = None) -> dict:
        """``objects``: [{id, kind, x, y, held?}] at rest after the match. ``events``: the run's
        ``(t, kind, object_id, detail)`` events, needed for targets (which sign fell, and what hit it)
        and for rules that fail the run."""
        rows, total = [], 0.0
        tr = self.targets_rule or {}
        tkinds, valid_by = set(tr.get("kinds", [])), set(tr.get("valid_by", ["ball"]))
        down = {}
        for e in events or []:
            if e[1] == "target_down" and e[2] not in down:
                down[e[2]] = (e[0], e[3])
        for o in objects:
            pts, zone = 0.0, None
            if o["kind"] in tkinds:
                t, by = down.get(o["id"], (None, ""))
                ok = t is not None and by in valid_by
                pts = self.target_points if ok else 0.0
                total += pts
                rows.append({"id": o["id"], "kind": o["kind"], "target": True, "down": t is not None, "down_s": t,
                             "knocked_by": by, "points": pts})
                continue
            if not o.get("held"):
                for z in self.zone_of(o["x"], o["y"]):
                    p = self.points_for(o["kind"], z)
                    if p > pts:
                        pts, zone = p, z
            total += pts
            rows.append({"id": o["id"], "kind": o["kind"], "zone": zone, "points": pts, "held": bool(o.get("held"))})
        fails = [{"t": e[0], "rule": e[1], "object": e[2], "detail": e[3]} for e in events or []
                 if e[1] in self.raw["scoring"].get("fail_on", [])]
        out = {"points": total, "max_points": self.max_points, "objects": rows}
        if fails:
            out.update(points=0.0, points_before_fail=total, failed=fails[0]["detail"], fails=fails)
        return out


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
