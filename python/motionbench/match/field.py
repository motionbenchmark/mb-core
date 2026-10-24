"""A game-neutral VEX V5 competition field for multi-robot matches.

Coordinates are metres on the floor inside the perimeter: origin at the near-left
corner as seen from the broadcast camera, +x to the right, +y away from the
camera. Red plays from the left (x near 0), blue from the right.

The structures only resemble a recent season's field closely enough that the
tracking problem is realistic (raised bars robots park beside, a centre
structure, side zones, corner loaders and alliance-coloured blocks). They are not
a rules model, and nothing here keeps a game score.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

FIELD_M = 3.566  # 6 x 6 foam tiles of 0.594 m
TILE_M = FIELD_M / 6
WALL_H = 0.08  # visible height of the perimeter above the tiles (the rail is taller off-field)
ALLIANCES = ("red", "blue")


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float
    label: str = ""

    def contains(self, px: float, py: float, pad: float = 0.0) -> bool:
        return self.x - pad <= px <= self.x + self.w + pad and self.y - pad <= py <= self.y + self.h + pad

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.w / 2, self.y + self.h / 2


@dataclass
class Goal:
    """A place blocks are delivered: robots stop at ``approach`` facing ``heading``."""

    name: str
    approach: tuple[float, float]
    heading: float
    slots: list[tuple[float, float, float]]  # where delivered blocks sit (x, y, z)
    shared_with: Optional[str] = None  # another goal filling the same slots from the other end


@dataclass
class Field:
    size_m: float = FIELD_M
    obstacles: list[Rect] = field(default_factory=list)  # robots cannot enter these footprints
    zones: dict[str, Rect] = field(default_factory=dict)
    goals: dict[str, Goal] = field(default_factory=dict)
    loaders: dict[str, tuple[float, float]] = field(default_factory=dict)
    bars: list[dict] = field(default_factory=list)  # raised bars, for rendering

    def to_dict(self) -> dict:
        return {
            "size_m": self.size_m,
            "obstacles": [r.__dict__ for r in self.obstacles],
            "zones": {k: r.__dict__ for k, r in self.zones.items()},
            "goals": {k: {"approach": g.approach, "heading": g.heading, "n_slots": len(g.slots)} for k, g in self.goals.items()},
            "loaders": self.loaders,
            "bars": self.bars,
        }


def default_field() -> Field:
    F = FIELD_M
    c = F / 2
    f = Field()
    bar_len, bar_y = 1.19, 0.60
    x0, x1 = c - bar_len / 2, c + bar_len / 2
    for name, y in (("near", bar_y), ("far", F - bar_y)):
        f.obstacles.append(Rect(x0, y - 0.07, bar_len, 0.14, f"long-goal-{name}"))
        f.bars.append({"name": name, "x0": x0, "x1": x1, "y": y, "z": 0.30, "depth": 0.12})
        slots = [(x0 + 0.04 + i * (bar_len - 0.08) / 14, y, 0.36) for i in range(15)]
        f.goals[f"long-{name}-left"] = Goal(f"long-{name}-left", (x0 - 0.33, y), 0.0, slots, f"long-{name}-right")
        f.goals[f"long-{name}-right"] = Goal(f"long-{name}-right", (x1 + 0.33, y), np.pi, slots[::-1], f"long-{name}-left")
    f.obstacles.append(Rect(c - 0.16, c - 0.16, 0.32, 0.32, "centre-goal"))
    d = 0.42 / np.sqrt(2)
    centre_slots = [(c + 0.05 * np.cos(k * 1.3), c + 0.05 * np.sin(k * 1.3), 0.12 + 0.07 * k) for k in range(7)]
    for name, (sx, sy) in {"centre-nl": (-1, -1), "centre-nr": (1, -1), "centre-fl": (-1, 1), "centre-fr": (1, 1)}.items():
        f.goals[name] = Goal(name, (c + sx * d, c + sy * d), float(np.arctan2(-sy, -sx)), centre_slots,
                             "centre" if name != "centre-nl" else None)
    f.zones["park-red"] = Rect(0.0, c - 0.34, 0.42, 0.68, "park-red")
    f.zones["park-blue"] = Rect(F - 0.42, c - 0.34, 0.42, 0.68, "park-blue")
    f.zones["centre-mat"] = Rect(c - 2 * TILE_M + TILE_M * 0.5, c - 2 * TILE_M + TILE_M * 0.5, 3 * TILE_M, 3 * TILE_M, "centre-mat")
    f.zones["red-half"] = Rect(0, 0, c, F, "red-half")
    f.zones["blue-half"] = Rect(c, 0, c, F, "blue-half")
    f.loaders = {"red-near": (0.06, 0.62), "red-far": (0.06, F - 0.62), "blue-near": (F - 0.06, 0.62), "blue-far": (F - 0.06, F - 0.62)}
    return f


def initial_blocks(rng: np.random.Generator, fld: Field) -> list[dict]:
    """Starting block positions: clusters by the bars and the centre, rows in the
    park zones, stacks in the loaders, and a few strays near the walls."""
    F, c = fld.size_m, fld.size_m / 2
    out: list[dict] = []

    def add(x, y, colour, state="floor", ref="", z=0.0):
        out.append({"id": f"b{len(out):02d}", "colour": colour, "x": float(x), "y": float(y), "z": z, "state": state, "ref": ref})

    jitter = lambda s=0.015: rng.normal(0, s)
    for y in (0.60 + 0.20, F - 0.60 - 0.20):  # rows of four just inside each bar
        for k in range(4):
            col = "red" if k < 2 else "blue"
            add(c + (k - 1.5) * 0.10 + jitter(), y + jitter(), col)
    for sx, sy in ((-1, -1), (1, 1), (-1, 1), (1, -1)):  # pairs around the centre structure
        for k in range(2):
            add(c + sx * (0.55 + 0.09 * k) + jitter(), c + sy * 0.55 + jitter(), "red" if (sx < 0) == (k == 0) else "blue")
    for zone, col in (("park-red", "blue"), ("park-blue", "red")):  # rows inside the park zones
        r = fld.zones[zone]
        for k in range(6):
            add(r.center[0] + jitter(0.01), r.y + 0.1 + k * 0.095, col)
    for name, (lx, ly) in fld.loaders.items():  # stacks in the loader tubes
        for k in range(6):
            col = ("red" if name.startswith("red") else "blue") if k % 2 else ("blue" if name.startswith("red") else "red")
            add(lx, ly, col, "loader", name, 0.05 + 0.09 * k)
    for (x, y, col) in ((0.85, 0.12, "blue"), (0.95, 0.12, "blue"), (F - 0.85, F - 0.12, "red"), (F - 0.95, F - 0.12, "red"),
                        (F - 0.35, 0.15, "red"), (F - 0.25, 0.15, "red"), (0.3, F - 0.15, "blue"), (0.4, F - 0.15, "blue")):
        add(x + jitter(), y + jitter(), col)
    # loose pairs and triples scattered over the field, clear of the structures
    placed = 0
    while placed < 10:
        x, y = rng.uniform(0.3, F - 0.3), rng.uniform(0.25, F - 0.25)
        if any(o.contains(x, y, 0.15) for o in fld.obstacles) or any(np.hypot(b["x"] - x, b["y"] - y) < 0.2 for b in out):
            continue
        for k in range(int(rng.integers(2, 4))):
            add(x + 0.09 * k + jitter(), y + jitter(), "red" if rng.random() < 0.5 else "blue")
        placed += 1
    return out


def zone_of(fld: Field, x: float, y: float) -> list[str]:
    return [k for k, r in fld.zones.items() if r.contains(x, y)]
