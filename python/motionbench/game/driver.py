"""Simulated drivers: people with a controller, each with a personality.

A driver sees the field with a delay (reaction time), steers with some shake
(motor skill), decides which object to go for (decision quality), lines up
deliveries more or less carefully (precision), hesitates sometimes, and makes
the occasional mistake (letting go too early, overshooting). They drive the
AIM robot like the One Stick controller: forward and back, turn, and a little
sideways for the skilled ones.

These are models of people, chosen to span a classroom, not measurements of
anyone. Every number is an assumption and can be changed.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np

from .world import DT, ROBOT_R, V_MAX, W_MAX, World, _wrap


@dataclass
class Driver:
    id: str
    name: str
    reaction_s: float  # how old the picture is that they react to
    speed_use: float  # how much of full speed they dare to use (0..1)
    steer_shake_deg: float  # steering shake
    precision: float  # 0..1: how carefully they line up and place
    decisions: float  # 0..1: chance of picking the sensible next object
    mistakes: float  # chance per delivery of a fumble
    hesitate: float  # chance per decision of pausing
    strafe: float = 0.0  # 0..1: how much they use sideways driving
    blurb: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


CHARACTERS = [
    Driver("rookie-riley", "Rookie Riley", 0.55, 0.55, 9.0, 0.30, 0.55, 0.16, 0.25, 0.0,
           "First week with the controller. Slow to react, overshoots, sometimes heads for the wrong barrel."),
    Driver("nervous-noah", "Nervous Noah", 0.45, 0.60, 5.0, 0.60, 0.70, 0.07, 0.40, 0.1,
           "Knows what to do but stops to think a lot."),
    Driver("average-alex", "Average Alex", 0.40, 0.70, 5.0, 0.55, 0.75, 0.08, 0.15, 0.2,
           "A typical student driver after a few sessions."),
    Driver("speedy-sloane", "Speedy Sloane", 0.30, 1.00, 7.0, 0.40, 0.70, 0.13, 0.05, 0.3,
           "Full speed everywhere. Fast when it works, messy when it doesn't."),
    Driver("steady-sam", "Steady Sam", 0.35, 0.55, 3.0, 0.85, 0.85, 0.03, 0.10, 0.3,
           "Slow and careful. Rarely makes a mistake."),
    Driver("pro-priya", "Pro Priya", 0.22, 0.95, 2.0, 0.85, 0.95, 0.02, 0.03, 0.6,
           "The team's best driver: quick reactions, clean lines, smart order."),
]


def by_id(did: str) -> Driver:
    for d in CHARACTERS:
        if d.id == did:
            return d
    raise ValueError(f"unknown driver {did!r} (have: {', '.join(d.id for d in CHARACTERS)})")


def class_roster(n: int = 20, seed: int = 0) -> list[Driver]:
    """A made-up class of drivers with traits spread like a real class might be."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        s = float(np.clip(rng.beta(2.2, 2.0), 0.05, 0.98))  # overall skill
        j = lambda sd: float(rng.normal(0, sd))
        out.append(Driver(
            f"student-{i + 1:02d}", f"Student {i + 1:02d}", reaction_s=float(np.clip(0.6 - 0.38 * s + j(0.04), 0.18, 0.8)),
            speed_use=float(np.clip(0.5 + 0.45 * s + j(0.1), 0.35, 1.0)), steer_shake_deg=float(np.clip(9 - 7 * s + j(1), 1.5, 12)),
            precision=float(np.clip(0.25 + 0.65 * s + j(0.08), 0.1, 0.95)), decisions=float(np.clip(0.5 + 0.45 * s + j(0.05), 0.4, 0.98)),
            mistakes=float(np.clip(0.17 - 0.15 * s + j(0.02), 0.01, 0.25)), hesitate=float(np.clip(0.3 - 0.25 * s + j(0.05), 0.02, 0.5)),
            strafe=float(np.clip(s - 0.2, 0, 0.8)), blurb=f"skill {s:.2f}"))
    return out


class DriverController:
    def __init__(self, world: World, d: Driver, seed: int):
        self.w, self.d = world, d
        self.rng = np.random.default_rng(seed)
        self.seen = deque()  # (t, x, y, th, [(id, x, y, held)...])
        self.state = "choose"
        self.target_obj: Optional[str] = None
        self.goal: Optional[tuple[float, float]] = None
        self.pause_until = 0.0
        self.backoff_until = 0.0
        self.phase = float(self.rng.uniform(0, 10))
        self.early_release = False

    # what the driver sees: the field as it was reaction_s ago
    def _perceive(self):
        w = self.w
        self.seen.append((w.t, w.x, w.y, w.th, [(o.id, o.x, o.y, o.held) for o in w.objects]))
        while len(self.seen) > 1 and self.seen[1][0] <= w.t - self.d.reaction_s:
            self.seen.popleft()
        return self.seen[0]

    def _sorted(self, oid, x, y) -> bool:
        kind = next(o.kind for o in self.w.objects if o.id == oid)
        z = self.w.game.best_zone(kind)
        return z in self.w.game.zone_of(x, y)

    def _choose(self, seen):
        _, x, y, th, objs = seen
        movable = {o.id for o in self.w.objects if o.shape != "tag" and self.w.game.best_zone(o.kind)}
        todo = [(oid, ox, oy) for oid, ox, oy, held in objs if oid in movable and not held and not self._sorted(oid, ox, oy)]
        if not todo:
            self.state = "done"
            return
        if self.rng.random() < self.d.decisions:
            # sensible: nearest, nudged towards ones near their own zone
            def cost(o):
                kind = next(ob.kind for ob in self.w.objects if ob.id == o[0])
                z = self.w.game.field["zones"][self.w.game.best_zone(kind)]
                zx, zy = z["x"] + z["w"] / 2, z["y"] + z["h"] / 2
                return math.hypot(o[1] - x, o[2] - y) + 0.5 * math.hypot(o[1] - zx, o[2] - zy)
            pick = min(todo, key=cost)
        else:
            pick = todo[int(self.rng.integers(len(todo)))]
        self.target_obj = pick[0]
        self.state = "fetch"
        if self.rng.random() < self.d.hesitate:
            self.pause_until = self.w.t + float(self.rng.uniform(0.3, 1.2))

    def _zone_point(self, kind):
        z = self.w.game.field["zones"][self.w.game.best_zone(kind)]
        spread = 1.0 - self.d.precision
        cx, cy = z["x"] + z["w"] / 2, z["y"] + z["h"] / 2
        # aim for the zone, deeper in when careful; sloppy drivers aim near the edge
        px = cx + self.rng.uniform(-0.5, 0.5) * z["w"] * (0.3 + 0.9 * spread)
        py = cy + self.rng.uniform(-0.5, 0.5) * z["h"] * (0.3 + 0.9 * spread)
        return px, py

    def step(self):
        w, d = self.w, self.d
        seen = self._perceive()
        _, sx, sy, sth, objs = seen
        # people partly make up for their delay by anticipating how the robot is moving
        lead = 0.75 * d.reaction_s
        sx, sy, sth = sx + w.vx * lead, sy + w.vy * lead, sth + w.w * lead
        if w.t < self.pause_until:
            self._cmd(0, 0, 0)
            return
        if w.t < self.backoff_until:
            self._cmd(-0.6 * d.speed_use, 0, 0)
            return
        if self.state == "choose":
            self._choose(seen)
            return
        if self.state == "done":
            self._cmd(0, 0, 0)
            return
        if self.state == "fetch":
            if w.held is not None:
                self.state = "deliver"
                self.goal = self._zone_point(w.held.kind)
                self.early_release = self.rng.random() < d.mistakes
                return
            tgt = next(((ox, oy) for oid, ox, oy, held in objs if oid == self.target_obj and not held), None)
            if tgt is None or self._sorted(self.target_obj, *tgt):
                self.state = "choose"
                return
            self._drive_to(sx, sy, sth, tgt[0], tgt[1], arrive=0.0, face=True)
        elif self.state == "deliver":
            if w.held is None:  # lost it
                self.state = "choose"
                return
            gx, gy = self.goal
            dist = math.hypot(gx - sx, gy - sy)
            stop_at = 0.16 if self.early_release else 0.04 + 0.05 * (1 - d.precision)
            if dist < stop_at + self.w.R * 0.5:
                w.place()
                self.backoff_until = w.busy_until + 0.25 + 0.3 * (1 - d.precision)
                self.state = "choose"
                return
            self._drive_to(sx, sy, sth, gx, gy, arrive=stop_at, face=False)
        # stuck: no progress for 3 s -> back off and rethink
        if int(w.t * 10) % 30 == 0:
            key = (round(w.x, 2), round(w.y, 2))
            if getattr(self, "_stuck_key", None) == key and self.state in ("fetch", "deliver"):
                self.backoff_until = w.t + 0.6
                if self.state == "fetch":
                    self.state = "choose"
            self._stuck_key = key

    def _drive_to(self, sx, sy, sth, tx, ty, arrive: float, face: bool):
        d = self.d
        dx, dy = tx - sx, ty - sy
        dist = math.hypot(dx, dy)
        want = math.atan2(dy, dx)
        shake = math.radians(d.steer_shake_deg) * (math.sin(3.1 * self.w.t + self.phase) + 0.5 * self.rng.normal())
        err = _wrap(want - sth) + shake
        turn = float(np.clip(1.3 * err, -1, 1))
        near = dist < 0.25
        fwd = d.speed_use * max(0.0, math.cos(err)) ** 2 * (0.45 + 0.55 * min(1.0, (dist - arrive) / 0.3) if near else 1.0)
        side = 0.0
        if d.strafe > 0 and not face:
            # skilled drivers slide sideways instead of turning all the way
            side = float(np.clip(d.strafe * math.sin(err) * 1.2, -0.6, 0.6))
            fwd = d.speed_use * max(0.0, math.cos(err)) * (0.5 if near else 1.0)
        self._cmd(fwd, side, turn)

    def _cmd(self, fwd: float, side: float, turn: float):
        w = self.w
        fx, fy = math.cos(w.th), math.sin(w.th)
        rx, ry = math.sin(w.th), -math.cos(w.th)
        f, r = V_MAX * fwd, V_MAX * side
        w.cmd_v = (f * fx + r * rx, f * fy + r * ry)
        w.cmd_w = W_MAX * turn


def run_driver(world: World, d: Driver, seed: int, time_s: Optional[float] = None) -> dict:
    ctl = DriverController(world, d, seed)
    end = time_s or world.game.time_s
    k = 0
    while world.t < end - 1e-9:
        if k % 5 == 0:  # decisions 20 times a second
            ctl.step()
        world.step(DT)
        k += 1
        if ctl.state == "done" and world.held is None and world.t >= ctl.backoff_until:
            break
    world.cmd_v, world.cmd_w = (0.0, 0.0), 0.0
    return {"status": "finished" if ctl.state == "done" else "time", "error": None, "ended_s": round(world.t, 2)}
