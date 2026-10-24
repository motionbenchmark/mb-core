"""A small 2D world for one VEX AIM robot and the game's objects.

The AIM robot drives holonomically (it can move in any direction while
turning). Speeds follow the VEXcode AIM API: 100% move velocity is 200 mm/s and
turns top out at 150 deg/s. The robot holds one object at a time on its front
magnet (barrels) or in its cradle (balls), can place it, or kick it.

What makes simulated runs differ from each other, like real ones: each run
draws its own wheel slip (the robot travels a little more or less than its
encoders say), a slow sideways drift, turn overshoot, kick scatter and, for
vision, measurement noise and missed detections. Dead-reckoning code
accumulates those errors; code that looks at the field can correct them.

Assumptions, not AIM specifications: the robot's footprint (0.09 m radius),
accelerations, kick speeds and friction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .spec import Game, heading_to_theta

DT = 0.01
ROBOT_R = 0.09
V_MAX = 0.200  # m/s at 100%
W_MAX = math.radians(150)  # rad/s
ACCEL = 0.8  # m/s^2
W_ACCEL = math.radians(720)
KICK_SPEED = {"SOFT": 0.35, "MEDIUM": 0.7, "HARD": 1.2}
FRICTION = {"barrel": 2.5, "ball": 0.45}  # m/s^2 deceleration while sliding / rolling
CAPTURE_GAP = 0.018  # how close the object must be to the robot's front to be picked up
CAPTURE_BEARING = math.radians(28)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Obj:
    id: str
    kind: str
    shape: str
    r: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    held: bool = False
    released: bool = False  # just let go: not picked up again until the robot has moved away


@dataclass
class Noise:
    """One run's worth of imperfection, drawn once per run."""

    dist_scale: float = 1.0  # true distance / encoder distance
    drift: float = 0.0  # sideways slip, metres per metre driven
    turn_err_deg: float = 0.0  # overshoot sd for commanded turns
    kick_sd_deg: float = 3.0
    vision_sd_deg: float = 1.0
    vision_miss: float = 0.03
    speed_scale: float = 1.0  # battery

    @classmethod
    def draw(cls, rng: np.random.Generator, level: float = 1.0) -> "Noise":
        return cls(dist_scale=float(1 + rng.normal(0, 0.025 * level)), drift=float(rng.normal(0, 0.02 * level)),
                   turn_err_deg=1.2 * level, kick_sd_deg=3.0 * level, vision_sd_deg=1.0 * level,
                   vision_miss=0.03 * level, speed_scale=float(rng.uniform(0.94, 1.0)))


class World:
    def __init__(self, game: Game, layout: Optional[str] = None, seed: int = 0, noise_level: float = 1.0):
        self.game = game
        self.layout_name, lay = game.layout(layout)
        self.rng = np.random.default_rng(seed)
        self.noise = Noise.draw(self.rng, noise_level)
        F = game.field
        self.W, self.H = F["width_m"], F["height_m"]
        self.obstacles = F.get("obstacles", [])
        s = lay["start"]
        self.x, self.y, self.th = s["x"], s["y"], heading_to_theta(s.get("heading_deg", 0))
        self.start = (self.x, self.y, self.th)
        self.vx = self.vy = self.w = 0.0  # field-frame velocity, rad/s
        self.cmd_v = (0.0, 0.0)  # desired field-frame velocity
        self.cmd_w = 0.0
        self.objects: list[Obj] = []
        for o in lay["objects"]:
            k = game.object_kind(o["kind"])
            self.objects.append(Obj(o["id"], o["kind"], k.get("shape", "barrel"), k.get("radius_m", 0.022), o["x"], o["y"]))
        self.held: Optional[Obj] = None
        self.t = 0.0
        self.busy_until = 0.0  # placing / kicking: drive commands wait
        self.poses: list[tuple] = []
        self.object_track: list[list] = []  # per recorded pose: [[x, y, held], ...] in self.objects order
        self.events: list[tuple] = []  # (t, type, object_id, detail)
        self._next_rec = 0.0
        self.odo = np.array([0.0, 0.0])  # what the encoders think, in the start frame (x right, y forward)
        self._last_place_score_t: Optional[float] = None
        self.bumps = 0
        self._bumping = False
        self.record()

    # -- geometry helpers -------------------------------------------------------------------------
    def front(self, extra: float = 0.0) -> tuple[float, float]:
        return self.x + math.cos(self.th) * (ROBOT_R + extra), self.y + math.sin(self.th) * (ROBOT_R + extra)

    def to_robot(self, px: float, py: float) -> tuple[float, float]:
        """Field point -> (distance, bearing clockwise from the robot's heading, radians)."""
        dx, dy = px - self.x, py - self.y
        return math.hypot(dx, dy), _wrap(self.th - math.atan2(dy, dx))

    def record(self):
        if self.t + 1e-9 >= self._next_rec:
            self.poses.append((round(self.t, 3), round(self.x, 4), round(self.y, 4), round(_wrap(self.th), 4)))
            self.object_track.append([[round(o.x, 4), round(o.y, 4), int(o.held)] for o in self.objects])
            self._next_rec += 0.05

    def event(self, kind: str, oid: str = "", detail: str = ""):
        self.events.append((round(self.t, 3), kind, oid, detail))

    # -- actions ------------------------------------------------------------------------------------
    def grab_check(self):
        if self.held is not None:
            return
        for o in self.objects:
            if o.held:
                continue
            d, b = self.to_robot(o.x, o.y)
            if o.released:
                if d > ROBOT_R + o.r + 0.04:
                    o.released = False
                continue
            if d <= ROBOT_R + o.r + CAPTURE_GAP and abs(b) <= CAPTURE_BEARING and math.hypot(o.vx, o.vy) < 0.4:
                o.held, o.vx, o.vy = True, 0.0, 0.0
                self.held = o
                self.event("pickup", o.id)
                return

    def place(self):
        if self.held is None:
            self.busy_until = self.t + 0.4
            return
        o = self.held
        fx, fy = self.front(o.r + 0.012)
        o.x, o.y, o.held, o.released = fx, fy, False, True
        o.vx = o.vy = 0.0
        self.held = None
        self.busy_until = self.t + 0.6
        self.event("place", o.id, ",".join(self.game.zone_of(o.x, o.y)))

    def kick(self, force: str = "MEDIUM"):
        self.busy_until = self.t + 0.35
        if self.held is None:
            self.event("kick", "", "nothing held")
            return
        o = self.held
        a = self.th + math.radians(self.rng.normal(0, self.noise.kick_sd_deg))
        sp = KICK_SPEED.get(force.upper(), 0.7) * (1 + self.rng.normal(0, 0.06))
        fx, fy = self.front(o.r + 0.004)
        o.x, o.y, o.held, o.released = fx, fy, False, True
        o.vx, o.vy = sp * math.cos(a) + self.vx, sp * math.sin(a) + self.vy
        self.held = None
        self.event("kick", o.id, force.upper())

    # -- simulation step --------------------------------------------------------------------------------
    def step(self, dt: float = DT):
        busy = self.t < self.busy_until
        tvx, tvy = (0.0, 0.0) if busy else self.cmd_v
        tw = 0.0 if busy else self.cmd_w
        s = self.noise.speed_scale
        tvx, tvy, tw = tvx * s, tvy * s, tw * s
        # acceleration limits
        dvx, dvy = tvx - self.vx, tvy - self.vy
        dv = math.hypot(dvx, dvy)
        if dv > ACCEL * dt:
            dvx, dvy = dvx / dv * ACCEL * dt, dvy / dv * ACCEL * dt
        self.vx += dvx
        self.vy += dvy
        self.w += float(np.clip(tw - self.w, -W_ACCEL * dt, W_ACCEL * dt))
        # encoders see the commanded motion; the floor gives slip and drift
        step_x, step_y = self.vx * dt, self.vy * dt
        c, sn = math.cos(self.th), math.sin(self.th)
        st = self.start[2]
        # odometry in the start frame: y = start heading, x = its right
        ex, ey = math.sin(st), -math.cos(st)  # start-frame x axis in field coords
        gx, gy = math.cos(st), math.sin(st)
        self.odo += np.array([step_x * ex + step_y * ey, step_x * gx + step_y * gy])
        k = self.noise.dist_scale
        d = math.hypot(step_x, step_y)
        lat = self.noise.drift * d
        self.x += step_x * k + (-sn) * lat
        self.y += step_y * k + c * lat
        self.th = _wrap(self.th + self.w * dt)
        self._collide()
        self._objects(dt)
        if self.held is not None:
            fx, fy = self.front(self.held.r + 0.004)
            self.held.x, self.held.y = fx, fy
        else:
            self.grab_check()
        self.t += dt
        self.record()

    def _collide(self):
        hit = False
        m = ROBOT_R
        if self.x < m or self.x > self.W - m or self.y < m or self.y > self.H - m:
            hit = True
            self.x, self.y = min(max(self.x, m), self.W - m), min(max(self.y, m), self.H - m)
        for o in self.obstacles:
            cx, cy = min(max(self.x, o["x"]), o["x"] + o["w"]), min(max(self.y, o["y"]), o["y"] + o["h"])
            dx, dy = self.x - cx, self.y - cy
            d = math.hypot(dx, dy)
            if d < m:
                hit = True
                if d < 1e-9:
                    dx, dy, d = 0.0, 1.0, 1.0
                self.x += dx / d * (m - d)
                self.y += dy / d * (m - d)
        if hit:
            self.vx *= 0.3
            self.vy *= 0.3
            if not self._bumping:
                self.bumps += 1
                self.event("bump")
        self._bumping = hit

    def _objects(self, dt: float):
        for o in self.objects:
            if o.held:
                continue
            # pushed by the robot body
            dx, dy = o.x - self.x, o.y - self.y
            d = math.hypot(dx, dy)
            lim = ROBOT_R + o.r
            if d < lim:
                nx, ny = (dx / d, dy / d) if d > 1e-9 else (math.cos(self.th), math.sin(self.th))
                o.x, o.y = self.x + nx * lim, self.y + ny * lim
                rel = self.vx * nx + self.vy * ny
                if rel > 0:
                    o.vx, o.vy = nx * rel * 1.1, ny * rel * 1.1
            sp = math.hypot(o.vx, o.vy)
            if sp > 0:
                dec = FRICTION.get(o.shape, 2.0) * dt
                if sp <= dec:
                    o.vx = o.vy = 0.0
                else:
                    o.vx -= o.vx / sp * dec
                    o.vy -= o.vy / sp * dec
                o.x += o.vx * dt
                o.y += o.vy * dt
                e = 0.5
                if o.x < o.r or o.x > self.W - o.r:
                    o.x = min(max(o.x, o.r), self.W - o.r)
                    o.vx = -o.vx * e
                if o.y < o.r or o.y > self.H - o.r:
                    o.y = min(max(o.y, o.r), self.H - o.r)
                    o.vy = -o.vy * e
        # objects push each other
        for i, a in enumerate(self.objects):
            for b in self.objects[i + 1:]:
                if a.held or b.held:
                    continue
                dx, dy = b.x - a.x, b.y - a.y
                d = math.hypot(dx, dy)
                lim = a.r + b.r
                if 1e-9 < d < lim:
                    nx, ny = dx / d, dy / d
                    over = (lim - d) / 2
                    a.x -= nx * over
                    a.y -= ny * over
                    b.x += nx * over
                    b.y += ny * over

    # -- end of match --------------------------------------------------------------------------------------
    def settle(self, max_s: float = 3.0):
        """Stop the robot and let anything still moving come to rest (scored at rest)."""
        self.cmd_v, self.cmd_w = (0.0, 0.0), 0.0
        t_end = self.t + max_s
        while self.t < t_end and (any(math.hypot(o.vx, o.vy) > 1e-3 for o in self.objects) or math.hypot(self.vx, self.vy) > 1e-3):
            self.step()

    def touching_robot(self, o: Obj) -> bool:
        return o.held or math.hypot(o.x - self.x, o.y - self.y) <= ROBOT_R + o.r + 0.003

    def final_objects(self) -> list[dict]:
        return [{"id": o.id, "kind": o.kind, "x": round(o.x, 4), "y": round(o.y, 4), "held": self.touching_robot(o)} for o in self.objects]
