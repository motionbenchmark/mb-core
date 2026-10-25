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

Games can also stand AprilTag signs on the field (``shape: "tag"``): a thin plate
on a small base that falls over when something hits it. What knocked it down is
recorded (a rolling ball, the robot itself, a carried ball, another object), so a
game's rules decide what counts. A ball or barrel held on the front magnet is
knocked loose when the robot turns while it is pressed against a wall.

Assumptions, not AIM specifications: the robot's footprint (0.09 m radius unless
the game sets ``robot.radius_m``), accelerations, kick speeds, friction, how hard
a hit must be to knock a sign down, and when a held object comes loose.
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
KNOCK_SPEED = 0.12  # m/s: a free object at least this fast knocks a standing sign over
DISLODGE_W = math.radians(25)  # turning faster than this with the held object against a wall knocks it loose


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
    capture_gap: float = CAPTURE_GAP
    capture_bearing: float = CAPTURE_BEARING
    tag_id: int = -1  # AprilTag signs: the tag's id
    facing: float = 0.0  # AprilTag signs: field heading (deg, clockwise from +y) the printed face points to
    down: bool = False  # AprilTag signs: knocked over
    knocked_by: str = ""
    wall_bounced: bool = False  # since it last left the robot


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
        self.R = float(game.raw.get("robot", {}).get("radius_m", ROBOT_R))
        self.rules = game.raw.get("scoring", {}).get("rule_flags", [])
        self._contacts: set = set()
        for o in lay["objects"]:
            k = game.object_kind(o["kind"])
            ob = Obj(o["id"], o["kind"], k.get("shape", "barrel"), k.get("radius_m", 0.022), o["x"], o["y"])
            ob.capture_gap = float(k.get("capture_gap_m", CAPTURE_GAP))
            ob.capture_bearing = math.radians(float(k.get("capture_bearing_deg", math.degrees(CAPTURE_BEARING))))
            if ob.shape == "tag":
                ob.tag_id, ob.facing = int(o.get("tag_id", -1)), float(o.get("facing_deg", 0.0))
            self.objects.append(ob)
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
        return self.x + math.cos(self.th) * (self.R + extra), self.y + math.sin(self.th) * (self.R + extra)

    def to_robot(self, px: float, py: float) -> tuple[float, float]:
        """Field point -> (distance, bearing clockwise from the robot's heading, radians)."""
        dx, dy = px - self.x, py - self.y
        return math.hypot(dx, dy), _wrap(self.th - math.atan2(dy, dx))

    def record(self):
        if self.t + 1e-9 >= self._next_rec:
            self.poses.append((round(self.t, 3), round(self.x, 4), round(self.y, 4), round(_wrap(self.th), 4)))
            self.object_track.append([[round(o.x, 4), round(o.y, 4), int(o.held), int(o.down)] for o in self.objects])
            self._next_rec += 0.05

    def event(self, kind: str, oid: str = "", detail: str = ""):
        self.events.append((round(self.t, 3), kind, oid, detail))

    # -- actions ------------------------------------------------------------------------------------
    def grab_check(self):
        if self.held is not None:
            return
        for o in self.objects:
            if o.held or o.shape == "tag":
                continue
            d, b = self.to_robot(o.x, o.y)
            if o.released:
                if d > self.R + o.r + 0.04:
                    o.released = False
                continue
            if d <= self.R + o.r + o.capture_gap and abs(b) <= o.capture_bearing and math.hypot(o.vx, o.vy) < 0.4:
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
        o.wall_bounced = False
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
        o.wall_bounced = False
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
            o = self.held
            fx, fy = self.front(o.r + 0.004)
            o.x, o.y = fx, fy
            against = fx < o.r + 0.003 or fx > self.W - o.r - 0.003 or fy < o.r + 0.003 or fy > self.H - o.r - 0.003
            if against and abs(self.w) > DISLODGE_W:
                # pressed against the wall while turning: the magnet loses it
                o.x, o.y = min(max(fx, o.r), self.W - o.r), min(max(fy, o.r), self.H - o.r)
                o.held, o.released, o.wall_bounced = False, True, True
                tx, ty = -math.sin(self.th) * self.w * (self.R + o.r), math.cos(self.th) * self.w * (self.R + o.r)
                o.vx, o.vy = tx * 0.8, ty * 0.8
                self.held = None
                self.event("dislodge", o.id, "turned against the wall")
        else:
            self.grab_check()
        self.t += dt
        self.record()

    def _collide(self):
        hit = False
        m = self.R
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

    def _rule(self, key: str, oid: str, detail: str):
        """Record a game-rule violation once per contact (the contact must end before it counts again)."""
        if (key, oid) in self._contacts:
            self._touching.add((key, oid))
            return
        self._contacts.add((key, oid))
        self._touching.add((key, oid))
        self.event(key, oid, detail)
        if key in self.rules:
            self.event("rule_flag", oid, detail)

    def knock(self, tag: Obj, by: str):
        tag.down, tag.knocked_by = True, by
        self.event("target_down", tag.id, by)
        if by in ("rammed", "carried") and "target_rammed" in self.rules:
            self.event("rule_flag", tag.id, f"target knocked down by the robot ({by})")

    def _tags(self):
        """Standing AprilTag signs: knocked over by the robot or by a moving object."""
        robot_speed = math.hypot(self.vx, self.vy) + abs(self.w) * self.R
        for t in self.objects:
            if t.shape != "tag" or t.down:
                continue
            if math.hypot(t.x - self.x, t.y - self.y) < self.R + t.r:
                if robot_speed > 0.01:
                    self.knock(t, "rammed")
                continue
            h = self.held
            if h is not None and math.hypot(t.x - h.x, t.y - h.y) < h.r + t.r:
                if robot_speed > 0.01:
                    self.knock(t, "carried")
                continue
            for o in self.objects:
                if o is t or o.held or o.shape == "tag":
                    continue
                dx, dy = o.x - t.x, o.y - t.y
                d = math.hypot(dx, dy)
                if d < o.r + t.r:
                    sp = math.hypot(o.vx, o.vy)
                    nx, ny = (dx / d, dy / d) if d > 1e-9 else (1.0, 0.0)
                    o.x, o.y = t.x + nx * (o.r + t.r), t.y + ny * (o.r + t.r)
                    vn = o.vx * nx + o.vy * ny
                    if vn < 0:  # bounce off the sign
                        o.vx -= 1.6 * vn * nx
                        o.vy -= 1.6 * vn * ny
                    if sp >= KNOCK_SPEED:
                        self.knock(t, "ball" if o.shape == "ball" else o.kind)
                        break

    def _barrel_rules(self):
        """Contacts a game may forbid: a ball hitting a barrel before it has bounced off a wall, and the robot
        running into a barrel while carrying the ball."""
        balls = [o for o in self.objects if o.shape == "ball"]
        for b in self.objects:
            if b.shape != "barrel":
                continue
            for ball in balls:
                if not ball.held and not ball.wall_bounced and math.hypot(ball.x - b.x, ball.y - b.y) < ball.r + b.r + 0.002 \
                        and math.hypot(ball.vx, ball.vy) > 0.02:
                    self._rule("barrel_hit", b.id, "barrel hit directly by the ball")
            if self.held is not None and self.held.shape == "ball":
                h = self.held
                if math.hypot(b.x - self.x, b.y - self.y) < self.R + b.r + 0.002 or math.hypot(b.x - h.x, b.y - h.y) < h.r + b.r + 0.002:
                    self._rule("barrel_hit", b.id, "ran into the barrel while carrying the ball")

    def _objects(self, dt: float):
        self._touching: set = set()
        self._tags()
        for o in self.objects:
            if o.held or o.shape == "tag":
                continue
            # pushed by the robot body
            dx, dy = o.x - self.x, o.y - self.y
            d = math.hypot(dx, dy)
            lim = self.R + o.r
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
                    o.wall_bounced = True
                if o.y < o.r or o.y > self.H - o.r:
                    o.y = min(max(o.y, o.r), self.H - o.r)
                    o.vy = -o.vy * e
                    o.wall_bounced = True
        # objects push each other
        for i, a in enumerate(self.objects):
            for b in self.objects[i + 1:]:
                if a.held or b.held or a.shape == "tag" or b.shape == "tag":
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
                    vrel = (a.vx - b.vx) * nx + (a.vy - b.vy) * ny
                    if vrel > 0:  # pass momentum on (a rolling ball moves a barrel); equal masses, restitution 0.6
                        j = 0.8 * vrel
                        a.vx, a.vy = a.vx - j * nx, a.vy - j * ny
                        b.vx, b.vy = b.vx + j * nx, b.vy + j * ny
        if self.rules:
            self._barrel_rules()
        self._contacts &= self._touching

    # -- end of match --------------------------------------------------------------------------------------
    def settle(self, max_s: float = 3.0):
        """Stop the robot and let anything still moving come to rest (scored at rest)."""
        self.cmd_v, self.cmd_w = (0.0, 0.0), 0.0
        t_end = self.t + max_s
        while self.t < t_end and (any(math.hypot(o.vx, o.vy) > 1e-3 for o in self.objects) or math.hypot(self.vx, self.vy) > 1e-3):
            self.step()

    def touching_robot(self, o: Obj) -> bool:
        return o.held or math.hypot(o.x - self.x, o.y - self.y) <= self.R + o.r + 0.003

    def final_objects(self) -> list[dict]:
        out = []
        for o in self.objects:
            d = {"id": o.id, "kind": o.kind, "x": round(o.x, 4), "y": round(o.y, 4), "held": self.touching_robot(o)}
            if o.shape == "tag":
                d.update(tag_id=o.tag_id, down=o.down, knocked_by=o.knocked_by)
            out.append(d)
        return out
