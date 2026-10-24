"""Simulate a four-robot match (2 v 2) with ground truth for every robot and block.

The simulation is deliberately simple, but it produces the situations a match
tracker has to survive: robots of the same alliance that look alike, robots
pushing each other, robots stopping beside tall structures, blocks moving,
picked up and carried, a pause between the autonomous and driver periods, and
robots parking side by side at the end.

    auton   0 .. 15 s     robots run their own code and stay on their half
    pause  15 .. 18 s     everything stops while the referees score
    driver 18 .. 123 s    people drive (more noise, defence, loaders, parking)

Everything is seeded: the same seed gives the same match.
"""

from __future__ import annotations

import heapq
import zlib
import math
from dataclasses import asdict, dataclass, field
from typing import Optional

import numpy as np

from .field import Field, default_field, initial_blocks, zone_of

DT = 0.02
RECORD_DT = 0.05
PERIODS = [("auton", 0.0, 15.0), ("pause", 15.0, 18.0), ("driver", 18.0, 123.0)]
ROBOT_R = 0.235  # collision radius (an 18 in square robot, a little rounded)
PLAN_R = 0.25  # clearance used by the path planner
CHASSIS = {"black": (40, 40, 44), "silver": (150, 152, 156), "purple": (110, 70, 140), "white": (215, 215, 210),
           "red-metal": (60, 50, 150), "grey": (95, 98, 104)}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Profile:
    v_max: float
    accel: float
    turn: float
    capacity: int
    intake_s: float
    aggression: float  # chance of switching to defence when choosing a new job (driver period)
    loader_pref: float
    parks: bool
    noise: float  # driver wobble
    hesitate: float  # chance per second of a short hesitation (driver period)


@dataclass
class RobotSpec:
    id: str
    alliance: str
    team: str
    size_m: tuple[float, float, float]  # length, width, height
    chassis: str
    accent: str
    profile: Profile


@dataclass
class Robot:
    spec: RobotSpec
    x: float
    y: float
    th: float
    v: float = 0.0
    w: float = 0.0
    carrying: list[str] = field(default_factory=list)
    job: str = "idle"
    job_until: float = 0.0
    target: Optional[tuple[float, float]] = None
    target_heading: Optional[float] = None
    target_obj: Optional[str] = None
    path: list[tuple[float, float]] = field(default_factory=list)
    plan_for: Optional[tuple[float, float]] = None
    dwell_until: float = 0.0
    hesitate_until: float = 0.0
    goal: Optional[str] = None
    in_contact: set = field(default_factory=set)
    wall_contact: bool = False
    stuck_t: float = 0.0
    last_xy: tuple[float, float] = (0.0, 0.0)


# ----------------------------------------------------------------------------- planning
class Planner:
    """A* on a 5 cm grid over the field, inflated by the robot clearance."""

    def __init__(self, fld: Field, cell: float = 0.05):
        self.fld, self.cell = fld, cell
        self.n = int(math.ceil(fld.size_m / cell))
        self.base = self._grid(None)
        self.halves = {a: self._grid(a) for a in ("red", "blue")}

    def _grid(self, half: Optional[str]) -> np.ndarray:
        n, c = self.n, self.cell
        xs = (np.arange(n) + 0.5) * c
        X, Y = np.meshgrid(xs, xs, indexing="ij")
        F = self.fld.size_m
        free = (X > PLAN_R) & (X < F - PLAN_R) & (Y > PLAN_R) & (Y < F - PLAN_R)
        for r in self.fld.obstacles:
            dx = np.maximum(np.maximum(r.x - X, X - (r.x + r.w)), 0)
            dy = np.maximum(np.maximum(r.y - Y, Y - (r.y + r.h)), 0)
            free &= np.hypot(dx, dy) > PLAN_R
        if half == "red":
            free &= X < F / 2 - 0.05
        elif half == "blue":
            free &= X > F / 2 + 0.05
        return free

    def _ij(self, p):
        return (min(max(int(p[0] / self.cell), 0), self.n - 1), min(max(int(p[1] / self.cell), 0), self.n - 1))

    def nearest_free(self, grid, ij):
        if grid[ij]:
            return ij
        idx = np.argwhere(grid)
        k = np.argmin(((idx - np.array(ij)) ** 2).sum(1))
        return tuple(idx[k])

    def line_free(self, grid, a, b) -> bool:
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        for s in np.linspace(0, 1, max(2, int(d / (self.cell * 0.5)))):
            if not grid[self._ij((a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s))]:
                return False
        return True

    def plan(self, a, b, half: Optional[str] = None) -> list[tuple[float, float]]:
        grid = self.halves[half] if half else self.base
        s = self.nearest_free(grid, self._ij(a))
        g = self.nearest_free(grid, self._ij(b))
        if s == g:
            return [b]
        n = self.n
        openq = [(0.0, s)]
        came = {s: None}
        cost = {s: 0.0}
        nbr = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for di, dj, w in nbr:
                nx, ny = cur[0] + di, cur[1] + dj
                if 0 <= nx < n and 0 <= ny < n and grid[nx, ny]:
                    nc = cost[cur] + w
                    if nc < cost.get((nx, ny), 1e18):
                        cost[(nx, ny)] = nc
                        came[(nx, ny)] = cur
                        heapq.heappush(openq, (nc + math.hypot(g[0] - nx, g[1] - ny), (nx, ny)))
        if g not in came:
            return [b]
        cells = []
        cur = g
        while cur is not None:
            cells.append(((cur[0] + 0.5) * self.cell, (cur[1] + 0.5) * self.cell))
            cur = came[cur]
        cells.reverse()
        # shortcut: keep only the points needed for straight, collision-free legs
        pts = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.line_free(grid, cells[i], cells[j]):
                j -= 1
            pts.append(cells[j])
            i = j
        pts = pts[1:]
        if grid[self._ij(b)]:
            pts[-1] = b
        else:
            pts.append(b)  # the last leg reaches into the clearance (a block by a wall)
        return pts


# ----------------------------------------------------------------------------- the match
def roster(rng: np.random.Generator) -> list[RobotSpec]:
    specs = []
    teams = rng.choice(np.arange(100, 99999), 4, replace=False)
    chassis = list(CHASSIS)
    for k, (alliance, num) in enumerate((("red", 1), ("red", 2), ("blue", 1), ("blue", 2))):
        p = Profile(
            v_max=float(rng.uniform(1.15, 1.9)), accel=float(rng.uniform(2.5, 5.0)), turn=float(rng.uniform(4.5, 8.5)),
            capacity=int(rng.integers(3, 8)), intake_s=float(rng.uniform(0.12, 0.35)), aggression=float(rng.choice([0.0, 0.1, 0.25, 0.45])),
            loader_pref=float(rng.uniform(0.0, 0.5)), parks=bool(num == 1 or rng.random() < 0.3),
            noise=float(rng.uniform(0.02, 0.10)), hesitate=float(rng.uniform(0.02, 0.15)))
        L = float(rng.uniform(0.38, 0.457))
        specs.append(RobotSpec(
            id=f"{alliance}{num}", alliance=alliance, team=f"{int(teams[k])}{rng.choice(list('ABCDXYZ'))}",
            size_m=(L, float(min(0.457, L + rng.uniform(-0.03, 0.04))), float(rng.uniform(0.28, 0.45))),
            chassis=str(rng.choice(chassis)), accent=str(rng.choice(["none", "purple", "orange", "green", "white"])), profile=p))
    return specs


class Match:
    def __init__(self, seed: int, fld: Optional[Field] = None):
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.fld = fld or default_field()
        self.planner = Planner(self.fld)
        self.specs = roster(self.rng)
        F = self.fld.size_m
        starts = {"red1": (0.32, 0.95, 0.0), "red2": (0.32, F - 0.95, 0.0), "blue1": (F - 0.32, F - 0.95, math.pi), "blue2": (F - 0.32, 0.95, math.pi)}
        if self.rng.random() < 0.5:  # mirror the near/far starts sometimes
            starts["red1"], starts["red2"] = starts["red2"], starts["red1"]
        self.robots = []
        for s in self.specs:
            x, y, th = starts[s.id]
            self.robots.append(Robot(s, x + self.rng.normal(0, 0.03), y + self.rng.normal(0, 0.05), th + self.rng.normal(0, 0.05)))
        self.blocks = {b["id"]: b for b in initial_blocks(self.rng, self.fld)}
        self.goal_q: dict[str, list[str]] = {}  # physical goal -> block ids (long goals: ordered left to right)
        self.t = 0.0
        self.events: list[tuple] = []  # (t, robot, type, other, detail)
        self.obj_log: list[tuple] = []  # (t, id, state, x, y, z, ref)
        self.poses: dict[str, list[tuple]] = {r.spec.id: [] for r in self.robots}
        self.claimed: dict[str, str] = {}
        self.parked: set = set()
        for b in self.blocks.values():
            self._log_obj(b)

    # -- helpers ------------------------------------------------------------------------------
    def period(self, t: Optional[float] = None) -> str:
        t = self.t if t is None else t
        for name, a, b in PERIODS:
            if a <= t < b:
                return name
        return "end"

    def _log_obj(self, b):
        self.obj_log.append((round(self.t, 3), b["id"], b["state"], round(b["x"], 3), round(b["y"], 3), round(b["z"], 3), b["ref"]))
        b["_logged"] = (b["x"], b["y"])

    def _event(self, r: Robot, kind: str, other: str = "", detail: str = ""):
        self.events.append((round(self.t, 3), r.spec.id, kind, other, detail))

    def _physical(self, gname: str) -> str:
        return "centre" if gname.startswith("centre") else gname.rsplit("-", 1)[0]

    def _goal_room(self, gname: str) -> int:
        if gname.startswith("long"):
            return 99  # a full long goal pushes blocks out of its far end
        g = self.fld.goals[gname]
        return len(g.slots) - len(self.goal_q.setdefault(self._physical(gname), []))

    # -- choosing jobs ------------------------------------------------------------------------
    def _own_half(self, r: Robot, x: float) -> bool:
        F = self.fld.size_m
        return x < F / 2 if r.spec.alliance == "red" else x > F / 2

    def _choose_job(self, r: Robot):
        p = r.spec.profile
        per = self.period()
        rng = self.rng
        r.path, r.plan_for, r.target_obj, r.goal, r.target_heading = [], None, None, None, None
        for k, v in list(self.claimed.items()):
            if v == r.spec.id:
                del self.claimed[k]
        F = self.fld.size_m
        # end game: one robot per alliance parks
        if per == "driver" and self.t > 111 + rng.uniform(-2, 2) and p.parks and not any(
                o.job == "park" for o in self.robots if o.spec.alliance == r.spec.alliance and o is not r):
            z = self.fld.zones[f"park-{r.spec.alliance}"]
            r.job, r.target = "park", (z.center[0] + (0.02 if r.spec.alliance == "red" else -0.02), z.center[1] + rng.normal(0, 0.05))
            r.target_heading = math.pi / 2 if rng.random() < 0.5 else -math.pi / 2
            return
        if r.carrying and (len(r.carrying) >= p.capacity or rng.random() < 0.25 or per == "auton" and len(r.carrying) >= 2):
            options = [g for g in self.fld.goals if self._goal_room(g) >= len(r.carrying)]
            if per == "auton":
                options = [g for g in options if self._own_half(r, self.fld.goals[g].approach[0])]
            if options:
                def gcost(g):
                    ax, ay = self.fld.goals[g].approach
                    crowd = sum(1 for o in self.robots if o is not r and math.hypot(o.x - ax, o.y - ay) < 0.6)
                    return math.hypot(ax - r.x, ay - r.y) + 1.2 * crowd + rng.uniform(0, 0.6)
                g = min(options, key=gcost)
                r.job, r.goal = "score", g
                r.target, r.target_heading = self.fld.goals[g].approach, self.fld.goals[g].heading
                return
        if per == "driver" and rng.random() < p.aggression:
            opp = [o for o in self.robots if o.spec.alliance != r.spec.alliance]
            o = min(opp, key=lambda o: math.hypot(o.x - r.x, o.y - r.y) + rng.uniform(0, 1))
            r.job, r.target_obj, r.job_until = "defend", o.spec.id, self.t + rng.uniform(2.5, 7)
            r.target = (o.x, o.y)
            return
        if per == "driver" and rng.random() < p.loader_pref and len(r.carrying) < p.capacity:
            loaders = [k for k in self.fld.loaders if k.startswith(r.spec.alliance)
                       and any(b["state"] == "loader" and b["ref"] == k for b in self.blocks.values())]
            if loaders:
                k = min(loaders, key=lambda k: math.hypot(self.fld.loaders[k][0] - r.x, self.fld.loaders[k][1] - r.y))
                lx, ly = self.fld.loaders[k]
                r.job, r.goal = "load", k
                r.target = (lx + (0.30 if lx < F / 2 else -0.30), ly)
                r.target_heading = math.pi if lx < F / 2 else 0.0
                return
        # pick up a block of our colour (others' blocks too, sometimes, to deny them)
        cands = []
        for b in self.blocks.values():
            if b["state"] != "floor" or b["id"] in self.claimed:
                continue
            if per == "auton" and not self._own_half(r, b["x"]):
                continue
            if any(z.startswith("park") for z in zone_of(self.fld, b["x"], b["y"])) and per == "auton":
                continue
            w = 1.0 if b["colour"] == r.spec.alliance else 2.5
            cands.append((math.hypot(b["x"] - r.x, b["y"] - r.y) * w + rng.uniform(0, 0.4), b["id"]))
        if cands:
            _, bid = min(cands)
            self.claimed[bid] = r.spec.id
            r.job, r.target_obj = "collect", bid
            r.target = (self.blocks[bid]["x"], self.blocks[bid]["y"])
            return
        if r.carrying:
            r.job = "idle"
            r.job_until = self.t
            return
        r.job, r.job_until = "wander", self.t + rng.uniform(1, 3)
        half = r.spec.alliance if per == "auton" else None
        grid = self.planner.halves[half] if half else self.planner.base
        idx = np.argwhere(grid)
        i, j = idx[rng.integers(len(idx))]
        r.target = ((i + 0.5) * self.planner.cell, (j + 0.5) * self.planner.cell)

    # -- control --------------------------------------------------------------------------------
    def _steer(self, r: Robot, dt: float):
        p = r.spec.profile
        per = self.period()
        if per in ("pause", "end") or self.t < r.dwell_until or self.t < r.hesitate_until:
            self._drive(r, 0.0, 0.0, dt)
            return
        if per == "driver" and self.rng.random() < p.hesitate * dt:
            r.hesitate_until = self.t + self.rng.uniform(0.2, 0.9)
        if r.target is None:
            self._drive(r, 0.0, 0.0, dt)
            return
        if r.job == "defend":
            o = next(o for o in self.robots if o.spec.id == r.target_obj)
            r.target = (o.x, o.y)
        if r.job == "collect" and r.target_obj:
            b = self.blocks[r.target_obj]
            r.target = (b["x"], b["y"])
        half = r.spec.alliance if per == "auton" else None
        # (re)plan when the target moved or the path ran out
        if r.plan_for is None or math.hypot(r.plan_for[0] - r.target[0], r.plan_for[1] - r.target[1]) > 0.15 or not r.path:
            r.path = self.planner.plan((r.x, r.y), r.target, half) if r.job != "defend" else [r.target]
            r.plan_for = r.target
        while len(r.path) > 1 and math.hypot(r.path[0][0] - r.x, r.path[0][1] - r.y) < 0.12:
            r.path.pop(0)
        wx, wy = r.path[0]
        dist = math.hypot(wx - r.x, wy - r.y)
        final = len(r.path) == 1
        if final and dist < 0.04 and r.target_heading is not None:
            err = _wrap(r.target_heading - r.th)
            self._drive(r, 0.0, float(np.clip(4 * err, -p.turn, p.turn)), dt)
            return
        want = math.atan2(wy - r.y, wx - r.x)
        noise = p.noise * (2.5 if per == "driver" else 0.6)
        want += noise * math.sin(self.t * 1.7 + zlib.crc32(r.spec.id.encode()) % 7)
        err = _wrap(want - r.th)
        reverse = False
        if final and dist < 0.6 and abs(err) > 2.3 and r.job not in ("collect",):
            reverse, err = True, _wrap(err + math.pi)
        w = float(np.clip(5 * err, -p.turn, p.turn))
        v = p.v_max * max(0.0, math.cos(err)) ** 2
        if final and r.job != "defend":
            v = min(v, math.sqrt(2 * p.accel * 0.6 * max(dist - 0.02, 0)) + 0.05)
        # give way to other robots (unless pushing on purpose)
        if r.job != "defend":
            for o in self.robots:
                if o is r:
                    continue
                dx, dy = o.x - r.x, o.y - r.y
                d = math.hypot(dx, dy)
                ahead = (dx * math.cos(r.th) + dy * math.sin(r.th)) / max(d, 1e-6)
                if d < 0.75 and ahead > 0.5 and not reverse:
                    v *= max(0.25, (d - 0.45) / 0.3)
        self._drive(r, -v if reverse else v, w, dt)

    def _drive(self, r: Robot, v_cmd: float, w_cmd: float, dt: float):
        p = r.spec.profile
        dv = float(np.clip(v_cmd - r.v, -p.accel * 1.5 * dt, p.accel * dt))
        r.v += dv
        r.w += float(np.clip(w_cmd - r.w, -p.turn * 6 * dt, p.turn * 6 * dt))
        r.th = _wrap(r.th + r.w * dt)
        r.x += r.v * math.cos(r.th) * dt
        r.y += r.v * math.sin(r.th) * dt

    # -- world interaction ----------------------------------------------------------------------
    def _collide(self):
        F = self.fld.size_m
        rs = self.robots
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                a, b = rs[i], rs[j]
                dx, dy = b.x - a.x, b.y - a.y
                d = math.hypot(dx, dy)
                touching = d < 2 * ROBOT_R
                if touching:
                    nx, ny = (dx / d, dy / d) if d > 1e-6 else (1.0, 0.0)
                    over = 2 * ROBOT_R - d
                    pa = 1 + max(0.0, a.v * (math.cos(a.th) * nx + math.sin(a.th) * ny)) / a.spec.profile.v_max
                    pb = 1 + max(0.0, -b.v * (math.cos(b.th) * nx + math.sin(b.th) * ny)) / b.spec.profile.v_max
                    a.x -= nx * over * pb / (pa + pb)
                    a.y -= ny * over * pb / (pa + pb)
                    b.x += nx * over * pa / (pa + pb)
                    b.y += ny * over * pa / (pa + pb)
                    a.v *= 0.92
                    b.v *= 0.92
                for x, y in ((a, b), (b, a)):
                    if touching and y.spec.id not in x.in_contact:
                        x.in_contact.add(y.spec.id)
                        self._event(x, "contact_robot", y.spec.id)
                    elif not touching and d > 2 * ROBOT_R + 0.03:
                        x.in_contact.discard(y.spec.id)
        for r in rs:
            hit = False
            m = ROBOT_R - 0.03
            if r.x < m or r.x > F - m or r.y < m or r.y > F - m:
                hit = True
                r.x, r.y = min(max(r.x, m), F - m), min(max(r.y, m), F - m)
            for o in self.fld.obstacles:
                cx, cy = min(max(r.x, o.x), o.x + o.w), min(max(r.y, o.y), o.y + o.h)
                dx, dy = r.x - cx, r.y - cy
                d = math.hypot(dx, dy)
                if d < m:
                    hit = True
                    if d < 1e-6:
                        dx, dy, d = 0.0, (1.0 if r.y > o.center[1] else -1.0), 1.0
                    r.x += dx / d * (m - d)
                    r.y += dy / d * (m - d)
            if self.period() == "auton":  # robots keep to their own half in autonomous
                r.x = min(r.x, F / 2 - 0.05) if r.spec.alliance == "red" else max(r.x, F / 2 + 0.05)
            if hit and not r.wall_contact and self.period() != "pause":
                self._event(r, "contact_wall")
            r.wall_contact = hit

    def _blocks(self):
        F = self.fld.size_m
        for r in self.robots:
            hx, hy = math.cos(r.th), math.sin(r.th)
            for b in self.blocks.values():
                if b["state"] != "floor":
                    continue
                dx, dy = b["x"] - r.x, b["y"] - r.y
                d = math.hypot(dx, dy)
                if d > 0.4:
                    continue
                front = dx * hx + dy * hy
                side = abs(-dx * hy + dy * hx)
                if (r.job == "collect" and len(r.carrying) < r.spec.profile.capacity and 0.12 < front < 0.30 and side < 0.13
                        and self.period() in ("auton", "driver")):
                    b["state"], b["ref"] = "carried", r.spec.id
                    r.carrying.append(b["id"])
                    self.claimed.pop(b["id"], None)
                    r.dwell_until = self.t + r.spec.profile.intake_s
                    self._log_obj(b)
                    self._event(r, "pickup", b["id"])
                    if r.target_obj == b["id"]:
                        r.job, r.job_until = "idle", self.t
                    continue
                # pushed out of the way (a robot is a ~0.23 m half-width square)
                lim = 0.26
                if d < lim:
                    nx, ny = (dx / d, dy / d) if d > 1e-6 else (hx, hy)
                    b["x"] = min(max(r.x + nx * lim, 0.04), F - 0.04)
                    b["y"] = min(max(r.y + ny * lim, 0.04), F - 0.04)
                    lx, ly = b.get("_logged", (b["x"], b["y"]))
                    if math.hypot(b["x"] - lx, b["y"] - ly) > 0.02:
                        self._log_obj(b)

    def _arrive(self, r: Robot):
        if r.target is None or self.period() not in ("auton", "driver") or self.t < r.dwell_until:
            return
        d = math.hypot(r.target[0] - r.x, r.target[1] - r.y)
        aligned = r.target_heading is None or abs(_wrap(r.target_heading - r.th)) < 0.2
        if r.job == "score" and d < 0.08 and aligned:
            g = self.fld.goals[r.goal]
            phys = self._physical(r.goal)
            q = self.goal_q.setdefault(phys, [])
            from_right = r.goal.endswith("right")
            n = 0
            for bid in list(r.carrying):
                if not phys.startswith("long") and len(q) >= len(g.slots):
                    break
                b = self.blocks[bid]
                r.carrying.remove(bid)
                n += 1
                self._event(r, "score", bid, r.goal)
                if phys.startswith("long"):
                    q.append(bid) if from_right else q.insert(0, bid)
                    slots = self.fld.goals[f"{phys}-left"].slots
                    if len(q) > len(slots):  # pushed through: the block at the far end drops out
                        out = q.pop(0) if from_right else q.pop()
                        ob = self.blocks[out]
                        ex = slots[0][0] - 0.12 if from_right else slots[-1][0] + 0.12
                        ob.update(state="floor", ref="", x=ex + self.rng.normal(0, 0.03), y=slots[0][1] + self.rng.normal(0, 0.08), z=0.0)
                        self._log_obj(ob)
                        self.events.append((round(self.t, 3), r.spec.id, "push_out", out, phys))
                    # slots fill from whichever end; keep blocks packed against the entry end
                    start = len(slots) - len(q) if from_right else 0
                    for k, qid in enumerate(q):
                        sx, sy, sz = slots[start + k]
                        qb = self.blocks[qid]
                        if qb["state"] != "scored" or abs(qb["x"] - sx) > 1e-6:
                            qb.update(state="scored", ref=phys, x=sx, y=sy, z=sz)
                            self._log_obj(qb)
                else:
                    sx, sy, sz = g.slots[len(q)]
                    q.append(bid)
                    b.update(state="scored", ref=phys, x=sx, y=sy, z=sz)
                    self._log_obj(b)
            r.dwell_until = self.t + 0.3 + 0.25 * n
            r.job, r.job_until = "idle", self.t + 0.2
        elif r.job == "load" and d < 0.08 and aligned:
            got = 0
            for b in sorted((b for b in self.blocks.values() if b["state"] == "loader" and b["ref"] == r.goal), key=lambda b: b["z"]):
                if len(r.carrying) >= r.spec.profile.capacity:
                    break
                b.update(state="carried", ref=r.spec.id)
                r.carrying.append(b["id"])
                self._log_obj(b)
                self._event(r, "load", b["id"], r.goal)
                got += 1
            r.dwell_until = self.t + 0.6 + 0.3 * got
            r.job, r.job_until = "idle", self.t
        elif r.job == "park" and d < 0.1:
            if r.spec.id not in self.parked:
                self.parked.add(r.spec.id)
                self._event(r, "park", "", f"park-{r.spec.alliance}")
            r.target = None
        elif r.job == "wander" and d < 0.15:
            r.job_until = self.t

    def _people_reload(self):
        """Drivers' teammates drop blocks back into the loaders now and then (driver period)."""
        if self.period() != "driver" or self.rng.random() > 0.15 * DT:
            return
        spare = [b for b in self.blocks.values() if b["state"] == "floor"
                 and min(b["x"], self.fld.size_m - b["x"]) < 0.12]
        if not spare:
            return
        b = spare[self.rng.integers(len(spare))]
        name = min(self.fld.loaders, key=lambda k: math.hypot(self.fld.loaders[k][0] - b["x"], self.fld.loaders[k][1] - b["y"]))
        n = sum(1 for o in self.blocks.values() if o["state"] == "loader" and o["ref"] == name)
        if n < 8:
            lx, ly = self.fld.loaders[name]
            b.update(state="loader", ref=name, x=lx, y=ly, z=0.05 + 0.09 * n)
            self._log_obj(b)
            self.events.append((round(self.t, 3), "", "reload", b["id"], name))

    # -- main loop --------------------------------------------------------------------------------
    def run(self) -> "MatchResult":
        end = PERIODS[-1][2]
        next_rec = 0.0
        steps = int(round(end / DT))
        for k in range(steps + 1):
            self.t = k * DT
            per = self.period()
            for r in self.robots:
                if per in ("auton", "driver") and (r.job in ("idle",) and self.t >= r.job_until
                                                   or r.job in ("defend", "wander") and self.t >= r.job_until
                                                   or r.job == "collect" and (r.target_obj not in self.claimed or self.blocks[r.target_obj]["state"] != "floor")
                                                   or r.job != "park" and per == "driver" and self.t > 109 and r.spec.profile.parks and self.rng.random() < 0.02):
                    self._choose_job(r)
                if per == "driver" and self.t - PERIODS[2][1] < DT:  # new period: rethink
                    self._choose_job(r)
                self._steer(r, DT)
                # stuck detection: pick something else
                if math.hypot(r.x - r.last_xy[0], r.y - r.last_xy[1]) < 0.002 and abs(r.v) > 0.3:
                    r.stuck_t += DT
                else:
                    r.stuck_t = 0.0
                r.last_xy = (r.x, r.y)
                if r.stuck_t > 1.5 and r.job != "park":
                    r.stuck_t = 0.0
                    self._choose_job(r)
            self._collide()
            self._blocks()
            for r in self.robots:
                self._arrive(r)
            for r in self.robots:  # carried blocks travel with their robot
                for bid in r.carrying:
                    self.blocks[bid].update(x=r.x, y=r.y)
            self._people_reload()
            if self.t >= next_rec - 1e-9:
                for r in self.robots:
                    self.poses[r.spec.id].append((round(self.t, 3), round(r.x, 4), round(r.y, 4), round(r.th, 4)))
                next_rec += RECORD_DT
        return MatchResult(self)


class MatchResult:
    def __init__(self, m: Match):
        self.seed = m.seed
        self.field = m.fld
        self.specs = m.specs
        self.poses = {k: np.array(v) for k, v in m.poses.items()}
        self.events = m.events
        self.obj_log = m.obj_log
        self.blocks = {k: {"id": k, "colour": b["colour"]} for k, b in m.blocks.items()}
        self.final = {k: {kk: b[kk] for kk in ("state", "x", "y", "z", "ref")} for k, b in m.blocks.items()}
        self.parked = sorted(m.parked)

    def robot_meta(self, s: RobotSpec) -> dict:
        d = asdict(s)
        d["profile"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d["profile"].items()}
        d["size_m"] = [round(v, 3) for v in s.size_m]
        return d


def simulate(seed: int) -> MatchResult:
    return Match(seed).run()
