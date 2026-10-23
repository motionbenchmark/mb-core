"""Synthetic dataset generator.

Simulates agents of every class on every built-in task so the full pipeline
(trace -> score -> leaderboard) can be exercised before any real field data
exists. The numbers are illustrative only: they encode plausible behaviours
(humans correct visually but drive with wobble; open-loop autons drift; AI
autons improve with feedback rounds) and must never be presented as findings.

Deterministic: the same `master_seed` produces byte-identical traces.

    python -m motionbench.synth --out data/synthetic-v0.1
"""

from __future__ import annotations

import argparse
import hashlib
import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Optional

import numpy as np

from . import load_task
from .trace import write_trace

DT_FIELD = 0.05  # 20 Hz for VEX tasks
DT_ROOM = 0.10  # 10 Hz for coverage tasks


class BudgetSpent(Exception):
    pass


@dataclass
class Profile:
    """Behavioural parameters for one simulated agent."""

    id: str
    cls: str
    name: str
    v_max: float = 1.0  # m/s
    accel: float = 2.0  # m/s^2
    turn_rate: float = 3.0  # rad/s
    stop_err: float = 0.03  # std of stopping position error (m)
    head_err: float = 0.03  # std of final heading error (rad)
    wobble: float = 0.0  # lateral wobble amplitude while driving (m)
    drift_per_m: float = 0.0  # odometry drift (m of error per m driven), open-loop only
    closed_loop: bool = True  # can it see/measure where it actually is?
    max_corrections: int = 20
    corr_gain: float = 0.5  # each correction scales errors by this
    reaction: tuple[float, float] = (0.05, 0.1)  # pause range between actions (s)
    hesitate_p: float = 0.0  # chance of an extra hesitation at each waypoint
    pick_time: tuple[float, float] = (0.3, 0.5)
    placement_spread: float = 0.5  # 0 = zone centre, 1 = anywhere in zone, >1 can miss
    suboptimal_order_p: float = 0.0
    track_noise: float = 0.004  # capture noise added to recorded positions (m)
    holonomic: bool = False  # omni drive: translates in any direction without turning first
    meta: dict = field(default_factory=dict)


class Sim:
    """Unicycle robot that records poses at a fixed rate and stops at the budget."""

    def __init__(self, x, y, th, budget, dt, rng, track_noise, bounds=None):
        self.x, self.y, self.th, self.t = x, y, th, 0.0
        self.bounds = bounds  # (xmin, ymin, xmax, ymax) for the robot centre: walls are solid
        self.budget, self.dt, self.rng, self.track_noise = budget, dt, rng, track_noise
        self.poses: list[tuple[float, float, float, float]] = []
        self.events: list[tuple[float, str, str, str]] = []
        self.odometer = 0.0
        self._rec()

    def _rec(self):
        n = self.track_noise
        self.poses.append(
            (
                self.t,
                self.x + self.rng.normal(0, n),
                self.y + self.rng.normal(0, n),
                _wrap(self.th + self.rng.normal(0, n * 2)),
            )
        )

    def _step(self):
        self.t += self.dt
        if self.t > self.budget + 1e-9:
            raise BudgetSpent
        self._rec()

    def event(self, kind, oid="", detail=""):
        self.events.append((self.t, kind, oid, detail))

    def pause(self, secs):
        for _ in range(max(0, int(round(secs / self.dt)))):
            self._step()

    def turn_to(self, target, rate):
        while True:
            d = _wrap(target - self.th)
            if abs(d) < 1e-3:
                self.th = _wrap(target)
                return
            step = math.copysign(min(abs(d), rate * self.dt), d)
            self.th = _wrap(self.th + step)
            self._step()

    def drive_to(self, tx, ty, v_max, accel, wobble=0.0, turn_rate=3.0, holonomic=False):
        if self.bounds:
            x0, y0, x1, y1 = self.bounds
            tx, ty = min(max(tx, x0), x1), min(max(ty, y0), y1)
        d0 = math.hypot(tx - self.x, ty - self.y)
        if d0 < 1e-3:
            return
        if d0 > 0.15 and not holonomic:
            self.turn_to(math.atan2(ty - self.y, tx - self.x), turn_rate)
        sx, sy = self.x, self.y
        ux, uy = (tx - sx) / d0, (ty - sy) / d0
        px, py = -uy, ux
        s, v = 0.0, 0.0
        phase = self.rng.uniform(0, math.pi)
        while s < d0 - 1e-6:
            remaining = d0 - s
            v = min(v + accel * self.dt, v_max, math.sqrt(max(2 * accel * remaining, 0.0)) + 0.02)
            s = min(d0, s + v * self.dt)
            lat = wobble * math.sin(math.pi * s / d0) * math.sin(phase + 6 * s)
            nx, ny = sx + ux * s + px * lat, sy + uy * s + py * lat
            if self.bounds:
                nx, ny = min(max(nx, self.bounds[0]), self.bounds[2]), min(max(ny, self.bounds[1]), self.bounds[3])
            self.odometer += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
            if holonomic:
                self.th = _wrap(self.th + 0.2 * wobble * math.cos(phase + 6 * s) * self.dt)
            elif d0 > 0.15:
                self.th = _wrap(math.atan2(uy, ux) + 0.5 * wobble * math.cos(phase + 6 * s))
            self._step()


def _bounds(task, slack=0.01):
    r = task["robot"]["radius_m"] - slack
    return (r, r, task["arena"]["width_m"] - r, task["arena"]["height_m"] - r)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rng(*parts) -> np.random.Generator:
    h = hashlib.sha256("/".join(map(str, parts)).encode()).digest()
    return np.random.default_rng(int.from_bytes(h[:8], "little"))


# --------------------------------------------------------------------------- agents


def vex_roster(master_seed: int) -> list[Profile]:
    rng = _rng(master_seed, "roster")
    out = []
    for i in range(1, 13):
        skill = float(np.clip(rng.beta(2.2, 2.0), 0.05, 0.98))
        out.append(
            Profile(
                id=f"driver-{i:02d}",
                cls="human",
                name=f"Driver {i:02d}",
                v_max=0.9 + 0.7 * skill,
                accel=1.5 + 1.5 * skill,
                turn_rate=2.2 + 1.8 * skill,
                stop_err=0.11 - 0.07 * skill,
                head_err=0.16 - 0.10 * skill,
                wobble=0.06 - 0.04 * skill,
                closed_loop=True,
                corr_gain=0.55,
                reaction=(0.35 - 0.2 * skill, 0.8 - 0.4 * skill),
                hesitate_p=0.25 - 0.2 * skill,
                pick_time=(0.6 - 0.3 * skill, 1.2 - 0.6 * skill),
                placement_spread=1.2 - 0.6 * skill,
                suboptimal_order_p=0.4 - 0.35 * skill,
                track_noise=0.006,
                meta={"skill": round(skill, 3)},
            )
        )
    out += [
        Profile("team-auton-a", "human_auton", "Team auton A (odometry + GPS correction)", v_max=1.1, accel=2.2,
                stop_err=0.02, head_err=0.03, closed_loop=True, max_corrections=3, reaction=(0.05, 0.1),
                pick_time=(0.3, 0.4), placement_spread=0.4),
        Profile("team-auton-b", "human_auton", "Team auton B (odometry only)", v_max=1.2, accel=2.5,
                stop_err=0.02, head_err=0.03, drift_per_m=0.008, closed_loop=False, max_corrections=0,
                pick_time=(0.3, 0.4), placement_spread=0.5),
        Profile("team-auton-c", "human_auton", "Team auton C (timed, open loop)", v_max=1.0, accel=2.0,
                stop_err=0.035, head_err=0.06, drift_per_m=0.02, closed_loop=False, max_corrections=0,
                pick_time=(0.4, 0.5), placement_spread=0.7),
        Profile("llm-auton-r1", "ai", "LLM-written auton, round 1", v_max=1.4, accel=3.0,
                stop_err=0.04, head_err=0.07, drift_per_m=0.025, closed_loop=False, max_corrections=0,
                pick_time=(0.3, 0.4), placement_spread=0.9, suboptimal_order_p=1.0, meta={"feedback_rounds": 1}),
        Profile("llm-auton-r3", "ai", "LLM-written auton, round 3", v_max=1.3, accel=2.8,
                stop_err=0.025, head_err=0.04, drift_per_m=0.01, closed_loop=True, max_corrections=1,
                pick_time=(0.3, 0.4), placement_spread=0.6, meta={"feedback_rounds": 3}),
        Profile("llm-auton-r10", "ai", "LLM-written auton, round 10", v_max=1.35, accel=2.8,
                stop_err=0.015, head_err=0.025, closed_loop=True, max_corrections=4, reaction=(0.03, 0.06),
                pick_time=(0.25, 0.35), placement_spread=0.35, meta={"feedback_rounds": 10}),
        Profile("driver-assist", "hybrid", "Driver 05 with auto-align", v_max=1.3, accel=2.4, turn_rate=3.2,
                stop_err=0.025, head_err=0.03, wobble=0.03, closed_loop=True, reaction=(0.15, 0.35),
                pick_time=(0.4, 0.7), placement_spread=0.6, track_noise=0.006),
        Profile("random-walk", "baseline", "Random walk (floor)", v_max=0.6, accel=1.5),
    ]
    return out


def aim_roster(master_seed: int) -> list[Profile]:
    """Classroom roster for VEX AIM tasks. Speeds and errors are assumptions, not AIM specifications."""
    rng = _rng(master_seed, "aim-roster")
    out = []
    for i in range(1, 11):
        skill = float(np.clip(rng.beta(2.2, 2.0), 0.05, 0.98))
        out.append(
            Profile(
                id=f"student-{i:02d}",
                cls="human",
                name=f"Student {i:02d} (One Stick controller)",
                v_max=0.16 + 0.16 * skill,
                accel=0.6 + 0.6 * skill,
                turn_rate=1.8 + 1.4 * skill,
                stop_err=0.045 - 0.03 * skill,
                head_err=0.20 - 0.12 * skill,
                wobble=0.02 - 0.014 * skill,
                closed_loop=True,
                corr_gain=0.55,
                reaction=(0.4 - 0.25 * skill, 0.9 - 0.5 * skill),
                hesitate_p=0.3 - 0.22 * skill,
                pick_time=(0.6 - 0.3 * skill, 1.2 - 0.6 * skill),
                placement_spread=1.1 - 0.6 * skill,
                suboptimal_order_p=0.45 - 0.35 * skill,
                track_noise=0.003,
                holonomic=True,
                meta={"skill": round(skill, 3)},
            )
        )
    common = dict(holonomic=True, track_noise=0.002)
    out += [
        Profile("class-blocks", "human_auton", "Class code A (Blocks, gyro and encoders)", v_max=0.28, accel=0.9,
                turn_rate=2.5, stop_err=0.008, head_err=0.03, drift_per_m=0.012, closed_loop=False, max_corrections=0,
                pick_time=(0.4, 0.5), placement_spread=0.5, **common),
        Profile("class-python-vision", "human_auton", "Class code B (Python, AI Vision on tags and barrels)", v_max=0.26,
                accel=0.9, turn_rate=2.5, stop_err=0.01, head_err=0.04, closed_loop=True, max_corrections=3,
                reaction=(0.15, 0.3), pick_time=(0.4, 0.6), placement_spread=0.45, **common),
        Profile("llm-python-r1", "ai", "LLM-written Python, round 1", v_max=0.35, accel=1.2, turn_rate=3.0,
                stop_err=0.015, head_err=0.08, drift_per_m=0.035, closed_loop=False, max_corrections=0,
                pick_time=(0.3, 0.4), placement_spread=0.9, suboptimal_order_p=1.0,
                meta={"feedback_rounds": 1}, **common),
        Profile("llm-python-r3", "ai", "LLM-written Python, round 3 (uses AI Vision)", v_max=0.3, accel=1.1,
                turn_rate=2.8, stop_err=0.01, head_err=0.04, closed_loop=True, max_corrections=2, reaction=(0.1, 0.2),
                pick_time=(0.3, 0.45), placement_spread=0.5, meta={"feedback_rounds": 3}, **common),
        Profile("student-assist", "hybrid", "Student 04 with AI Vision auto-aim", v_max=0.28, accel=1.0,
                turn_rate=2.8, stop_err=0.012, head_err=0.05, wobble=0.008, closed_loop=True, reaction=(0.2, 0.4),
                pick_time=(0.35, 0.6), placement_spread=0.6, holonomic=True, track_noise=0.003),
        Profile("random-walk", "baseline", "Random walk (floor)", v_max=0.2, accel=0.6,
                meta={"walk": (0.25, 0.2)}, **common),
    ]
    return out


def room_roster(master_seed: int) -> list[Profile]:
    rng = _rng(master_seed, "room-roster")
    out = []
    for i in range(1, 7):
        skill = float(np.clip(rng.beta(2.5, 2.0), 0.1, 0.95))
        out.append(Profile(f"person-{i:02d}", "human", f"Person {i:02d} with stick vacuum",
                           v_max=0.45 + 0.25 * skill, accel=1.0, wobble=0.02,
                           meta={"skill": round(skill, 3), "lane_m": round(0.36 - 0.1 * skill, 3),
                                 "skip_p": round(0.2 - 0.17 * skill, 3), "stop_at_s": round(110 + 60 * skill, 1)},
                           track_noise=0.008))
    out += [
        Profile("vac-systematic", "product", "Robot vacuum, systematic (lane-planning)", v_max=0.30, accel=0.5,
                meta={"lane_m": 0.26}, track_noise=0.008),
        Profile("vac-bump", "product", "Robot vacuum, bump-and-turn", v_max=0.30, accel=0.5, track_noise=0.008),
        Profile("llm-cover-r3", "ai", "LLM-written coverage planner, round 3", v_max=0.35, accel=0.6,
                meta={"lane_m": 0.30, "feedback_rounds": 3}, track_noise=0.008),
        Profile("random-walk", "baseline", "Random walk (floor)", v_max=0.3, accel=0.5, track_noise=0.008),
    ]
    return out


# --------------------------------------------------------------------------- behaviours


def _offset(rng, p: Profile, sim: Sim, scale=1.0):
    """Position error at a stop: random aim error plus accumulated drift for open-loop agents."""
    ex, ey = rng.normal(0, p.stop_err * scale, 2)
    if not p.closed_loop and p.drift_per_m > 0:
        dx, dy = p.meta.setdefault("_drift_dir", (math.cos(rng.uniform(0, 6.283)), math.sin(rng.uniform(0, 6.283))))
        ex += dx * p.drift_per_m * sim.odometer
        ey += dy * p.drift_per_m * sim.odometer
    return ex, ey


def _random_walk(sim: Sim, rng, task, step=0.6, speed=0.5):
    w, h = task["arena"]["width_m"], task["arena"]["height_m"]
    r = task["robot"]["radius_m"]
    while True:
        tx = float(np.clip(sim.x + rng.normal(0, step), r, w - r))
        ty = float(np.clip(sim.y + rng.normal(0, step), r, h - r))
        sim.drive_to(tx, ty, speed, 1.0)
        sim.pause(rng.uniform(0.2, 1.0))


def run_waypoints(task, layout, p: Profile, rng, dt):
    s = layout["start"]
    sim = Sim(s["x"], s["y"], s["theta"], task["budget"]["time_s"], dt, rng, p.track_noise, _bounds(task))
    tol, htol = task["goal"]["pos_tol_m"], task["goal"]["heading_tol_rad"]
    p = replace(p, meta=dict(p.meta))
    try:
        if p.cls == "baseline":
            _random_walk(sim, rng, task, *p.meta.get("walk", (0.6, 0.5)))
        for w in layout["waypoints"]:
            ex, ey = _offset(rng, p, sim)
            sim.drive_to(w["x"] + ex, w["y"] + ey, p.v_max, p.accel, p.wobble, p.turn_rate, p.holonomic)
            sim.turn_to(w["theta"] + rng.normal(0, p.head_err), p.turn_rate)
            n = 0
            while p.closed_loop and n < p.max_corrections:
                if math.hypot(sim.x - w["x"], sim.y - w["y"]) <= tol * 0.9 and abs(_wrap(sim.th - w["theta"])) <= htol * 0.9:
                    break
                n += 1
                sim.pause(rng.uniform(*p.reaction))
                g = p.corr_gain**n
                ex, ey = rng.normal(0, p.stop_err * g, 2)
                sim.drive_to(w["x"] + ex, w["y"] + ey, min(p.v_max, 0.35), p.accel, 0.0, p.turn_rate, p.holonomic)
                sim.turn_to(w["theta"] + rng.normal(0, p.head_err * g), p.turn_rate)
            sim.pause(rng.uniform(*p.reaction) + (rng.uniform(0.3, 1.2) if rng.random() < p.hesitate_p else 0))
        sim.pause(0.2)
    except BudgetSpent:
        pass
    return sim, None


def run_delivery(task, layout, p: Profile, rng, dt):
    s = layout["start"]
    sim = Sim(s["x"], s["y"], s["theta"], task["budget"]["time_s"], dt, rng, p.track_noise, _bounds(task))
    zones = task["arena"]["zones"]
    default_zone = task["goal"].get("zone")
    final = {o["id"]: (o["x"], o["y"]) for o in layout["objects"]}
    remaining = [dict(o) for o in layout["objects"]]
    carrying: Optional[str] = None
    small = task["robot"]["radius_m"] < 0.1  # classroom robots (VEX AIM) vs competition robots
    intake = 0.035 if small else 0.12
    drop_noise = 0.4 if small else 1.0
    rects = _inflated(task, task["robot"]["radius_m"])
    p = replace(p, meta=dict(p.meta))

    def go(tx, ty, v, wobble):
        for wx, wy in _route((sim.x, sim.y), (tx, ty), rects):
            sim.drive_to(wx, wy, v, p.accel, wobble, p.turn_rate, p.holonomic)

    try:
        if p.cls == "baseline":
            _random_walk(sim, rng, task, *p.meta.get("walk", (0.6, 0.5)))
        while remaining:
            if rng.random() < p.suboptimal_order_p:
                o = remaining[0] if p.suboptimal_order_p >= 1 else remaining[int(rng.integers(len(remaining)))]
            else:
                o = min(remaining, key=lambda o: math.hypot(o["x"] - sim.x, o["y"] - sim.y))
            remaining.remove(o)
            ex, ey = _offset(rng, p, sim, 1.5)
            go(o["x"] + ex, o["y"] + ey, p.v_max, p.wobble)
            tries = 0
            while math.hypot(sim.x - o["x"], sim.y - o["y"]) > intake:
                if not p.closed_loop or tries >= 3:
                    sim.event("pickup_miss", o["id"])
                    break
                tries += 1
                sim.pause(rng.uniform(*p.reaction))
                sim.drive_to(o["x"], o["y"], min(0.4, p.v_max), p.accel, 0.0, p.turn_rate, p.holonomic)
            else:
                sim.pause(rng.uniform(*p.pick_time))
                sim.event("pickup", o["id"])
                carrying = o["id"]
                zone = zones[o.get("zone") or default_zone]
                zx, zy = zone["x"] + zone["w"] / 2, zone["y"] + zone["h"] / 2
                hx, hy = zone["w"] / 2 * p.placement_spread, zone["h"] / 2 * p.placement_spread
                ax, ay = zx + rng.uniform(-hx, hx), zy + rng.uniform(-hy, hy)
                ex, ey = _offset(rng, p, sim)
                go(ax + ex, ay + ey, p.v_max, p.wobble)
                dx, dy = rng.normal(0, (0.03 + 0.04 * p.placement_spread) * drop_noise, 2)
                final[carrying] = (sim.x + dx, sim.y + dy)
                sim.event("deliver", carrying)
                carrying = None
                sim.pause(rng.uniform(*p.reaction))
    except BudgetSpent:
        if carrying:
            final[carrying] = (sim.x, sim.y)
    world = {"objects": [{"id": k, "x": round(v[0], 4), "y": round(v[1], 4)} for k, v in final.items()]}
    return sim, world


def _inflated(task, margin):
    return [(o["x"] - margin, o["y"] - margin, o["x"] + o["w"] + margin, o["y"] + o["h"] + margin)
            for o in task["arena"]["obstacles"]]


def _blocked(a, b, rects, step=0.05):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    for i in range(n + 1):
        x = a[0] + (b[0] - a[0]) * i / n
        y = a[1] + (b[1] - a[1]) * i / n
        for (x0, y0, x1, y1) in rects:
            if x0 < x < x1 and y0 < y < y1:
                return (x0, y0, x1, y1)
    return None


def _route(a, b, rects):
    """Straight line, or a detour around the first blocking obstacle's corners."""
    hit = _blocked(a, b, rects)
    if not hit:
        return [b]
    x0, y0, x1, y1 = hit
    e = 0.02
    corners = [(x0 - e, y0 - e), (x1 + e, y0 - e), (x1 + e, y1 + e), (x0 - e, y1 + e)]
    best = None
    for i in range(4):
        for j in range(4):
            path = [corners[i]] if i == j else [corners[i], corners[j]]
            pts = [a] + path + [b]
            if any(_blocked(pts[k], pts[k + 1], rects) for k in range(len(pts) - 1)):
                continue
            length = sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]) for k in range(len(pts) - 1))
            if best is None or length < best[0]:
                best = (length, path + [b])
    return best[1] if best else [b]


def _lanes(task, lane, r, rects):
    """Serpentine lane endpoints, split around inflated obstacles."""
    w, h = task["arena"]["width_m"], task["arena"]["height_m"]
    pts, y, left = [], r, True
    while y <= h - r + 1e-9:
        xs = np.arange(r, w - r + 1e-9, 0.02)
        free = [x for x in xs if not any(x0 < x < x1 and y0 < y < y1 for (x0, y0, x1, y1) in rects)]
        segs, cur = [], []
        for x in free:
            if cur and x - cur[-1] > 0.03:
                segs.append((cur[0], cur[-1]))
                cur = []
            cur.append(x)
        if cur:
            segs.append((cur[0], cur[-1]))
        if not left:
            segs = [(b, a) for (a, b) in reversed(segs)]
        for a, b in segs:
            pts += [(a, y), (b, y)]
        y += lane
        left = not left
    return pts


def run_coverage(task, layout, p: Profile, rng, dt):
    s = layout["start"]
    sim = Sim(s["x"], s["y"], s["theta"], task["budget"]["time_s"], dt, rng, p.track_noise, _bounds(task))
    r = task["robot"]["radius_m"]
    rects = _inflated(task, r)
    w, h = task["arena"]["width_m"], task["arena"]["height_m"]
    try:
        if p.cls == "baseline":
            _random_walk(sim, rng, task)
        if p.id == "vac-bump":
            heading = s["theta"]
            while True:
                tx, ty = sim.x + math.cos(heading) * 0.05, sim.y + math.sin(heading) * 0.05
                wall = not (r <= tx <= w - r and r <= ty <= h - r)
                furniture = any(x0 < tx < x1 and y0 < ty < y1 for (x0, y0, x1, y1) in rects)
                if wall or furniture:
                    sim.event("collision", "", "bumper:" + ("wall" if wall else "furniture"))
                    heading = _wrap(heading + rng.uniform(0.6 * math.pi, 1.4 * math.pi))
                    sim.turn_to(heading, 2.0)
                    continue
                sim.drive_to(tx, ty, p.v_max, 5.0, 0.0, 2.0)
        lane = p.meta.get("lane_m", 0.28)
        start_y = r + (rng.uniform(0.03, 0.08) if p.cls == "human" else 0.0)
        tmp_task = dict(task)
        pts = _lanes(tmp_task, lane, r, rects)
        if start_y > r:
            pts = [(x, y + start_y - r) for (x, y) in pts if y + start_y - r <= h - r]
        stop_at = p.meta.get("stop_at_s", 1e9)
        skip_p = p.meta.get("skip_p", 0.0)
        cur = (sim.x, sim.y)
        for i, target in enumerate(pts):
            if sim.t > stop_at:
                break
            if p.cls == "human" and i % 2 == 0 and rng.random() < skip_p:
                continue
            for wp in _route(cur, target, rects):
                jitter = rng.normal(0, 0.015, 2) if p.cls == "human" else (0.0, 0.0)
                sim.drive_to(wp[0] + jitter[0], wp[1] + jitter[1], p.v_max, p.accel,
                             p.wobble if p.cls == "human" else 0.0, 2.5)
                cur = (sim.x, sim.y)
        sim.pause(0.5)
    except BudgetSpent:
        pass
    return sim, None


RUNNERS = {"waypoints": (run_waypoints, DT_FIELD), "delivery": (run_delivery, DT_FIELD), "coverage": (run_coverage, DT_ROOM)}


# --------------------------------------------------------------------------- dataset

DEFAULT_PLAN = {
    "MB-VEX-NAV@1.0": {"seeds": ["dev-01", "dev-02"], "trials": 5, "roster": "vex"},
    "MB-VEX-PICK@1.0": {"seeds": ["dev-01"], "trials": 5, "roster": "vex"},
    "MB-COVER@0.1": {"seeds": ["dev-01"], "trials": 3, "roster": "room"},
    "MB-AIM-NAV@0.1": {"seeds": ["dev-01", "dev-02"], "trials": 5, "roster": "aim"},
    "MB-AIM-SORT@0.1": {"seeds": ["dev-01"], "trials": 5, "roster": "aim"},
}

ROSTERS = {"vex": vex_roster, "room": room_roster, "aim": aim_roster}


def generate(out: "str | Path", master_seed: int = 2026, plan: Optional[dict] = None, quiet: bool = False) -> int:
    """Generate the synthetic dataset. Returns the number of traces written."""
    out = Path(out)
    plan = plan or DEFAULT_PLAN
    n = 0
    for key, cfg in plan.items():
        task = load_task(key)
        runner, dt = RUNNERS[task["goal"]["kind"]]
        roster = ROSTERS[cfg["roster"]](master_seed)
        for seed in cfg["seeds"]:
            layout = task["layouts"][seed]
            for prof in roster:
                for trial in range(cfg["trials"]):
                    rng = _rng(master_seed, key, seed, prof.id, trial)
                    sim, world = runner(task, layout, prof, rng, dt)
                    meta = {k: v for k, v in prof.meta.items() if not k.startswith("_")}
                    write_trace(
                        out / key / seed / prof.id / f"trial-{trial}",
                        task=key, seed=seed, agent_id=prof.id, agent_class=prof.cls, agent_name=prof.name,
                        agent_meta={**meta, "synthetic": True}, trial=trial, tier="S", capture="synthetic",
                        poses=sim.poses, events=sim.events, world_final=world,
                        notes=f"Synthetic run from motionbench.synth (master_seed={master_seed}). Not real data.",
                    )
                    n += 1
            if not quiet:
                print(f"{key} {seed}: {len(roster) * cfg['trials']} traces")
    (out / "README.md").write_text(DATASET_CARD.format(master_seed=master_seed, n=n))
    return n


DATASET_CARD = """# MotionBench synthetic dataset v0.1

**Synthetic. Not real robot or human data.** Generated by `motionbench.synth`
(master seed {master_seed}, {n} traces) to exercise the trace format, the scorer and
the leaderboard before real field data exists. Agent behaviours are hand-tuned
assumptions; any ranking derived from this dataset illustrates the method only.

| Task | Seeds | Agents | Trials |
| --- | --- | --- | --- |
| MB-VEX-NAV@1.0 | dev-01, dev-02 | 12 simulated drivers, 3 team autons, 3 LLM-auton rounds, 1 hybrid, 1 random walk | 5 |
| MB-VEX-PICK@1.0 | dev-01 | same roster | 5 |
| MB-COVER@0.1 | dev-01 | 6 simulated people, 2 robot vacuums, 1 LLM planner, 1 random walk | 3 |
| MB-AIM-NAV@0.1 | dev-01, dev-02 | 10 simulated students, 2 class programs, 2 LLM-Python rounds, 1 hybrid, 1 random walk | 5 |
| MB-AIM-SORT@0.1 | dev-01 | same classroom roster | 5 |

The VEX AIM agents use assumed speeds and errors for a small omni-drive
classroom robot; they are not measurements or specifications of VEX AIM.

Regenerate byte-for-byte:

    python -m motionbench.synth --out data/synthetic-v0.1 --seed {master_seed}

License: CC BY 4.0.
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate the MotionBench synthetic dataset")
    ap.add_argument("--out", default="data/synthetic-v0.1")
    ap.add_argument("--seed", type=int, default=2026)
    a = ap.parse_args(argv)
    n = generate(a.out, a.seed)
    print(f"wrote {n} traces to {a.out}")


if __name__ == "__main__":
    main()
