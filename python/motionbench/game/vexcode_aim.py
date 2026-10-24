"""Run a VEXcode AIM Python project against the simulated world.

The project file runs unchanged: its ``from vex import *`` and ``robot=Robot()``
lines get this module's stand-ins, and every robot call moves simulated time
forward. Blocking calls (``move_for``, ``turn_for``, ``turn_to``, ``wait``,
``kicker.place``...) run the world until they finish; getters cost a
millisecond, like a real loop. When the match clock runs out the project is
stopped wherever it is.

Covered: motion (move_at, move_for, move_with_vectors, turn, turn_for, turn_to,
stop_all_movement, set_move_velocity, set_turn_velocity, set_xy_position,
get_x_position, get_y_position, is_move_active, is_turn_active, is_stopped),
inertial (get_heading, get_rotation, set_heading), kicker (kick, place), sensing
(has_sports_ball, has_orange_barrel, has_blue_barrel, has_any_barrel), AI Vision
(get_data for SPORTS_BALL, ORANGE_BARREL, BLUE_BARREL, ALL_TAGS, TAG0-TAG37),
timer, wait. Screen, sound and LEDs are accepted and do nothing. Threads and
controller events are not simulated.
"""

from __future__ import annotations

import builtins
import math
import random
import sys
import types
from typing import Optional

from .world import DT, V_MAX, W_MAX, World, _wrap


class MatchOver(Exception):
    """Raised inside the project when the match clock runs out."""


# ----------------------------------------------------------------------------- constants
class _C(str):
    pass


CONSTS = {n: _C(n) for n in (
    "LEFT", "RIGHT", "FORWARD", "REVERSE", "PERCENT", "MMPS", "DPS", "MSEC", "SECONDS", "MM", "INCHES",
    "HARD", "MEDIUM", "SOFT", "SPORTS_BALL", "ORANGE_BARREL", "BLUE_BARREL", "AIM_ROBOT", "ALL_TAGS", "ALL_CARGO",
    "ALL_LEDS", "LED1", "LED2", "LED3", "LED4", "LED5", "LED6", "RED", "GREEN", "BLUE", "YELLOW", "ORANGE",
    "PURPLE", "CYAN", "WHITE", "BLACK", "HAPPY", "SAD", "SILLY", "ANGRY", "EXCITED", "QUIET", "ANNOYED", "PROUD",
    "THRILLED", "LAUGHING", "LOOK_LEFT", "LOOK_RIGHT", "LOOK_FORWARD", "ACT_HAPPY", "ACT_SAD", "ACT_SILLY",
    "ACT_ANGRY", "ACT_EXCITED", "XAXIS", "YAXIS", "ZAXIS", "DEGREES", "TURNS")}
for _i in range(38):
    CONSTS[f"TAG{_i}"] = _C(f"TAG{_i}")

FOV = math.radians(60)  # assumed horizontal field of view of the AI Vision camera
CAM_W, CAM_H = 320, 240
RANGE = 1.6


class _NoOp:
    def __getattr__(self, name):
        return lambda *a, **k: None


class VisionObject:
    def __init__(self, exists=False, bearing=0.0, width=0, height=0, centerX=0, centerY=0, id=-1, kind="", distance=0.0):
        self.exists, self.bearing, self.width, self.height = exists, bearing, width, height
        self.centerX, self.centerY, self.id, self.kind = centerX, centerY, id, kind
        self.originX, self.originY = centerX - width / 2, centerY - height / 2
        self.rotation = 0.0

    def __repr__(self):
        return f"VisionObject(exists={self.exists}, bearing={self.bearing:.1f}, width={self.width})"


class VisionList(list):
    """A list of detections. Empty lists are falsy, but [0] still gives an object with exists=False,
    so both `if data:` and `if data[0].exists:` styles work."""

    def __getitem__(self, i):
        if not len(self) and i == 0:
            return VisionObject()
        return super().__getitem__(i)


class Timer:
    def __init__(self, sim: "SimRobot"):
        self.sim, self.t0 = sim, 0.0

    def time(self, units=None):
        t = self.sim.world.t - self.t0
        return t if str(units) == "SECONDS" else t * 1000.0

    def reset(self):
        self.t0 = self.sim.world.t


# ----------------------------------------------------------------------------- the robot
class SimRobot:
    def __init__(self, world: World, time_s: float):
        self.world = world
        self.end = time_s
        self.move_pct = 50.0
        self.turn_dps = 75.0
        self.mode = "idle"  # idle | vel | move_for | turn_to
        self.target = None
        self.heading0 = world.start[2]
        self.odo_offset = [0.0, 0.0]
        self.calls = 0
        self.timer = Timer(self)
        self.inertial = _Inertial(self)
        self.kicker = _Kicker(self)
        self.vision = _Vision(self)
        self.screen = _NoOp()
        self.sound = _NoOp()
        self.led = _NoOp()
        self.controller = _NoOp()

    # -- time ---------------------------------------------------------------------------------------
    def _tick(self, n: int = 1):
        w = self.world
        for _ in range(n):
            if w.t >= self.end - 1e-9:
                w.cmd_v, w.cmd_w = (0.0, 0.0), 0.0
                raise MatchOver()
            self._control()
            w.step(DT)

    def _cost(self, ms: float = 1.0):
        """Getters cost a little time, like a real loop (and stop runaway loops)."""
        self.calls += 1
        self._acc = getattr(self, "_acc", 0.0) + ms / 1000.0
        if self._acc >= DT:
            k = int(self._acc / DT)
            self._acc -= k * DT
            self._tick(k)
        elif self.world.t >= self.end - 1e-9:
            raise MatchOver()

    def _run_until(self, done, timeout_s: float = 60.0):
        t0 = self.world.t
        while not done():
            self._tick()
            if self.world.t - t0 > timeout_s:
                break

    # -- frames -------------------------------------------------------------------------------------
    def _heading_deg(self) -> float:
        """Clockwise degrees from the start heading (what the inertial sensor reports)."""
        return math.degrees(_wrap(self.heading0 - self.world.th)) % 360.0

    def _dir(self, angle_deg: float) -> float:
        """Robot-relative angle (clockwise from forward) -> field math angle."""
        return self.world.th - math.radians(angle_deg)

    def _control(self):
        w = self.world
        if self.mode == "move_for":
            tgt = self.target
            done = math.hypot(w.odo[0] - tgt["ox"], w.odo[1] - tgt["oy"])
            remaining = tgt["dist"] - done
            if remaining <= 0.0005:
                w.cmd_v = (0.0, 0.0)
                self.mode = "idle"
                return
            sp = min(tgt["speed"], math.sqrt(2 * 0.7 * remaining) + 0.01)
            a = tgt["dir_field"]
            w.cmd_v = (sp * math.cos(a), sp * math.sin(a))
            w.cmd_w = 0.0
        elif self.mode == "turn_to":
            tgt = self.target
            err = _wrap(tgt["theta"] - w.th)
            if abs(err) < math.radians(0.4):
                w.cmd_w = 0.0
                self.mode = "idle"
                return
            sp = min(tgt["speed"], math.sqrt(2 * math.radians(500) * abs(err)) + 0.05)
            w.cmd_w = math.copysign(sp, err)

    # -- motion API ---------------------------------------------------------------------------------
    def _speed(self, velocity, units) -> float:
        v = self.move_pct if velocity is None else float(velocity)
        return min(V_MAX, v / 1000.0) if str(units) == "MMPS" else min(V_MAX, V_MAX * v / 100.0)

    def _tspeed(self, velocity, units) -> float:
        v = self.turn_dps if velocity is None else float(velocity)
        dps = v * 1.5 if str(units) == "PERCENT" else v
        return min(W_MAX, math.radians(dps))

    def move_at(self, angle, velocity=None, units=None):
        sp = self._speed(velocity, units)
        a = self._dir(float(angle))
        self.mode = "vel"
        self.world.cmd_v = (sp * math.cos(a), sp * math.sin(a))
        self.world.cmd_w = 0.0
        self._cost()

    def move_with_vectors(self, forward, rightward, rotation):
        w = self.world
        fx, fy = math.cos(w.th), math.sin(w.th)
        rx, ry = math.sin(w.th), -math.cos(w.th)
        f, r = V_MAX * float(forward) / 100.0, V_MAX * float(rightward) / 100.0
        self.mode = "vel"
        w.cmd_v = (f * fx + r * rx, f * fy + r * ry)
        w.cmd_w = -W_MAX * float(rotation) / 100.0
        self._cost()

    def move_for(self, distance, angle=0, velocity=None, units=None, wait=True):
        d = float(distance) / 1000.0
        a_deg = float(angle) + (180.0 if d < 0 else 0.0)
        self.mode = "move_for"
        self.target = {"dist": abs(d), "ox": float(self.world.odo[0]), "oy": float(self.world.odo[1]),
                       "dir_field": self._dir(a_deg), "speed": self._speed(velocity, units)}
        self._cost()
        if wait:
            self._run_until(lambda: self.mode != "move_for")

    def turn(self, direction, velocity=None, units=None):
        sp = self._tspeed(velocity, units)
        self.mode = "vel"
        self.world.cmd_v = (0.0, 0.0)
        self.world.cmd_w = -sp if str(direction) == "RIGHT" else sp
        self._cost()

    def turn_for(self, direction, angle, velocity=None, units=None, wait=True):
        a = float(angle) * (1 if str(direction) == "RIGHT" else -1)
        err = self.world.rng.normal(0, self.world.noise.turn_err_deg)
        self._turn_to_theta(self.world.th - math.radians(a + err), velocity, units, wait)

    def turn_to(self, heading, velocity=None, units=None, wait=True):
        err = self.world.rng.normal(0, self.world.noise.turn_err_deg)
        self._turn_to_theta(self.heading0 - math.radians(float(heading) + err), velocity, units, wait)

    def _turn_to_theta(self, theta, velocity, units, wait):
        self.mode = "turn_to"
        self.world.cmd_v = (0.0, 0.0)
        self.target = {"theta": _wrap(theta), "speed": self._tspeed(velocity, units)}
        self._cost()
        if wait:
            self._run_until(lambda: self.mode != "turn_to")

    def stop_all_movement(self):
        self.mode = "idle"
        self.world.cmd_v, self.world.cmd_w = (0.0, 0.0), 0.0
        self._cost()

    def set_move_velocity(self, velocity, units=None):
        self.move_pct = float(velocity) / 2.0 if str(units) == "MMPS" else float(velocity)

    def set_turn_velocity(self, velocity, units=None):
        self.turn_dps = float(velocity) * 1.5 if str(units) == "PERCENT" else float(velocity)

    def set_xy_position(self, x, y):
        self.odo_offset = [float(x) / 1000.0 - float(self.world.odo[0]), float(y) / 1000.0 - float(self.world.odo[1])]

    def get_x_position(self, units=None):
        self._cost()
        return (float(self.world.odo[0]) + self.odo_offset[0]) * 1000.0

    def get_y_position(self, units=None):
        self._cost()
        return (float(self.world.odo[1]) + self.odo_offset[1]) * 1000.0

    def is_move_active(self):
        self._cost()
        return self.mode in ("move_for", "vel") and math.hypot(*self.world.cmd_v) > 0

    def is_turn_active(self):
        self._cost()
        return self.mode == "turn_to" or (self.mode == "vel" and self.world.cmd_w != 0)

    def is_stopped(self):
        self._cost()
        return math.hypot(self.world.vx, self.world.vy) < 1e-3 and abs(self.world.w) < 1e-2

    # -- sensing ----------------------------------------------------------------------------------------
    def _holding(self, kinds):
        self._cost()
        h = self.world.held
        return h is not None and h.kind in kinds

    def has_sports_ball(self):
        return self._holding({"sports-ball"})

    def has_orange_barrel(self):
        return self._holding({"barrel-orange"})

    def has_blue_barrel(self):
        return self._holding({"barrel-blue"})

    def has_any_barrel(self):
        return self._holding({"barrel-orange", "barrel-blue"})


class _Inertial:
    def __init__(self, sim: SimRobot):
        self.sim = sim

    def get_heading(self, units=None):
        self.sim._cost()
        return self.sim._heading_deg()

    def get_rotation(self, units=None):
        self.sim._cost()
        return math.degrees(self.sim.heading0 - self.sim.world.th)

    def get_yaw(self, units=None):
        h = self.get_heading()
        return h - 360 if h > 180 else h

    def set_heading(self, heading, units=None):
        self.sim.heading0 = self.sim.world.th + math.radians(float(heading))

    def calibrate(self):
        self.sim._cost(1.0)

    def is_calibrating(self):
        return False


class _Kicker:
    def __init__(self, sim: SimRobot):
        self.sim = sim

    def kick(self, force=CONSTS["MEDIUM"]):
        self.sim.world.kick(str(force))
        self.sim._run_until(lambda: self.sim.world.t >= self.sim.world.busy_until)

    def place(self):
        self.sim.world.place()
        self.sim._run_until(lambda: self.sim.world.t >= self.sim.world.busy_until)


class _Vision:
    KINDS = {"SPORTS_BALL": {"sports-ball"}, "ORANGE_BARREL": {"barrel-orange"}, "BLUE_BARREL": {"barrel-blue"},
             "ALL_CARGO": {"sports-ball", "barrel-orange", "barrel-blue"}}

    def __init__(self, sim: SimRobot):
        self.sim = sim

    def _px(self, bearing: float, size: float, dist: float):
        f = (CAM_W / 2) / math.tan(FOV / 2)
        w = max(1, int(f * size / max(dist, 0.05)))
        cx = int(CAM_W / 2 + f * math.tan(bearing))
        return w, cx

    def get_data(self, kind=CONSTS["ALL_CARGO"], count=8):
        sim = self.sim
        sim._cost(33.0)  # a new camera frame about 30 times a second
        w, rng, nz = sim.world, sim.world.rng, sim.world.noise
        k = str(kind)
        out = []
        if k in self.KINDS:
            for o in w.objects:
                if o.held or o.kind not in self.KINDS[k]:
                    continue
                d, b = w.to_robot(o.x, o.y)
                d -= 0.03  # camera sits behind the front of the robot... roughly at its edge
                if d > RANGE or abs(b) > FOV / 2 or rng.random() < nz.vision_miss:
                    continue
                bn = b + math.radians(rng.normal(0, nz.vision_sd_deg))
                wpx, cx = self._px(bn, 2 * o.r, d)
                out.append(VisionObject(True, round(math.degrees(bn), 1), wpx, int(wpx * 1.2), cx, 150, -1, o.kind, round(d * 1000)))
        elif k == "ALL_TAGS" or k.startswith("TAG"):
            want = None if k == "ALL_TAGS" else int(k[3:])
            for tag in w.game.field.get("tags", []):
                if want is not None and tag["id"] != want:
                    continue
                d, b = w.to_robot(tag["x"], tag["y"])
                if d > RANGE * 1.5 or abs(b) > FOV / 2 or rng.random() < nz.vision_miss:
                    continue
                bn = b + math.radians(rng.normal(0, nz.vision_sd_deg))
                wpx, cx = self._px(bn, 0.05, d)
                out.append(VisionObject(True, round(math.degrees(bn), 1), wpx, wpx, cx, 80, tag["id"], "tag", round(d * 1000)))
        out.sort(key=lambda v: -v.width)
        return VisionList(out[: int(count)])


def run_code(code: str, world: World, time_s: Optional[float] = None, filename: str = "<routine>") -> dict:
    """Run a VEXcode AIM Python project in ``world`` until it ends or time runs out."""
    sim = SimRobot(world, time_s or world.game.time_s)
    vex = types.ModuleType("vex")
    for k, v in CONSTS.items():
        setattr(vex, k, v)

    def wait(t, units=CONSTS["MSEC"]):
        s = float(t) if str(units) == "SECONDS" else float(t) / 1000.0
        end = world.t + s
        sim._run_until(lambda: world.t >= end - 1e-9, timeout_s=s + 1)

    class Color:
        def __init__(self, *a, **k):
            pass

    class Thread:
        def __init__(self, *a, **k):
            raise NotImplementedError("Threads are not simulated yet: call your functions in order instead")

    vex.Robot = lambda *a, **k: sim
    vex.wait = wait
    vex.Color = Color
    vex.Thread = Thread
    vex.Timer = lambda *a, **k: Timer(sim)
    vex.Brain = lambda *a, **k: types.SimpleNamespace(timer=Timer(sim), screen=_NoOp())
    urandom = types.ModuleType("urandom")
    for n in ("random", "randint", "uniform", "choice", "seed"):
        setattr(urandom, n, getattr(random.Random(0), n))
    saved = {m: sys.modules.get(m) for m in ("vex", "urandom")}
    sys.modules["vex"], sys.modules["urandom"] = vex, urandom
    g = {"__name__": "__main__", "__builtins__": builtins, **CONSTS, "wait": wait, "robot": sim, "Color": Color,
         "Thread": Thread, "timer": sim.timer}
    status, error = "finished", None
    lines = [0]

    def tracer(frame, event, arg):  # pure-Python busy loops still burn match time
        if event == "line" and frame.f_code.co_filename == filename:
            lines[0] += 1
            if lines[0] % 2000 == 0:
                sim._cost(1.0)
        return tracer

    old = sys.gettrace()
    try:
        sys.settrace(tracer)
        exec(compile(code, filename, "exec"), g)  # when the project ends, the robot stops
    except MatchOver:
        status = "time"
    except NotImplementedError as e:
        status, error = "unsupported", str(e)
    except Exception as e:  # the routine crashed: the robot stops, like on the brain
        status, error = "error", f"{type(e).__name__}: {e}"
    finally:
        sys.settrace(old)
        for m, v in saved.items():
            if v is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = v
    world.cmd_v, world.cmd_w = (0.0, 0.0), 0.0
    return {"status": status, "error": error, "ended_s": round(world.t, 2), "api_calls": sim.calls}
