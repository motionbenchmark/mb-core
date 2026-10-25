"""Run programs written for the VEX AIM **WebSocket Python client** against the
simulated robot.

The client's own code runs unchanged: ``Robot()``, ``move_for``, the 10 s
blocking timeout, ``get_data`` with its sorting and bearing formula,
``has_sports_ball``, ``Timer``, ``wait``, ``Thread``. Only two things are
replaced:

* the connection: instead of WebSockets to a robot on Wi-Fi, the client talks
  to :class:`Firmware`, a simulated AIM robot that answers its commands and sends
  it status packets (20 a second, camera detections 10 a second), driving the
  same :class:`~motionbench.game.world.World` as every other simulated run;
* the clock: ``time``, ``sleep``, ``wait`` and ``threading`` used by the client
  and by the program run on simulated time, so a 60 s trial takes a few seconds.
  Program threads run one at a time, in step with the simulation.

Pass the folder that holds the client package (``vex/``), e.g. a robo-bench
checkout: ``run_program(code, world, client_dir="robo-bench")``.

What the simulated robot assumes (not AIM specifications; the camera numbers were
checked against recorded real runs, where the ball's pixel size and row matched
within a few pixels): the camera sits at the front, 4.5 cm up, tilted 20 degrees
down, with a 68 x 58 degree field of view;
it sees objects up to 0.8 m away and AprilTags only within 65 degrees of
face-on; detections jitter by about a degree and are sometimes missed. The
raw heading and odometry start at about zero, as in recorded runs. A program starts
0.5 s into the trial (Python start-up and connecting), and every command costs
35 ms of round trip. AI model and AprilTag detection start enabled. A held ball
shows up in the camera as the box recorded runs report (146 x 42 px at the
bottom), which is what the client's ``has_sports_ball()`` tests.

Checked against four recorded real Game 1 runs (robo-bench, three models):
replaying each program's exact code reproduces its real outcome on every seed
(two knock the tag down with the ball, one never finds it, one rams it), the
tag ID and pixel sizes the program logs, and end positions within 2-4 cm.
"""

from __future__ import annotations

import builtins
import heapq
import importlib
import json
import math
import sys
import threading as _real_threading
import traceback
import types
from pathlib import Path
from typing import Callable, Optional

from .world import DT, World, _wrap

STATUS_PERIOD = 0.05
VISION_PERIOD = 0.10
CMD_LATENCY = 0.035  # Wi-Fi round trip per command; fitted to step durations in recorded runs
STATUS_READ_COST = 0.0005
LINE_COST = (5000, 0.001)  # every 5000 traced lines cost 1 ms
STARTUP_S = 0.5  # recorded runs: the program's first log line comes ~0.5 s into the trial
TELEMETRY_PERIOD = 0.2  # robo-bench's launcher records at most 5 status samples a second

# camera model
CAM_W, CAM_H = 320, 240
F_H = 160 / math.tan(math.radians(34))
F_V = 120 / math.tan(math.radians(29))
CAM_Z, CAM_PITCH = 0.045, math.radians(20)
VIS_RANGE = 0.8
TAG_VIEW = math.radians(65)
CLASS_ID = {"sports-ball": 0, "barrel-blue": 1, "barrel-orange": 2}
HUE = {"orange": 25.0, "blue": 215.0, "yellow": 55.0, "purple": 280.0}

SYS_MOVE, SYS_TURN, SYS_MOVING, SYS_CAL, SYS_PROG = 1 << 1, 1 << 4, 1 << 5, 1 << 3, 1 << 10


class MatchOver(BaseException):
    """Raised in the program's threads when the trial's time runs out (BaseException, so a bare
    ``except Exception`` in the program can't swallow it)."""


# ===================================================================================== the clock
class _SimThread:
    def __init__(self, clock: "Clock", target, args=(), kwargs=None, name=None, daemon=False):
        self.clock, self.target, self.args, self.kwargs = clock, target, tuple(args or ()), dict(kwargs or {})
        clock._n += 1
        self.name = name or f"Thread-{clock._n} ({getattr(target, '__name__', 'run')})"
        self.daemon = daemon
        self.go = _real_threading.Event()
        self.back = _real_threading.Event()
        self.finished = False
        self.started = False
        self.os = _real_threading.Thread(target=self._main, daemon=True)
        self.acc = 0.0

    def start(self):
        if self.started:
            raise RuntimeError("threads can only be started once")
        self.started = True
        self.clock.threads.append(self)
        self.os.start()
        self.clock._push(self.clock.world.t, self)

    def _main(self):
        self.go.wait()
        self.go.clear()
        clock = self.clock
        clock._local.me = self
        sys.settrace(clock._tracer)
        try:
            if not clock.killed:
                self.target(*self.args, **self.kwargs)
        except (MatchOver, SystemExit):
            pass
        except BaseException as e:  # like threading's excepthook
            clock.console(f"Exception in thread {self.name}:")
            for ln in _format_exc(e, clock.filename):
                clock.console(ln)
        finally:
            sys.settrace(None)
            self.finished = True
            self.back.set()

    def join(self, timeout=None):
        clock = self.clock
        end = None if timeout is None else clock.world.t + float(timeout)
        while not self.finished and (end is None or clock.world.t < end):
            clock.sleep(DT)

    def is_alive(self):
        return self.started and not self.finished


class Clock:
    """Simulated time for the program and the client. The program's main thread is the master: it
    steps the world; other program threads run in turn when their wake-up time comes."""

    def __init__(self, world: World, end_s: float, on_step: Callable[[], None], console: Callable[[str], None], filename: str):
        self.world, self.end, self.on_step, self.console, self.filename = world, end_s, on_step, console, filename
        self.master = _real_threading.get_ident()
        self.sleepers: list = []
        self.threads: list[_SimThread] = []
        self.killed = False
        self._seq = 0
        self._n = 0
        self._acc = 0.0
        self._lines = 0
        self._local = _real_threading.local()

    # -- scheduling -----------------------------------------------------------------------------------
    def _push(self, wake: float, th: _SimThread):
        self._seq += 1
        heapq.heappush(self.sleepers, (wake, self._seq, th))

    def _is_master(self) -> bool:
        return _real_threading.get_ident() == self.master

    def _run_due(self):
        while self.sleepers and self.sleepers[0][0] <= self.world.t + 1e-9:
            _, _, th = heapq.heappop(self.sleepers)
            if th.finished:
                continue
            th.back.clear()
            th.go.set()
            th.back.wait()

    def _step(self):
        if self.world.t >= self.end - 1e-9:
            raise MatchOver()
        self.on_step()
        self._run_due()

    def advance_to(self, t: float):
        while self.world.t < t - 1e-9:
            self._step()

    def sleep(self, s: float):
        s = max(0.0, float(s))
        if self._is_master():
            self.advance_to(self.world.t + s)
            if self.world.t >= self.end - 1e-9:
                raise MatchOver()
            return
        me: _SimThread = self._local.me
        self._push(self.world.t + max(s, DT), me)
        me.back.set()
        me.go.wait()
        me.go.clear()
        if self.killed:
            raise MatchOver()

    def cost(self, s: float):
        """Time that passes while the program computes or reads cached sensor data."""
        if self._is_master():
            self._acc += s
            if self._acc >= DT:
                k = int(self._acc / DT)
                self._acc -= k * DT
                self.sleep(k * DT)
            elif self.world.t >= self.end - 1e-9:
                raise MatchOver()
        else:
            me = self._local.me
            me.acc += s
            if me.acc >= DT:
                k = int(me.acc / DT)
                me.acc -= k * DT
                self.sleep(k * DT)

    def _tracer(self, frame, event, arg):
        if event == "line":
            self._lines += 1
            if self._lines % LINE_COST[0] == 0:
                self.cost(LINE_COST[1])
        return self._tracer

    def finish_threads(self):
        """After the main program returns: keep the clock running while non-daemon threads live."""
        while any(t.is_alive() and not t.daemon for t in self.threads):
            self.advance_to(self.world.t + DT)

    def kill_all(self):
        self.killed = True
        for th in self.threads:
            if th.started and not th.finished:
                th.back.clear()
                th.go.set()
                th.back.wait(timeout=5)

    # -- stand-in modules -------------------------------------------------------------------------------
    def time_module(self, wall0: float) -> types.ModuleType:
        import time as _t

        m = types.ModuleType("time")
        clock = self
        for k in dir(_t):
            if not k.startswith("__"):
                setattr(m, k, getattr(_t, k))

        def now():
            clock.cost(0.00005)
            return wall0 + clock.world.t

        m.time = now
        m.time_ns = lambda: int(now() * 1e9)
        m.monotonic = m.perf_counter = m.process_time = lambda: (clock.cost(0.00005), 1000.0 + clock.world.t)[1]
        m.monotonic_ns = m.perf_counter_ns = lambda: int((1000.0 + clock.world.t) * 1e9)
        m.sleep = self.sleep
        m.localtime = lambda secs=None: _t.localtime(now() if secs is None else secs)
        m.gmtime = lambda secs=None: _t.gmtime(now() if secs is None else secs)
        m.ctime = lambda secs=None: _t.ctime(now() if secs is None else secs)
        m.strftime = lambda fmt, tt=None: _t.strftime(fmt, m.localtime() if tt is None else tt)
        return m

    def threading_module(self) -> types.ModuleType:
        m = types.ModuleType("threading")
        clock = self
        for k in dir(_real_threading):
            if not k.startswith("__"):
                setattr(m, k, getattr(_real_threading, k))

        class Thread:
            def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None):
                self._t = _SimThread(clock, target or self.run, args, kwargs, name, bool(daemon))

            def run(self):
                pass

            def start(self):
                self._t.start()

            def join(self, timeout=None):
                self._t.join(timeout)

            def is_alive(self):
                return self._t.is_alive()

            name = property(lambda self: self._t.name)
            daemon = property(lambda self: self._t.daemon, lambda self, v: setattr(self._t, "daemon", bool(v)))

        class Lock:  # one program thread runs at a time, so a lock never has to wait
            def __init__(self):
                self._held = 0

            def acquire(self, blocking=True, timeout=-1):
                self._held += 1
                return True

            def release(self):
                self._held = max(0, self._held - 1)

            def locked(self):
                return self._held > 0

            __enter__ = lambda self: self.acquire()
            __exit__ = lambda self, *a: self.release()

        class Event:
            def __init__(self):
                self._f = False

            def set(self):
                self._f = True

            def clear(self):
                self._f = False

            def is_set(self):
                return self._f

            def wait(self, timeout=None):
                end = None if timeout is None else clock.world.t + float(timeout)
                while not self._f and (end is None or clock.world.t < end):
                    clock.sleep(DT)
                return self._f

        m.Thread, m.Lock, m.RLock, m.Event = Thread, Lock, Lock, Event
        m.current_thread = lambda: types.SimpleNamespace(name="MainThread" if clock._is_master() else clock._local.me.name)
        return m


# ===================================================================================== the robot
class Firmware:
    """A simulated AIM robot behind the client's connection: answers commands, sends status."""

    def __init__(self, world: World, seed: int = 0):
        self.w = world
        rng = world.rng
        # Recorded real runs start with the raw heading and odometry at ~0 (the robot resets them
        # when a program connects), whatever the API notes say.
        self.raw_heading0 = float(rng.normal(0, 0.1)) % 360.0
        self.raw_odo0 = (float(rng.normal(0, 0.1)), float(rng.normal(0, 0.1)))
        self.pose_offset = (0.0, 0.0)  # set_pose moves the odometry origin
        self.rot_cw = 0.0  # clockwise degrees turned since the start, unwrapped
        self.imu_drift = float(rng.normal(0, 0.02))  # deg/s
        self.mode = "idle"
        self.target: dict = {}
        self.drive: tuple = (0.0, 0.0)  # robot-frame (forward, right) m/s
        self.spin: tuple = (0.0, 0.0, 0.0)  # forward, right m/s, clockwise rad/s
        self.turn_rate = 0.0  # rad/s clockwise
        self.model_on, self.tags_on, self.colors_on = True, True, False
        self.colors: dict[int, dict] = {}
        self.cal_until = -1.0
        self.battery0 = int(rng.integers(28, 90))  # recorded runs report 28-37 %
        self._vision: list = []
        self._vision_t = -1.0
        self._last = (world.vx, world.vy, world.w)

    # -- frames -----------------------------------------------------------------------------------
    def _rot_cw_deg(self) -> float:
        """Clockwise rotation since the start, from the IMU (with a little drift)."""
        return self.rot_cw + self.imu_drift * self.w.t

    def after_step(self):
        self.rot_cw -= math.degrees(self.w.w * DT)

    def raw_heading(self) -> float:
        return (self.raw_heading0 + self._rot_cw_deg()) % 360.0

    def raw_xy(self) -> tuple[float, float]:
        """Wheel odometry in the IMU's raw frame (mm). World odometry is in the start frame; the raw
        frame is the start frame turned by the IMU's raw start heading."""
        bx, by = self._raw_base()
        return bx + self.pose_offset[0], by + self.pose_offset[1]

    def _raw_base(self) -> tuple[float, float]:
        ox, oy = float(self.w.odo[0]) * 1000.0, float(self.w.odo[1]) * 1000.0
        a = math.radians(self.raw_heading0)
        # start frame (x right, y forward) -> raw frame (y = the IMU's raw heading 0)
        return self.raw_odo0[0] + ox * math.cos(a) + oy * math.sin(a), self.raw_odo0[1] - ox * math.sin(a) + oy * math.cos(a)

    # -- commands ---------------------------------------------------------------------------------
    def command(self, cmd: dict) -> dict:
        c = cmd.get("cmd_id", "")
        w = self.w
        ok = {"cmd_id": c, "status": "complete"}
        if c == "program_init":
            return ok
        if c == "drive":
            sp, ang = float(cmd.get("speed", 0)) / 1000.0, math.radians(float(cmd.get("angle", 0)))
            if sp < 0:
                sp, ang = -sp, ang + math.pi
            self.mode = "drive" if sp > 0 else "idle"
            self.drive = (sp * math.cos(ang), sp * math.sin(ang))
            self.turn_rate = 0.0
            return {"cmd_id": c, "status": "in_progress" if sp > 0 else "complete"}
        if c == "drive_for":
            d = float(cmd.get("distance", 0)) / 1000.0
            sp = abs(float(cmd.get("drive_speed", 100))) / 1000.0
            ang = math.radians(float(cmd.get("angle", 0))) + (math.pi if d < 0 else 0.0)
            if abs(d) < 1e-6 or sp <= 0:
                self.mode = "idle"
                return ok
            self.mode = "drive_for"
            self.target = {"dist": abs(d), "ox": float(w.odo[0]), "oy": float(w.odo[1]), "speed": sp,
                           "dir": w.th - ang}  # robot-relative clockwise angle -> field math angle
            return {"cmd_id": c, "status": "in_progress"}
        if c == "turn":
            rate = math.radians(float(cmd.get("turn_rate", 0)))
            self.mode = "turn" if rate else "idle"
            self.turn_rate, self.drive = rate, (0.0, 0.0)
            return {"cmd_id": c, "status": "in_progress" if rate else "complete"}
        if c in ("turn_for", "turn_to"):
            rate = abs(math.radians(float(cmd.get("turn_rate", 75))))
            err = math.radians(float(w.rng.normal(0, w.noise.turn_err_deg)))
            if c == "turn_for":
                goal = w.th - math.radians(float(cmd.get("angle", 0))) - err
            else:
                delta = _wrap(math.radians(float(cmd.get("heading", 0)) - self.raw_heading()))  # clockwise
                goal = w.th - delta - err
            self.mode, self.target, self.drive = "turn_to", {"theta": goal, "speed": rate}, (0.0, 0.0)
            return {"cmd_id": c, "status": "in_progress"}
        if c == "spin_wheels":
            w1, w2, w3 = (float(cmd.get(k, 0)) for k in ("vel1", "vel2", "vel3"))
            r = (w1 + w2 + w3) / 3.0
            fwd, right = (w1 - w2) / 1.732, r - w3
            self.spin = (fwd / 1000.0, right / 1000.0, math.radians(r))
            self.mode = "spin" if any(abs(v) > 1e-9 for v in (w1, w2, w3)) else "idle"
            return ok
        if c == "set_pose":
            bx, by = self._raw_base()
            self.pose_offset = (float(cmd.get("x", 0)) - bx, float(cmd.get("y", 0)) - by)
            return ok
        if c in ("kick_soft", "kick_medium", "kick_hard"):
            w.kick(c.split("_")[1].upper())
            return ok
        if c == "model_detection":
            self.model_on = bool(cmd.get("b_enable", True))
            return ok
        if c == "tag_detection":
            self.tags_on = bool(cmd.get("b_enable", True))
            return ok
        if c == "color_detection":
            self.colors_on = bool(cmd.get("b_enable", True))
            return ok
        if c == "color_description":
            self.colors[int(cmd.get("id", 0))] = {"rgb": (cmd.get("red", 0), cmd.get("green", 0), cmd.get("blue", 0)),
                                                  "hangle": float(cmd.get("hangle", 10)), "hdsat": float(cmd.get("hdsat", 0.2))}
            return ok
        if c == "imu_calibrate":
            self.cal_until = w.t + 2.0
            return ok
        if c in ("code_description", "imu_set_crash_threshold") or c.startswith(("screen", "play", "led", "sound", "stop_sound", "show", "hide")) \
                or c in ("set_led", "lcd_print", "print"):
            return ok
        return {"cmd_id": "cmd_unknown", "status": "error"}

    def stop(self):
        self.mode, self.drive, self.spin, self.turn_rate = "idle", (0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        self.w.cmd_v, self.w.cmd_w = (0.0, 0.0), 0.0

    # -- motion control, every world step -------------------------------------------------------------
    def control(self):
        w = self.w
        if self.mode == "drive":
            f, r = self.drive
            fx, fy = math.cos(w.th), math.sin(w.th)
            rx, ry = math.sin(w.th), -math.cos(w.th)
            w.cmd_v, w.cmd_w = (f * fx + r * rx, f * fy + r * ry), 0.0
        elif self.mode == "spin":
            f, r, cw = self.spin
            fx, fy = math.cos(w.th), math.sin(w.th)
            rx, ry = math.sin(w.th), -math.cos(w.th)
            w.cmd_v, w.cmd_w = (f * fx + r * rx, f * fy + r * ry), -cw
        elif self.mode == "turn":
            w.cmd_v, w.cmd_w = (0.0, 0.0), -self.turn_rate
        elif self.mode == "drive_for":
            t = self.target
            done = math.hypot(w.odo[0] - t["ox"], w.odo[1] - t["oy"])
            rem = t["dist"] - done
            if rem <= 0.0005:
                self.stop()
            else:
                sp = min(t["speed"], math.sqrt(2 * 0.7 * rem) + 0.01)
                w.cmd_v, w.cmd_w = (sp * math.cos(t["dir"]), sp * math.sin(t["dir"])), 0.0
        elif self.mode == "turn_to":
            t = self.target
            err = _wrap(t["theta"] - w.th)
            if abs(err) < math.radians(0.4):
                self.stop()
            else:
                sp = min(t["speed"], math.sqrt(2 * math.radians(500) * abs(err)) + 0.05)
                w.cmd_v, w.cmd_w = (0.0, 0.0), math.copysign(sp, err)
        else:
            w.cmd_v, w.cmd_w = (0.0, 0.0), 0.0

    # -- status ---------------------------------------------------------------------------------------
    def flags(self) -> int:
        w = self.w
        f = SYS_PROG
        if self.mode in ("drive", "drive_for"):
            f |= SYS_MOVE
        if self.mode in ("turn", "turn_to"):
            f |= SYS_TURN
        if self.mode != "idle" or math.hypot(w.vx, w.vy) > 0.002 or abs(w.w) > 0.02:
            f |= SYS_MOVING
        if w.t < self.cal_until:
            f |= SYS_CAL
        return f

    def status(self) -> dict:
        w = self.w
        if w.t - self._vision_t >= VISION_PERIOD - 1e-9:
            self._vision, self._vision_t = self.detect(), w.t
        rx, ry = self.raw_xy()
        dvx, dvy, dw = (w.vx - self._last[0]) / STATUS_PERIOD, (w.vy - self._last[1]) / STATUS_PERIOD, w.w
        self._last = (w.vx, w.vy, w.w)
        fwd_a = (dvx * math.cos(w.th) + dvy * math.sin(w.th)) / 9.81
        right_a = (dvx * math.sin(w.th) - dvy * math.cos(w.th)) / 9.81
        rot = self._rot_cw_deg()  # unwrapped
        yaw = ((self.raw_heading() + 180) % 360) - 180
        self.battery = max(5, self.battery0 - int(w.t / 60))
        return {
            "controller": {"flags": "0x0000", "stick_x": 0, "stick_y": 0, "battery": 0},
            "robot": {"flags": hex(self.flags()), "battery": self.battery, "touch_flags": "0x0000", "touch_x": 0, "touch_y": 0,
                      "robot_x": f"{rx:.3f}", "robot_y": f"{ry:.3f}", "roll": "0.000", "pitch": "0.000", "yaw": f"{yaw:.3f}",
                      "heading": f"{self.raw_heading():.3f}", "rotation": f"{self.raw_heading0 + rot:.3f}",
                      "acceleration": {"x": f"{fwd_a:.3f}", "y": f"{right_a:.3f}", "z": "-1.000"},
                      "gyro_rate": {"x": "0.000", "y": "0.000", "z": f"{-math.degrees(dw):.3f}"},
                      "screen": {"row": "1", "column": "1"}},
            "aivision": {"classnames": {"count": 4, "items": [{"index": 0, "name": "SportsBall"}, {"index": 1, "name": "BlueBarrel"},
                                                               {"index": 2, "name": "OrangeBarrel"}, {"index": 3, "name": "Robot"}]},
                         "objects": {"count": len(self._vision), "items": self._vision}},
        }

    # -- the AI Vision camera ----------------------------------------------------------------------------
    def _cam(self):
        w = self.w
        off = w.R - 0.02  # the camera sits ~2 cm behind the front (matches the held ball's box in recorded runs)
        return w.x + math.cos(w.th) * off, w.y + math.sin(w.th) * off

    @staticmethod
    def _cx_for(bearing_deg: float, cy: float) -> float:
        """Invert the client's bearing formula: the pixel column at which get_data reports this bearing."""
        a2 = -0.000042011
        a1 = 0.22539 + 0.000010433 * cy
        a0 = -34.656 + 0.011526 * cy - 0.00007073 * cy * cy - bearing_deg
        disc = a1 * a1 - 4 * a2 * a0
        if disc < 0:
            return -1.0
        roots = [(-a1 + sq) / (2 * a2) for sq in (math.sqrt(disc), -math.sqrt(disc))]
        return min(roots, key=lambda c: abs(c - CAM_W / 2))

    @staticmethod
    def _row(z: float, d: float) -> float:
        return CAM_H / 2 + F_V * math.tan(math.atan2(CAM_Z - z, max(d, 1e-3)) - CAM_PITCH)

    def _box(self, px: float, py: float, r: float, z0: float, z1: float, width_m: Optional[float] = None, held: bool = False):
        """Pixel box of an object standing at (px, py): (originX, originY, width, height, true bearing) or None."""
        w = self.w
        cx0, cy0 = self._cam()
        dx, dy = px - cx0, py - cy0
        d = max(math.hypot(dx, dy) - (0 if held else 0.0), 0.012)
        b = math.degrees(_wrap(w.th - math.atan2(dy, dx)))  # clockwise from forward
        if abs(b) > 40 or d > VIS_RANGE:
            return None
        top, bot = self._row(z1, d - r * 0.5), self._row(z0, max(d - r, 0.008))
        if top >= CAM_H or bot <= 0:
            return None
        # (objects clipped by the bottom of the image stay detected: recorded runs report a ball at
        # w=129 h=73 cy=199 while it slides out of view, and the held ball as w=146 h=42 cy=218)
        top, bot = max(0.0, top), min(float(CAM_H - 1), bot)
        cy = (top + bot) / 2
        b_noisy = b + float(w.rng.normal(0, w.noise.vision_sd_deg)) * (0.3 if held else 1.0)
        cx = self._cx_for(b_noisy, cy)
        half = F_H * (width_m if width_m is not None else r) / d
        x0, x1 = max(0.0, cx - half), min(float(CAM_W - 1), cx + half)
        if x1 - x0 < 2 or cx < 0 or cx > CAM_W:
            return None
        return int(round(x0)), int(round(top)), max(1, int(round(x1 - x0))), max(1, int(round(bot - top))), b

    def detect(self) -> list[dict]:
        w = self.w
        items = []
        kinds = w.game.raw["objects"]
        for o in w.objects:
            k = kinds[o.kind]
            if o.shape == "tag":
                continue
            if o.held:
                # held on the kicker: the box recorded runs report for a held ball (cx~155, cy~218,
                # 146 x 42 px); barrels assumed similar but taller. This is what has_sports_ball() and
                # has_*_barrel() test (originY > 170 / 160, centre between 120 and 200).
                hw, hh, top = (146, 42, 197) if o.shape == "ball" else (120, 66, 173)
                cx = 156 + float(w.rng.normal(0, 2))
                box = (int(cx - hw / 2), top, hw, hh, 0.0)
            else:
                box = self._box(o.x, o.y, o.r, 0.0, float(k.get("height_m", 2 * o.r)))
            if not box:
                continue
            # misses are mostly small, far objects (recorded runs show near objects detected every frame)
            if not o.held and w.rng.random() < w.noise.vision_miss * min(1.0, (20.0 / max(box[2], 1)) ** 2):
                continue
            x0, y0, bw, bh, _ = box
            if self.model_on and o.kind in CLASS_ID:
                items.append({"type": 4, "id": CLASS_ID[o.kind], "type_str": "aiobj", "originx": x0, "originy": y0, "width": bw, "height": bh,
                              "score": int(w.rng.integers(88, 100)), "name": "", "angle": 0})
            self._color(items, k.get("colour"), x0, y0, bw, bh)
        # AprilTag signs on the field, and tags on the walls
        signs = [(o.x, o.y, o.facing, o.tag_id, o.down, kinds[o.kind]) for o in w.objects if o.shape == "tag"]
        signs += [(t["x"], t["y"], {"north": 0, "east": 90, "south": 180, "west": 270}.get(t.get("facing"), 0), t["id"], False, None)
                  for t in w.game.field.get("tags", [])]
        for x, y, facing, tid, down, k in signs:
            if k is not None:  # the purple sign is a colour blob from any side, standing or fallen
                box = self._box(x, y, 0.016, 0.0, float(k.get("height_m", 0.05)) if not down else 0.006)
                if box:
                    self._color(items, "purple", *box[:4])
            if down or not self.tags_on or w.rng.random() < w.noise.vision_miss:
                continue
            cx0, cy0 = self._cam()
            to_cam = math.atan2(cy0 - y, cx0 - x)
            normal = math.radians(90 - facing)
            view = abs(_wrap(to_cam - normal))
            if view > TAG_VIEW:
                continue
            sq = float((k or {}).get("tag_square_m", 0.05)) / 2
            zc = float((k or {}).get("tag_centre_height_m", 0.035))
            box = self._box(x, y, 0.004, zc - sq, zc + sq, width_m=sq * math.cos(view))
            if not box:
                continue
            x0, y0, bw, bh, _ = box
            item = {"type": 8, "id": int(tid), "type_str": "tag", "originx": x0, "originy": y0, "width": bw, "height": bh, "score": 0,
                    "name": "", "angle": 0}
            item.update({"x0": x0, "y0": y0, "x1": x0 + bw, "y1": y0, "x2": x0 + bw, "y2": y0 + bh, "x3": x0, "y3": y0 + bh})
            items.append(item)
        items.sort(key=lambda it: -it["width"] * it["height"])
        return items[:24]

    def _color(self, items, colour, x0, y0, bw, bh):
        if not self.colors_on or colour not in HUE:
            return
        for cid, c in self.colors.items():
            r, g, b = (float(v) / 255.0 for v in c["rgb"])
            mx, mn = max(r, g, b), min(r, g, b)
            if mx - mn < 1e-6:
                continue
            if mx == r:
                h = (60 * ((g - b) / (mx - mn)) + 360) % 360
            elif mx == g:
                h = 60 * ((b - r) / (mx - mn)) + 120
            else:
                h = 60 * ((r - g) / (mx - mn)) + 240
            if abs((HUE[colour] - h + 180) % 360 - 180) <= c["hangle"] + 10:
                items.append({"type": 1, "id": cid, "type_str": "color", "originx": x0, "originy": y0, "width": bw, "height": bh,
                              "score": 0, "name": "", "angle": 0})


# ===================================================================================== the connection
class _FakeWS:
    def __init__(self, name: str, link: "Link"):
        self.name, self.link, self.connected = name, link, True
        self._reply = ""

    def send(self, payload, opcode=None):
        if self.name != "ws_cmd":
            return
        try:
            cmd = json.loads(payload.decode() if isinstance(payload, (bytes, bytearray)) else payload)
        except Exception:
            cmd = {}
        self.link.clock.sleep(CMD_LATENCY)
        self._reply = json.dumps(self.link.fw.command(cmd))

    def recv(self):
        return self._reply

    def close(self):
        pass

    def connect(self, *a, **k):
        pass


class Link:
    """Wires one run: the client package, its fake connection, the clock and the firmware."""

    def __init__(self, fw: Firmware, clock: Clock):
        self.fw, self.clock = fw, clock
        self.status_thread = None  # the client's status "thread", once the program created its Robot
        self.next_status = 0.0
        self.status_log: list = []  # (t, status) at most every TELEMETRY_PERIOD
        self._next_tel = 0.0


def load_client(client_dir: "str | Path") -> types.ModuleType:
    """Import the client package ``vex`` from ``client_dir`` (once per process)."""
    client_dir = str(Path(client_dir).resolve())
    mod = sys.modules.get("vex")
    if mod is not None and getattr(mod, "__file__", "").startswith(client_dir) and hasattr(mod, "aim"):
        return mod
    for k in [k for k in sys.modules if k == "vex" or k.startswith("vex.")]:
        del sys.modules[k]
    sys.path.insert(0, client_dir)
    try:
        return importlib.import_module("vex")
    finally:
        sys.path.remove(client_dir)


def _format_exc(e: BaseException, filename: str) -> list[str]:
    """A traceback as ``python program.py`` would print it: without this module's frames."""
    tb = e.__traceback__
    while tb is not None and Path(tb.tb_frame.f_code.co_filename).name in ("aim_websocket.py",):
        tb = tb.tb_next
    return "".join(traceback.format_exception(type(e), e, tb)).rstrip("\n").split("\n")


def run_program(code: str, world: World, client_dir: "str | Path", *, filename: str = "program.py", time_s: Optional[float] = None,
                startup_s: float = STARTUP_S, wall0: float = 1.8e9, on_status: Optional[Callable[[float, dict], None]] = None) -> dict:
    """Run a program written for the AIM WebSocket client in ``world`` until it ends or time runs out.

    Returns ``{status, error, exit_code, ended_s, console: [(t, line)], status_log: [(t, status)]}``;
    ``status`` is ``finished``, ``time`` or ``error``."""
    vex = load_client(client_dir)
    aim, util = vex.aim, sys.modules["vex.vex_util"]
    end = float(time_s or world.game.time_s)
    console: list = []
    partial = [""]

    def emit(line: str):
        console.append((round(world.t, 3), line))

    def cprint(*args, sep=" ", end_="\n", file=None, flush=False, **kw):
        text = sep.join(str(a) for a in args) + kw.get("end", end_)
        buf = partial[0] + text
        *lines, partial[0] = buf.split("\n")
        for ln in lines:
            emit(ln)

    fw = Firmware(world)
    link = Link(fw, None)

    def on_step():
        fw.control()
        world.step(DT)
        fw.after_step()
        if world.t + 1e-9 >= link.next_status:
            link.next_status = world.t + STATUS_PERIOD
            st = fw.status()
            if link.status_thread is not None and world.t + 1e-9 >= link._next_tel:  # recorded once the program's Robot exists
                link._next_tel = world.t + TELEMETRY_PERIOD
                link.status_log.append((round(world.t, 3), st))
                if on_status:
                    on_status(world.t, st)
            th = link.status_thread
            if th is not None:  # what the client's status thread does with each packet
                th.current_status = st
                th.update_status_flags()
                th.check_shake_flag()
                th.check_crash_flag()
                th.check_screen_pressing()
                th.check_program_active_flag()
                th.heartbeat = not th.heartbeat

    clock = Clock(world, end, on_step, emit, filename)
    link.clock = clock
    ftime, fthreading = clock.time_module(wall0), clock.threading_module()

    # -- patch the client: connection, clock, threads, console --------------------------------------------
    saved: list = []

    def patch(obj, name, value):
        saved.append((obj, name, getattr(obj, name, _MISSING)))
        setattr(obj, name, value)

    def fake_connect(self, timeout):
        if self.ws_name == "ws_status":
            link.status_thread = self
        return _FakeWS(self.ws_name, link)

    real_status = aim.Robot.status

    def status_prop(self):
        clock.cost(STATUS_READ_COST)
        return self._ws_status_thread.current_status

    real_init = aim.Robot.__init__

    def robot_init(self, host=""):
        real_init(self, host or "sim")

    for m in (aim, util, vex):
        patch(m, "time", ftime)
        patch(m, "print", cprint)
    patch(vex, "threading", fthreading)  # the program's threads; the client's own stay as they are (never started)
    patch(vex, "sleep", ftime.sleep)
    patch(util, "sleep", ftime.sleep) if hasattr(util, "sleep") else None
    patch(aim, "atexit", types.SimpleNamespace(register=lambda *a, **k: None))
    patch(aim, "signal", types.SimpleNamespace(signal=lambda *a, **k: None, SIGINT=2, SIGTERM=15, Signals=lambda n: n))
    patch(aim, "Settings", lambda *a, **k: types.SimpleNamespace(host="sim"))
    patch(aim.WSThread, "connect_websocket", fake_connect)
    patch(aim.WSThread, "start", lambda self: None)
    patch(aim.Robot, "status", property(status_prop))
    patch(aim.Robot, "__init__", robot_init)

    class _VexThread:  # the client's Thread starts right away
        def __init__(self, func, args=None):
            self.t = fthreading.Thread(target=func, args=tuple(args or ()))
            self.t.start()

    patch(aim, "Thread", _VexThread)
    patch(vex, "Thread", _VexThread)

    # -- the program's namespace ----------------------------------------------------------------------------
    real_import = builtins.__import__

    def sim_import(name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0:
            if name == "time":
                return ftime
            if name == "threading":
                return fthreading
        return real_import(name, globals, locals, fromlist, level)

    bi = dict(vars(builtins))
    bi["print"], bi["__import__"] = cprint, sim_import
    g = {"__name__": "__main__", "__file__": filename, "__builtins__": bi}

    import random as _random

    _random.seed(int(world.rng.integers(1 << 30)))
    status, error, exit_code = "finished", None, 0
    old_trace = sys.gettrace()
    try:
        clock.advance_to(min(startup_s, end))  # Python start-up and connecting
        sys.settrace(clock._tracer)
        try:
            exec(compile(code, filename, "exec"), g)
            sys.settrace(None)
            clock.finish_threads()
        except SystemExit as e:
            exit_code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
        except MatchOver:
            status, exit_code = "time", -9
        except Exception as e:  # the program crashed: its traceback goes to the console
            status, error, exit_code = "error", f"{type(e).__name__}: {e}", 1
            sys.settrace(None)
            for ln in _format_exc(e, filename):
                emit(ln)
            try:
                clock.finish_threads()
            except MatchOver:
                status, exit_code = "time", -9
    except MatchOver:
        status, exit_code = "time", -9
    finally:
        sys.settrace(old_trace)
        if partial[0]:
            emit(partial[0])
        try:
            clock.kill_all()
        finally:
            for obj, name, val in reversed(saved):
                if val is _MISSING:
                    delattr(obj, name)
                else:
                    setattr(obj, name, val)
        fw.stop()  # the robot stops when the program ends
    return {"status": status, "error": error, "exit_code": exit_code, "ended_s": round(world.t, 2), "console": console,
            "status_log": link.status_log, "raw_heading0": fw.raw_heading0}


_MISSING = object()
