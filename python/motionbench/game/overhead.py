"""Overhead camera frames of a simulated VEX AIM field, drawn to look like the
real overhead webcam: the 18 in field from straight above, tiles with their
square outlines, white walls, a 12 in ruler along the far wall, the robot, the
ball, barrels and AprilTag signs (standing signs look like a purple disc with a
line through it; fallen ones show the tag).

These frames are what a simulated trial "films" every 2 s, and what the model
sees as feedback, so they stay simple and honest: a small SIMULATED label sits
outside the field.

    img = OverheadCamera(game).frame(world_state)
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from ..tracker.track import _cv2

# BGR
FLOOR = (214, 216, 213)
SQUARE = (150, 152, 150)
WALL = (226, 228, 226)
WALL_EDGE = (180, 182, 180)
BG = (36, 34, 34)
RULER = (120, 178, 214)
COLOURS = {"orange": (40, 110, 245), "blue": (215, 120, 35), "yellow": (60, 225, 240), "purple": (150, 40, 105)}


class OverheadCamera:
    def __init__(self, game, width: int = 1280, height: int = 720, field_px: int = 553):
        """Defaults match what the model is sent from the real overhead webcam: 1920 x 1080 frames
        shrunk to 1280 x 720, with the 18 in field about 553 px across."""
        self.g = game
        F = game.field
        self.W, self.H = F["width_m"], F["height_m"]
        self.R = float(game.raw.get("robot", {}).get("radius_m", 0.09))
        self.ppm = field_px / max(self.W, self.H)
        self.margin = int(width * 0.52 - self.W * self.ppm / 2)
        self.top = int((height - self.H * self.ppm) / 2 + 12)
        self.size = (width, height)
        self._static = self._draw_static()

    # field metres -> pixels (y up on the field, down in the image)
    def p(self, x: float, y: float) -> tuple[int, int]:
        return int(round(self.margin + x * self.ppm)), int(round(self.top + (self.H - y) * self.ppm))

    def _draw_static(self) -> np.ndarray:
        cv2 = _cv2()
        Wp, Hp = self.size
        img = np.full((Hp, Wp, 3), BG, np.uint8)
        x0, y0 = self.p(0, self.H)
        x1, y1 = self.p(self.W, 0)
        wall = int(0.025 * self.ppm)
        cv2.rectangle(img, (x0 - wall, y0 - wall), (x1 + wall, y1 + wall), WALL, -1)
        for k in range(0, x1 - x0, 22):  # holes along the walls
            for yy in (y0 - wall // 2, y1 + wall // 2):
                cv2.circle(img, (x0 + 10 + k, yy), 2, WALL_EDGE, -1, cv2.LINE_AA)
        for k in range(0, y1 - y0, 22):
            for xx in (x0 - wall // 2, x1 + wall // 2):
                cv2.circle(img, (xx, y0 + 10 + k), 2, WALL_EDGE, -1, cv2.LINE_AA)
        cv2.rectangle(img, (x0, y0), (x1, y1), FLOOR, -1)
        # each 9 in tile carries a 3 x 3 grid of square outlines
        tile = 0.2286
        pitch = tile / 3
        side = pitch * 0.74
        nx, ny = int(round(self.W / pitch)), int(round(self.H / pitch))
        th = max(2, int(0.0045 * self.ppm))
        for i in range(nx):
            for j in range(ny):
                cx, cy = (i + 0.5) * pitch, (j + 0.5) * pitch
                a, b = self.p(cx - side / 2, cy + side / 2), self.p(cx + side / 2, cy - side / 2)
                cv2.rectangle(img, a, b, SQUARE, th, cv2.LINE_AA)
                gap = int(side * self.ppm * 0.12)  # the outlines are drawn as four brackets
                mx, my = (a[0] + b[0]) // 2, (a[1] + b[1]) // 2
                cv2.line(img, (mx - gap // 2, a[1]), (mx + gap // 2, a[1]), FLOOR, th + 2)
                cv2.line(img, (mx - gap // 2, b[1]), (mx + gap // 2, b[1]), FLOOR, th + 2)
                cv2.line(img, (a[0], my - gap // 2), (a[0], my + gap // 2), FLOOR, th + 2)
                cv2.line(img, (b[0], my - gap // 2), (b[0], my + gap // 2), FLOOR, th + 2)
        for k in range(1, int(round(self.W / tile))):
            cv2.line(img, self.p(k * tile, 0), self.p(k * tile, self.H), (198, 200, 197), 1, cv2.LINE_AA)
        for k in range(1, int(round(self.H / tile))):
            cv2.line(img, self.p(0, k * tile), self.p(self.W, k * tile), (198, 200, 197), 1, cv2.LINE_AA)
        # 12 in ruler lying along the far wall
        r0 = (x0 + 4, max(0, y0 - wall - 26))
        r1 = (x0 + 4 + int(0.3048 * self.ppm), y0 - wall)
        cv2.rectangle(img, r0, r1, RULER, -1)
        for k in range(0, 49):
            xx = r0[0] + int(k * 0.3048 / 48 * self.ppm)
            ln = 12 if k % 4 == 0 else 6
            cv2.line(img, (xx, r1[1]), (xx, r1[1] - ln), (40, 60, 80), 1)
            if k % 4 == 0 and 0 < k < 48:
                cv2.putText(img, str(k // 4), (xx - 3, r0[1] + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (40, 60, 80), 1, cv2.LINE_AA)
        cv2.putText(img, "SIMULATED", (x1 + wall + 16, Hp - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        return img

    # -- objects ------------------------------------------------------------------------------
    def _ball(self, img, x, y, r):
        cv2 = _cv2()
        c, rp = self.p(x, y), max(3, int(r * self.ppm))
        cv2.circle(img, (c[0] + 2, c[1] + 2), rp, (120, 120, 118), -1, cv2.LINE_AA)
        cv2.circle(img, c, rp, COLOURS["yellow"], -1, cv2.LINE_AA)
        cv2.circle(img, c, max(2, rp // 3), (20, 20, 20), -1, cv2.LINE_AA)
        for a in range(5):
            ang = a * 2 * math.pi / 5 + 0.3
            cv2.circle(img, (int(c[0] + 0.75 * rp * math.cos(ang)), int(c[1] + 0.75 * rp * math.sin(ang))), max(1, rp // 5), (20, 20, 20), -1, cv2.LINE_AA)
        cv2.circle(img, c, rp, (30, 30, 30), 1, cv2.LINE_AA)

    def _barrel(self, img, x, y, r, colour):
        cv2 = _cv2()
        c, rp = self.p(x, y), max(3, int(r * self.ppm))
        col = COLOURS.get(colour, (120, 120, 120))
        cv2.circle(img, (c[0] + 3, c[1] + 3), rp, (120, 120, 118), -1, cv2.LINE_AA)
        cv2.circle(img, c, rp, tuple(int(v * 0.8) for v in col), -1, cv2.LINE_AA)
        cv2.circle(img, c, int(rp * 0.8), col, -1, cv2.LINE_AA)
        cv2.circle(img, (c[0] - rp // 4, c[1] - rp // 4), max(1, rp // 4), tuple(min(255, int(v * 1.3) + 30) for v in col), -1, cv2.LINE_AA)

    def _tag(self, img, x, y, r, facing_deg, down):
        cv2 = _cv2()
        c = self.p(x, y)
        f = math.radians(facing_deg)  # clockwise from +y
        nx, ny = math.sin(f), math.cos(f)  # face normal (field)
        ax, ay = ny, -nx  # along the plate
        if not down:
            cv2.circle(img, (c[0] + 2, c[1] + 2), int(r * self.ppm), (120, 120, 118), -1, cv2.LINE_AA)
            cv2.circle(img, c, int(r * self.ppm), COLOURS["purple"], -1, cv2.LINE_AA)
            half = 0.03
            a, b = self.p(x - ax * half, y - ay * half), self.p(x + ax * half, y + ay * half)
            cv2.line(img, a, b, (110, 25, 80), max(2, int(0.004 * self.ppm)), cv2.LINE_AA)
            # a hint of the printed face, on the side it faces
            fa, fb = self.p(x - ax * half * 0.6 + nx * 0.002, y - ay * half * 0.6 + ny * 0.002), self.p(x + ax * half * 0.6 + nx * 0.002, y + ay * half * 0.6 + ny * 0.002)
            cv2.line(img, fa, fb, (235, 235, 235), 1, cv2.LINE_AA)
        else:
            # lying flat behind its base: the plate and its tag are visible from above
            L, Wd = 0.065, 0.033
            cx, cy = x - nx * L / 2, y - ny * L / 2
            pts = [(cx + ax * Wd / 2 + nx * L / 2, cy + ay * Wd / 2 + ny * L / 2), (cx - ax * Wd / 2 + nx * L / 2, cy - ay * Wd / 2 + ny * L / 2),
                   (cx - ax * Wd / 2 - nx * L / 2, cy - ay * Wd / 2 - ny * L / 2), (cx + ax * Wd / 2 - nx * L / 2, cy + ay * Wd / 2 - ny * L / 2)]
            P = np.int32([self.p(*q) for q in pts])
            cv2.fillPoly(img, [P], COLOURS["purple"], cv2.LINE_AA)
            inner = np.int32([self.p(cx + (q[0] - cx) * 0.55, cy + (q[1] - cy) * 0.55) for q in pts])
            cv2.fillPoly(img, [inner], (240, 240, 240), cv2.LINE_AA)
            cv2.circle(img, self.p(cx, cy), max(2, int(0.006 * self.ppm)), (20, 20, 20), -1, cv2.LINE_AA)

    def _robot(self, img, x, y, th):
        """The AIM robot from above: a white cylinder whose round top is a black ring around a glowing
        screen; a dark kicker notch marks the front."""
        cv2 = _cv2()
        R = self.R
        c, rp = self.p(x, y), int(R * self.ppm)
        cv2.circle(img, (c[0] + 3, c[1] + 3), rp, (110, 110, 108), -1, cv2.LINE_AA)
        cv2.circle(img, c, rp, (232, 234, 234), -1, cv2.LINE_AA)
        cv2.circle(img, c, int(rp * 0.86), (28, 28, 30), -1, cv2.LINE_AA)
        cv2.circle(img, c, int(rp * 0.58), (250, 236, 215), -1, cv2.LINE_AA)
        cv2.circle(img, c, int(rp * 0.38), (245, 175, 70), -1, cv2.LINE_AA)
        fx, fy = math.cos(th), math.sin(th)
        a0 = math.degrees(-th)
        cv2.ellipse(img, c, (rp, rp), a0, -30, 30, (40, 40, 44), max(2, rp // 6), cv2.LINE_AA)
        tip = self.p(x + fx * R * 0.72, y + fy * R * 0.72)
        cv2.circle(img, tip, max(2, rp // 8), (240, 240, 240), -1, cv2.LINE_AA)

    def frame(self, robot_pose: Sequence[float], objects: Sequence[dict], label: Optional[str] = None) -> np.ndarray:
        """``robot_pose`` = (x, y, theta_rad); ``objects`` = [{kind, x, y, held, down, facing_deg}]."""
        cv2 = _cv2()
        img = self._static.copy()
        kinds = self.g.raw["objects"]
        held = []
        for o in objects:
            k = kinds[o["kind"]]
            if o.get("held"):
                held.append((o, k))
                continue
            if k.get("shape") == "tag":
                self._tag(img, o["x"], o["y"], k.get("radius_m", 0.016), o.get("facing_deg", 0.0), o.get("down", False))
        for o in objects:
            k = kinds[o["kind"]]
            if o.get("held") or k.get("shape") == "tag":
                continue
            (self._ball if k.get("shape") == "ball" else lambda im, x, y, r: self._barrel(im, x, y, r, k.get("colour")))(img, o["x"], o["y"], k.get("radius_m", 0.02))
        self._robot(img, *robot_pose)
        for o, k in held:
            if k.get("shape") == "ball":
                self._ball(img, o["x"], o["y"], k.get("radius_m", 0.013))
            else:
                self._barrel(img, o["x"], o["y"], k.get("radius_m", 0.014), k.get("colour"))
        if label:
            cv2.putText(img, label, (16, self.size[1] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (190, 190, 190), 1, cv2.LINE_AA)
        return img


def world_objects(world) -> list[dict]:
    """The current objects of a ``World`` in the form ``OverheadCamera.frame`` takes."""
    return [{"kind": o.kind, "x": o.x, "y": o.y, "held": o.held, "down": o.down, "facing_deg": o.facing} for o in world.objects]


def replay_objects(game, layout: dict, track_row: list) -> list[dict]:
    """Objects at one recorded instant of ``World.object_track`` (rows ``[x, y, held, down]``)."""
    out = []
    for o, row in zip(layout["objects"], track_row):
        out.append({"kind": o["kind"], "x": row[0], "y": row[1], "held": bool(row[2]), "down": bool(row[3]) if len(row) > 3 else False,
                    "facing_deg": o.get("facing_deg", 0.0)})
    return out
