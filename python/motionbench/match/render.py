"""Render a simulated match as broadcast-style overhead video, with ground truth.

The camera defaults to the geometry fitted from a real event broadcast (a fixed
camera high above the near wall, ~44 degrees down). The picture has what makes
real footage hard: robots of the same alliance that look alike apart from small
coloured plates, tall robots that lean in perspective, raised bars and loaders
that hide them, moving blocks the same colours as the plates, people standing
round the field with arms reaching in, a broadcast overlay, compression noise,
and an optional intro before the overhead shot starts.

Alongside the video, ``gt.csv`` gives every robot's floor position and image box
for every frame, so trackers can be scored (``motionbench match eval``).
"""

from __future__ import annotations

import csv
import zlib
import math
import subprocess
import shutil
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from ..tracker.floorcam import FloorCam
from ..tracker.track import _cv2
from .io import BROADCAST_CAMERA, MatchData

WALL_H = 0.18
TEX_PPM = 280  # floor texture pixels per metre
ALLIANCE_BGR = {"red": (52, 40, 200), "blue": (205, 120, 30)}
BLOCK_BGR = {"red": (60, 50, 215), "blue": (225, 150, 40)}
CHASSIS_BGR = {"black": (40, 40, 44), "silver": (150, 152, 156), "purple": (130, 70, 115), "white": (215, 215, 210),
               "red-metal": (70, 60, 150), "grey": (95, 98, 104)}
ACCENT_BGR = {"none": None, "purple": (150, 60, 120), "orange": (40, 130, 230), "green": (70, 160, 60), "white": (225, 225, 225)}
YELLOW = (30, 170, 235)


# ----------------------------------------------------------------------------- camera
class FieldCamera:
    """Projects field coordinates (x, y on the floor, z up; metres) into the image."""

    def __init__(self, cam: dict, width: Optional[int] = None):
        W0, H0 = cam["frame_size"]
        s = (width or W0) / W0
        self.size = (int(round(W0 * s)) // 2 * 2, int(round(H0 * s)) // 2 * 2)
        corners = np.array(cam["floor_corners_px"], float) * s
        self.fc = FloorCam.from_corners(corners.tolist(), cam["f_px"] * s, cam["camera_height_m"], self.size)
        fl, fr, nr, nl = (self.fc.to_floor(*p) for p in corners)
        self.o = nl
        ex = nr - nl
        self.F = float(np.linalg.norm(ex))
        self.ex = ex / self.F
        self.ey = np.array([-self.ex[1], self.ex[0]])
        if self.ey @ (fl - nl) < 0:
            self.ey = -self.ey
        self.scale_to_field = cam.get("field_m", self.F) / self.F
        self.k = 1.0 / self.scale_to_field  # world metres per field metre
        cw = -self.o
        self.cam_xyz = np.array([cw @ self.ex * self.scale_to_field, cw @ self.ey * self.scale_to_field, self.fc.h * self.scale_to_field])

    def world(self, P: np.ndarray) -> np.ndarray:
        P = np.atleast_2d(np.asarray(P, float))
        if P.shape[1] == 2:
            P = np.c_[P, np.zeros(len(P))]
        xy = self.o + np.outer(P[:, 0] * self.k, self.ex) + np.outer(P[:, 1] * self.k, self.ey)
        return np.c_[xy, P[:, 2] * self.k]

    def project(self, P) -> np.ndarray:
        return self.fc.project(self.world(P))

    def to_field(self, px: float, py: float) -> Optional[np.ndarray]:
        w = self.fc.to_floor(px, py)
        if w is None:
            return None
        d = w - self.o
        return np.array([d @ self.ex, d @ self.ey]) * self.scale_to_field

    def floor_homography(self, ppm: float, F: float) -> np.ndarray:
        """Field-texture pixels (u right, v up the field) -> image pixels."""
        cv2 = _cv2()
        src = np.float32([[0, 0], [F * ppm, 0], [F * ppm, F * ppm], [0, F * ppm]])
        dst = np.float32(self.project(np.array([[0, F], [F, F], [F, 0], [0, 0]])))
        return cv2.getPerspectiveTransform(src, dst)

    def facing(self, centre, normal) -> bool:
        c = np.asarray(centre, float)
        return float(np.dot(np.asarray(normal, float), self.cam_xyz - c)) > 0


# ----------------------------------------------------------------------------- drawing helpers
def _shade(bgr, k):
    return tuple(int(min(255, max(0, c * k))) for c in bgr)


def _poly(img, pts, colour, outline=None):
    cv2 = _cv2()
    p = np.round(np.asarray(pts) * 4).astype(np.int32)
    cv2.fillPoly(img, [p], colour, lineType=cv2.LINE_AA, shift=2)
    if outline is not None:
        cv2.polylines(img, [p], True, outline, 1, lineType=cv2.LINE_AA, shift=2)


LIGHT = np.array([0.35, -0.5, 0.8])
LIGHT /= np.linalg.norm(LIGHT)


def draw_box(img, cam: FieldCamera, cx, cy, th, L, W, z0, z1, colour, outline=True, decorate: Optional[Callable] = None):
    """A box standing on the floor, rotated by ``th``; draws the faces the camera sees."""
    c, s = math.cos(th), math.sin(th)
    hx, hy = L / 2, W / 2
    base = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    ring = np.array([(cx + c * u - s * v, cy + s * u + c * v) for u, v in base])
    lo = np.c_[ring, np.full(4, z0)]
    hi = np.c_[ring, np.full(4, z1)]
    plo, phi = cam.project(lo), cam.project(hi)
    faces = []
    for i in range(4):
        j = (i + 1) % 4
        mid = (lo[i] + lo[j] + hi[i] + hi[j]) / 4
        n = np.array([(ring[j][1] - ring[i][1]), -(ring[j][0] - ring[i][0]), 0.0])
        n /= np.linalg.norm(n) + 1e-12
        if cam.facing(mid, n):
            faces.append((np.linalg.norm(mid - cam.cam_xyz), i, j, n))
    faces.sort(reverse=True)
    line = _shade(colour, 0.45) if outline else None
    for _, i, j, n in faces:
        k = 0.55 + 0.4 * max(0.0, float(n @ LIGHT))
        quad = [plo[i], plo[j], phi[j], phi[i]]
        _poly(img, quad, _shade(colour, k), line)
        if decorate:
            decorate(img, i, lo[i], lo[j], z0, z1)
    _poly(img, phi, _shade(colour, 1.08), line)
    return np.vstack([plo, phi])


def draw_block(img, cam: FieldCamera, x, y, z, colour, r=0.042):
    cv2 = _cv2()
    p = cam.project(np.array([[x, y, z + r]]))[0]
    q = cam.project(np.array([[x + r, y, z + r]]))[0]
    rad = max(1.0, float(np.linalg.norm(q - p)))
    col = BLOCK_BGR[colour]
    shadow = cam.project(np.array([[x, y - 0.01, max(0.0, z - 0.02)]]))[0]
    if z < 0.01:
        cv2.ellipse(img, (int(shadow[0]), int(shadow[1] + rad * 0.5)), (int(rad), max(1, int(rad * 0.5))), 0, 0, 360, (40, 40, 40), -1, cv2.LINE_AA)
    hexa = np.array([[p[0] + rad * math.cos(a), p[1] + rad * 0.9 * math.sin(a)] for a in np.arange(6) * math.pi / 3 + 0.3])
    _poly(img, hexa, _shade(col, 0.85), _shade(col, 0.5))
    cv2.circle(img, (int(p[0] - rad * 0.25), int(p[1] - rad * 0.3)), max(1, int(rad * 0.35)), _shade(col, 1.25), -1, cv2.LINE_AA)


# ----------------------------------------------------------------------------- the static scene
def floor_texture(meta: dict, rng: np.random.Generator) -> np.ndarray:
    cv2 = _cv2()
    F = meta["field"]["size_m"]
    n = int(F * TEX_PPM)
    tile = F / 6
    tex = np.full((n, n, 3), 86, np.uint8)
    tex = (tex.astype(np.int16) + rng.normal(0, 6, (n, n, 1)).astype(np.int16)).clip(0, 255).astype(np.uint8)

    def P(x, y):  # field metres -> texture px (v grows downwards = far side up)
        return int(round(x * TEX_PPM)), int(round((F - y) * TEX_PPM))

    for i in range(6):
        for j in range(6):
            d = int(rng.integers(-5, 6))
            x0, y0 = P(i * tile, (j + 1) * tile)
            x1, y1 = P((i + 1) * tile, j * tile)
            tex[y0:y1, x0:x1] = np.clip(tex[y0:y1, x0:x1].astype(np.int16) + d, 0, 255).astype(np.uint8)
    c = F / 2
    # centre mat: lighter tiles with white outlines
    m0, m1 = c - 1.5 * tile, c + 1.5 * tile
    x0, y0 = P(m0, m1)
    x1, y1 = P(m1, m0)
    tex[y0:y1, x0:x1] = np.clip(tex[y0:y1, x0:x1].astype(np.int16) + 22, 0, 255).astype(np.uint8)
    white = (225, 225, 225)
    lw = max(2, int(0.018 * TEX_PPM))
    for sgn in (1, -1):
        for off in (-0.13, 0.13):
            a = np.array([c - 0.62, c - sgn * 0.62]) + off * np.array([1, sgn]) / math.sqrt(2) * np.array([1, -1])
            b = np.array([c + 0.62, c + sgn * 0.62]) + off * np.array([1, sgn]) / math.sqrt(2) * np.array([1, -1])
            cv2.line(tex, P(*a), P(*b), white, lw, cv2.LINE_AA)
    for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        ox, oy = c + sx * 0.72, c + sy * 0.72
        oct_ = np.array([P(ox + 0.13 * math.cos(a), oy + 0.13 * math.sin(a)) for a in np.arange(8) * math.pi / 4 + math.pi / 8])
        cv2.polylines(tex, [oct_], True, white, lw, cv2.LINE_AA)
    # tile seams
    for i in range(7):
        for d in range(2):
            v = int(i * tile * TEX_PPM)
            seam = (62, 62, 64)
            if d == 0:
                cv2.line(tex, (v, 0), (v, n), seam, 2)
            else:
                cv2.line(tex, (0, v), (n, v), seam, 2)
    # centre line: two white strips
    for off in (-0.035, 0.035):
        cv2.line(tex, P(c + off, 0), P(c + off, F), white, max(2, int(0.02 * TEX_PPM)), cv2.LINE_AA)
    # park zone frames (low rails, drawn flat with a shadow)
    for name, z in meta["field"]["zones"].items():
        if not name.startswith("park"):
            continue
        col = ALLIANCE_BGR[name.split("-")[1]]
        inner_x = z["x"] + z["w"] if name.endswith("red") else z["x"]
        segs = [((z["x"], z["y"]), (z["x"] + z["w"], z["y"])), ((z["x"], z["y"] + z["h"]), (z["x"] + z["w"], z["y"] + z["h"])),
                ((inner_x, z["y"]), (inner_x, z["y"] + z["h"]))]
        for a, b in segs:
            cv2.line(tex, P(a[0], a[1] - 0.012), P(b[0], b[1] - 0.012), (35, 35, 35), int(0.035 * TEX_PPM))
            cv2.line(tex, P(*a), P(*b), col, int(0.03 * TEX_PPM), cv2.LINE_AA)
    return tex


class Person:
    def __init__(self, rng, anchor, facing):
        self.anchor = np.array(anchor, float)
        self.facing = np.array(facing, float)
        self.shirt = tuple(int(v) for v in rng.choice([(40, 40, 40), (30, 30, 160), (150, 90, 40), (200, 200, 200), (60, 60, 60), (40, 90, 40),
                                                        (180, 120, 200), (20, 20, 20), (90, 60, 140)]))
        self.skin = tuple(int(v) for v in rng.choice([(150, 175, 215), (110, 140, 190), (70, 100, 140), (130, 160, 205)]))
        self.hair = tuple(int(v) for v in rng.choice([(20, 20, 25), (40, 50, 70), (30, 40, 60), (90, 130, 170)]))
        self.height = float(rng.uniform(1.45, 1.85))
        self.trousers = tuple(int(v) for v in rng.choice([(60, 50, 40), (110, 80, 50), (30, 30, 30), (150, 150, 150)]))
        self.shoes = tuple(int(v) for v in rng.choice([(240, 240, 240), (30, 30, 30), (60, 60, 200), (200, 120, 60)]))
        self.phase = float(rng.uniform(0, 100))
        self.reach: list[tuple[float, float, np.ndarray]] = []  # (t0, t1, target xyz)

    def draw(self, img, cam: FieldCamera, t: float):
        cv2 = _cv2()
        sway = 0.04 * np.array([math.sin(0.4 * t + self.phase), math.cos(0.3 * t + self.phase * 1.3)])
        base = self.anchor + sway
        side = np.array([-self.facing[1], self.facing[0]])
        sh_z, hd_z = self.height * 0.82, self.height * 0.93
        shoulder = np.r_[base + self.facing * 0.05, sh_z]
        head = np.r_[base, hd_z]
        hip = np.r_[base - self.facing * 0.02, self.height * 0.5]
        pts = cam.project(np.array([hip, shoulder, head, np.r_[base + side * 0.2, sh_z], np.r_[base - side * 0.2, sh_z]]))
        if not np.isfinite(pts).all():
            return
        ph, ps, pd, pl, pr = pts
        width = max(4, int(np.linalg.norm(pl - pr)))
        I = lambda q: tuple(int(v) for v in q)
        for k in (1, -1):  # legs and shoes
            foot = cam.project(np.array([np.r_[base + side * 0.1 * k, 0.0], np.r_[base + side * 0.1 * k, self.height * 0.5]]))
            if np.isfinite(foot).all():
                cv2.line(img, I(foot[0]), I(foot[1]), self.trousers, max(3, int(width * 0.3)), cv2.LINE_AA)
                cv2.ellipse(img, I(foot[0]), (max(3, width // 6), max(2, width // 10)), 0, 0, 360, self.shoes, -1, cv2.LINE_AA)
        cv2.line(img, I(ph), I(ps), self.shirt, int(width * 0.9), cv2.LINE_AA)
        cv2.line(img, tuple(int(v) for v in pl), tuple(int(v) for v in pr), self.shirt, int(width * 0.45), cv2.LINE_AA)
        # arms: resting, or reaching in when a reach is active
        for k, sp in enumerate((pl, pr)):
            hand = np.r_[base + side * (0.22 if k == 0 else -0.22) + self.facing * 0.25, self.height * 0.55]
            for t0, t1, tgt in self.reach:
                if t0 <= t < t1 and k == 0:
                    a = min(1.0, (t - t0) / 0.5, (t1 - t) / 0.5)
                    hand = hand * (1 - a) + tgt * a
            ph2 = cam.project(np.array([hand]))[0]
            if np.isfinite(ph2).all():
                cv2.line(img, tuple(int(v) for v in sp), tuple(int(v) for v in ph2), self.shirt, max(3, width // 4), cv2.LINE_AA)
                cv2.circle(img, tuple(int(v) for v in ph2), max(2, width // 7), self.skin, -1, cv2.LINE_AA)
        r = max(3, int(width * 0.28))
        cv2.circle(img, tuple(int(v) for v in pd), r, self.hair, -1, cv2.LINE_AA)


class Scene:
    def __init__(self, m: MatchData, width: Optional[int] = None, seed: int = 0, overlay: bool = True, people: bool = True):
        _cv2()
        self.m, self.rng = m, np.random.default_rng(seed)
        self.cam = FieldCamera(m.meta.get("camera", BROADCAST_CAMERA), width)
        self.W, self.H = self.cam.size
        self.F = m.meta["field"]["size_m"]
        self.overlay, self.with_people = overlay, people
        self.gain = float(self.rng.uniform(0.9, 1.1))
        self.tint = self.rng.uniform(0.95, 1.05, 3)
        self.static = self._static()
        self.people = self._people() if people else []
        self.robot_parts = {r["id"]: self._robot_parts(r) for r in m.robots}

    # -- static picture: venue, floor, far and side walls --------------------------------------------
    def _static(self) -> np.ndarray:
        cv2 = _cv2()
        W, H, F = self.W, self.H, self.F
        img = np.zeros((H, W, 3), np.uint8)
        carpet = self.rng.integers(0, 4, (H // 6 + 1, W // 6 + 1))
        pal = np.array([[70, 85, 100], [55, 70, 85], [95, 110, 120], [45, 50, 60]], np.uint8)
        img[:] = cv2.resize(pal[carpet], (W, H), interpolation=cv2.INTER_NEAREST)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        # dark floor round the field, in front of the walls
        ring = self.cam.project(np.array([[-0.45, F + 0.45], [F + 0.45, F + 0.45], [F + 0.45, -0.3], [-0.45, -0.3]]))
        _poly(img, ring, (55, 52, 50))
        tex = floor_texture(self.m.meta, self.rng)
        Hm = self.cam.floor_homography(TEX_PPM, F)
        warped = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full(tex.shape[:2], 255, np.uint8), Hm, (W, H))
        img[mask > 128] = warped[mask > 128]
        for wall in ("far", "left", "right"):
            self._wall(img, wall)
        return img

    def _wall(self, img, which):
        F, t, h = self.F, 0.05, WALL_H
        if which == "far":
            a, b, out = (0, F), (F, F), (0, 1)
        elif which == "left":
            a, b, out = (0, 0), (0, F), (-1, 0)
        elif which == "right":
            a, b, out = (F, 0), (F, F), (1, 0)
        else:
            a, b, out = (0, 0), (F, 0), (0, -1)
        a, b, o = np.array(a, float), np.array(b, float), np.array(out, float)
        inner = [np.r_[a, 0], np.r_[b, 0], np.r_[b, h], np.r_[a, h]]
        outer = [np.r_[a + o * t, 0], np.r_[b + o * t, 0], np.r_[b + o * t, h], np.r_[a + o * t, h]]
        top = [np.r_[a, h], np.r_[b, h], np.r_[b + o * t, h], np.r_[a + o * t, h]]
        P = lambda q: self.cam.project(np.array(q))
        if which == "near":
            _poly(img, P(outer), (232, 232, 230))
        else:
            _poly(img, P(inner), (205, 205, 200))
        _poly(img, P(top), (240, 240, 238))
        # checkered strip along the top
        L = float(np.linalg.norm(b - a))
        n = int(L / 0.04)
        d = (b - a) / L
        for k in range(0, n, 2):
            p0, p1 = a + d * k * 0.04, a + d * (k + 1) * 0.04
            q = [np.r_[p0 + o * t * 0.35, h], np.r_[p1 + o * t * 0.35, h], np.r_[p1 + o * t * 0.75, h], np.r_[p0 + o * t * 0.75, h]]
            _poly(img, P(q), (25, 25, 25))

    def _people(self) -> list[Person]:
        F, rng = self.F, self.rng
        ppl = []
        for x in np.linspace(0.2, F - 0.2, int(rng.integers(5, 9))):
            ppl.append(Person(rng, (x + rng.normal(0, 0.08), F + rng.uniform(0.3, 0.55)), (0, -1)))
        for side, fx in ((-1, -0.28), (1, F + 0.28)):
            for y in np.linspace(0.5, F + 0.1, int(rng.integers(3, 6))):
                ppl.append(Person(rng, (fx + side * rng.uniform(0, 0.15), y + rng.normal(0, 0.1)), (-side, 0)))
        # reaching in: people near loaders top them up; a few lean in at random
        loaders = self.m.meta["field"]["loaders"]
        for e in self.m.events:
            if e["type"] == "reload":
                lx, ly = loaders[e["detail"]]
                p = min(ppl, key=lambda p: np.hypot(*(p.anchor - (lx, ly))))
                p.reach.append((e["t_s"] - 0.8, e["t_s"] + 0.6, np.array([lx, ly, 0.55])))
        for _ in range(int(rng.integers(3, 8))):
            p = ppl[int(rng.integers(len(ppl)))]
            t0 = float(rng.uniform(0, self.m.duration))
            tgt = p.anchor + p.facing * rng.uniform(0.5, 0.75)
            p.reach.append((t0, t0 + rng.uniform(1.5, 4), np.r_[tgt, rng.uniform(0.25, 0.5)]))
        return ppl

    def _robot_parts(self, r: dict) -> list[dict]:
        rng = np.random.default_rng([zlib.crc32(r["id"].encode()), int(self.m.meta["seed"])])
        L, W, H = r["size_m"]
        ch = CHASSIS_BGR.get(r["chassis"], (90, 90, 90))
        acc = ACCENT_BGR.get(r["accent"]) or _shade(ch, 1.3)
        parts = [dict(u=0, v=0, L=L, W=W, z0=0.0, z1=0.09, col=_shade(ch, 0.7), plate=True)]
        for _ in range(int(rng.integers(2, 5))):
            l, w = rng.uniform(0.08, L * 0.6), rng.uniform(0.06, W * 0.45)
            parts.append(dict(u=float(rng.uniform(-L / 2 + l / 2, L / 2 - l / 2)), v=float(rng.uniform(-W / 2 + w / 2, W / 2 - w / 2)),
                              L=l, W=w, z0=0.09, z1=float(rng.uniform(0.18, H)), col=ch if rng.random() < 0.7 else (165, 165, 170)))
        parts.append(dict(u=L / 2 - 0.03, v=0, L=0.06, W=W * 0.85, z0=0.02, z1=float(rng.uniform(0.12, 0.25)), col=acc))
        return parts

    # -- per frame ------------------------------------------------------------------------------
    def frame(self, t: float, noise: bool = True) -> tuple[np.ndarray, list[dict]]:
        _cv2()
        img = self.static.copy()
        m, cam, F = self.m, self.cam, self.F
        items = []  # (depth key, draw fn); larger y (farther) drawn first
        objs = self._objs = m.objects_at(t)
        gt = []
        for r in m.robots:
            x, y, th = m.pose(r["id"], t)
            items.append((y, lambda img, r=r, x=x, y=y, th=th: gt.append(self._draw_robot(img, r, x, y, th, t))))
        for o in objs:
            if o["state"] == "floor":
                items.append((o["y"] + 0.03, lambda img, o=o: draw_block(img, cam, o["x"], o["y"], 0.0, o["colour"])))
            elif o["state"] == "carried":
                pass  # drawn by the robot
        for bar in m.meta["field"]["bars"]:
            inside = [o for o in objs if o["state"] == "scored" and o["ref"] == f"long-{bar['name']}"]
            items.append((bar["y"], lambda img, bar=bar, inside=inside: self._draw_bar(img, bar, inside)))
        centre = [o for o in objs if o["state"] == "scored" and o["ref"] == "centre"]
        items.append((F / 2, lambda img: self._draw_centre(img, centre)))
        for name, (lx, ly) in m.meta["field"]["loaders"].items():
            inside = [o for o in objs if o["state"] == "loader" and o["ref"] == name]
            items.append((ly, lambda img, name=name, lx=lx, ly=ly, inside=inside: self._draw_loader(img, lx, ly, inside)))
        for p in self.people:
            items.append((p.anchor[1], lambda img, p=p: p.draw(img, cam, t)))
        items.sort(key=lambda it: -it[0])
        for _, fn in items:
            fn(img)
        self._wall(img, "near")
        if self.overlay:
            self._overlay(img, t)
        if noise:
            img = self._noise(img, t)
        for g in gt:  # visible fraction after the overlay, roughly
            g["in_frame"] = int(g["x1"] > 0 and g["y1"] > 0 and g["x0"] < self.W and g["y0"] < self.H)
        return img, gt

    def _draw_robot(self, img, r, x, y, th, t) -> dict:
        cam = self.cam
        c, s = math.cos(th), math.sin(th)
        allparts = self.robot_parts[r["id"]]
        dist = lambda p: float(np.linalg.norm(np.array([x + c * p["u"] - s * p["v"], y + s * p["u"] + c * p["v"], (p["z0"] + p["z1"]) / 2]) - cam.cam_xyz))
        parts = allparts[:1] + sorted(allparts[1:], key=lambda p: -dist(p))
        pts = []
        plate = ALLIANCE_BGR[r["alliance"]]

        def deco(img, i, a, b, z0, z1):
            # an alliance plate in the middle 60% of each visible side of the base
            a, b = np.asarray(a), np.asarray(b)
            p0, p1 = a + (b - a) * 0.2, a + (b - a) * 0.8
            q = [np.r_[p0[:2], 0.025], np.r_[p1[:2], 0.025], np.r_[p1[:2], 0.08], np.r_[p0[:2], 0.08]]
            _poly(img, cam.project(np.array(q)), plate)
            mid = cam.project(np.array([np.r_[(p0[:2] + p1[:2]) / 2, 0.045]]))[0]
            size = np.linalg.norm(cam.project(np.array([q[0]]))[0] - cam.project(np.array([q[1]]))[0])
            if size > 30:
                _cv2().putText(img, r["team"][:5], (int(mid[0] - size * 0.3), int(mid[1] + 3)), _cv2().FONT_HERSHEY_SIMPLEX,
                               size / 110, (240, 240, 240), 1, _cv2().LINE_AA)

        for p in parts:
            px, py = x + c * p["u"] - s * p["v"], y + s * p["u"] + c * p["v"]
            pts.append(draw_box(img, cam, px, py, th, p["L"], p["W"], p["z0"], p["z1"], p["col"], decorate=deco if p.get("plate") else None))
        # carried blocks peek out of the top
        carried = [o for o in self._objs if o["state"] == "carried" and o["ref"] == r["id"]]
        for k, o in enumerate(carried[:4]):
            u = -0.08 + 0.08 * (k % 2)
            v = -0.06 + 0.12 * (k // 2)
            draw_block(img, cam, x + c * u - s * v, y + s * u + c * v, 0.12, o["colour"], 0.035)
        allp = np.vstack(pts)
        u, v = cam.project(np.array([[x, y, 0.0]]))[0]
        return {"robot": r["id"], "x_m": x, "y_m": y, "theta": th, "u_px": u, "v_px": v,
                "x0": float(allp[:, 0].min()), "y0": float(allp[:, 1].min()), "x1": float(allp[:, 0].max()), "y1": float(allp[:, 1].max())}

    def _draw_bar(self, img, bar, inside):
        cam = self.cam
        x0, x1, y, z, _d = bar["x0"], bar["x1"], bar["y"], bar["z"], bar["depth"]
        for px in (x0, x1):  # posts (A-frames)
            draw_box(img, cam, px, y, 0.0, 0.05, 0.16, 0.0, z, YELLOW)
        draw_box(img, cam, (x0 + x1) / 2, y, 0.0, x1 - x0 + 0.06, 0.05, z - 0.04, z, YELLOW)
        for o in sorted(inside, key=lambda o: -o["y"]):
            draw_block(img, cam, o["x"], o["y"], o["z"] - 0.04, o["colour"])
        # clear tube: just its outline
        cv2 = _cv2()
        for zz in (z + 0.0, z + 0.09):
            a, b = cam.project(np.array([[x0, y - 0.05, zz], [x1, y - 0.05, zz]]))
            cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (215, 215, 210), 1, cv2.LINE_AA)
        for px in ((x0 + x1) / 2 - 0.15, (x0 + x1) / 2 + 0.15):
            a, b = cam.project(np.array([[px, y - 0.05, z], [px, y - 0.05, z + 0.1]]))
            cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (245, 245, 245), 2, cv2.LINE_AA)

    def _draw_centre(self, img, inside):
        c = self.F / 2
        for a in (math.pi / 4, -math.pi / 4):
            draw_box(img, self.cam, c, c, a, 0.42, 0.05, 0.0, 0.10, YELLOW)
        for dx, dy in ((-0.06, -0.06), (0.06, -0.06), (0.06, 0.06), (-0.06, 0.06)):
            draw_box(img, self.cam, c + dx, c + dy, 0.0, 0.03, 0.03, 0.0, 0.28, YELLOW)
        for o in inside:
            draw_block(img, self.cam, o["x"], o["y"], o["z"], o["colour"])

    def _draw_loader(self, img, lx, ly, inside):
        cv2 = _cv2()
        cam = self.cam
        top = 0.58
        for o in sorted(inside, key=lambda o: o["z"]):
            draw_block(img, cam, lx, ly, o["z"], o["colour"], 0.038)
        ring = np.array([[lx + 0.065 * math.cos(a), ly + 0.065 * math.sin(a), top] for a in np.linspace(0, 2 * math.pi, 16)])
        pr = cam.project(ring)
        cv2.polylines(img, [np.int32(pr)], True, (30, 140, 240), max(2, int(self.W / 250)), cv2.LINE_AA)
        for sx in (-0.06, 0.06):
            a, b = cam.project(np.array([[lx + sx, ly, 0.0], [lx + sx, ly, top]]))
            cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (200, 200, 200), 1, cv2.LINE_AA)

    def _overlay(self, img, t):
        cv2 = _cv2()
        W, H = self.W, self.H
        k = W / 1020
        cv2.rectangle(img, (0, 0), (W, int(18 * k)), (8, 8, 8), -1)
        cv2.putText(img, "MotionBench synthetic match - simulated, not real footage", (int(200 * k), int(13 * k)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42 * k, (200, 200, 200), 1, cv2.LINE_AA)
        _poly(img, np.array([[0, 0], [int(140 * k), 0], [int(95 * k), int(75 * k)], [0, int(75 * k)]]), (8, 8, 8))
        _poly(img, np.array([[W - int(70 * k), 0], [W, 0], [W, int(70 * k)], [W - int(45 * k), int(70 * k)]]), (8, 8, 8))
        cv2.putText(img, "MB", (W - int(52 * k), int(45 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.9 * k, (240, 240, 240), 2, cv2.LINE_AA)
        # side score bars
        x0 = W - int(22 * k)
        for i in range(12):
            col = (40, 40, 210) if i < 6 else (200, 110, 30)
            y0 = int((210 + i * 34) * k)
            cv2.rectangle(img, (x0, y0), (W, y0 + int(28 * k)), col, -1)
        cv2.rectangle(img, (W - int(25 * k), H - int(60 * k)), (W, H), (8, 8, 8), -1)
        clock = max(0.0, 15 - t) if t < 15 else max(0.0, 123 - t) if t >= 18 else 105.0
        cv2.putText(img, f"{int(clock) // 60}:{int(clock) % 60:02d}", (int(10 * k), int(40 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * k,
                    (240, 240, 240), 2, cv2.LINE_AA)

    def _noise(self, img, t):
        cv2 = _cv2()
        g = self.gain * (1 + 0.01 * math.sin(t * 2.3))
        out = img.astype(np.float32) * (g * self.tint)
        out += self.rng.normal(0, 3.0, out.shape[:2] + (1,)).astype(np.float32)
        out = cv2.GaussianBlur(np.clip(out, 0, 255).astype(np.uint8), (3, 3), 0.6)
        ok, enc = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 55])
        return cv2.imdecode(enc, cv2.IMREAD_COLOR)

    def intro(self, t: float) -> np.ndarray:
        """Stand-in for a broadcast intro: a dark title card, then a close-up pan."""
        cv2 = _cv2()
        img = np.full((self.H, self.W, 3), 18, np.uint8)
        k = self.W / 1020
        cv2.putText(img, "Synthetic match", (int(330 * k), int(330 * k)), cv2.FONT_HERSHEY_SIMPLEX, 1.4 * k, (230, 230, 230), 2, cv2.LINE_AA)
        cv2.putText(img, f"seed {self.m.meta['seed']}", (int(450 * k), int(380 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * k, (160, 160, 160), 1, cv2.LINE_AA)
        return self._noise(img, t)


def render_match(match_dir, out_path, *, fps: float = 15.0, width: Optional[int] = None, start_s: float = 0.0,
                 end_s: Optional[float] = None, intro_s: float = 0.0, seed: int = 0, overlay: bool = True, people: bool = True,
                 gt_path: Optional[str] = None, progress: Optional[Callable[[float], None]] = None) -> dict:
    """Write ``out_path`` (.mp4) and ground truth (``gt.csv`` next to it unless ``gt_path`` is given)."""
    cv2 = _cv2()
    m = MatchData(match_dir)
    sc = Scene(m, width, seed, overlay, people)
    end_s = m.duration if end_s is None else min(end_s, m.duration)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(".tmp.mp4")
    vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), fps, sc.cam.size)
    gt_path = Path(gt_path) if gt_path else out_path.with_name(out_path.stem + ".gt.csv")
    n_intro = int(round(intro_s * fps))
    n = int(math.floor((end_s - start_s) * fps)) + 1
    with open(gt_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "video_t_s", "match_t_s", "robot", "alliance", "x_m", "y_m", "theta_rad", "u_px", "v_px", "x0", "y0", "x1", "y1", "in_frame"])
        for i in range(n_intro):
            vw.write(sc.intro(i / fps))
        alliance = {r["id"]: r["alliance"] for r in m.robots}
        for i in range(n):
            t = start_s + i / fps
            img, gt = sc.frame(t)
            vw.write(img)
            fi = n_intro + i
            for g in gt:
                w.writerow([fi, f"{fi / fps:.3f}", f"{t:.3f}", g["robot"], alliance[g["robot"]], f"{g['x_m']:.4f}", f"{g['y_m']:.4f}",
                            f"{g['theta']:.4f}", f"{g['u_px']:.1f}", f"{g['v_px']:.1f}", f"{g['x0']:.1f}", f"{g['y0']:.1f}",
                            f"{g['x1']:.1f}", f"{g['y1']:.1f}", g["in_frame"]])
            if progress and i % 30 == 0:
                progress(i / n)
    vw.release()
    ff = shutil.which("ffmpeg")
    if ff:  # re-encode to H.264 so it plays in browsers
        r = subprocess.run([ff, "-loglevel", "error", "-y", "-i", str(tmp), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
                            "-movflags", "+faststart", str(out_path)], capture_output=True)
        if r.returncode == 0:
            tmp.unlink()
    if tmp.exists():
        tmp.replace(out_path)
    cam = {k: v for k, v in m.meta.get("camera", BROADCAST_CAMERA).items()}
    cam["frame_size_rendered"] = list(sc.cam.size)
    return {"video": str(out_path), "gt": str(gt_path), "frames": n_intro + n, "fps": fps, "intro_s": n_intro / fps,
            "match_start_s": start_s, "camera": cam}
