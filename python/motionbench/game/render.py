"""Render a simulated game run as video: the table seen from a raised camera at
the near end, the robot, the objects, and a score bar.

    motionbench game render games/aim-sort runs/vision_v2/ -o run.mp4
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
from pathlib import Path
from typing import Optional

import numpy as np

from ..match.render import _poly, _shade, draw_box
from ..tracker.track import _cv2
from .spec import Game
from .world import ROBOT_R

COL = {"orange": (30, 120, 245), "blue": (215, 120, 30), "white": (238, 238, 236), "wall": (232, 232, 228)}
WALL_H = 0.07
TEX_PPM = 400


class LookAtCamera:
    """A pinhole camera at ``pos`` looking at ``target`` (z up)."""

    def __init__(self, pos, target, f_px: float, size: tuple[int, int]):
        self.cam_xyz = np.asarray(pos, float)
        fwd = np.asarray(target, float) - self.cam_xyz
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        self.R = np.stack([right, down, fwd])
        self.f, self.size = f_px, size
        self.c = (size[0] / 2, size[1] / 2)

    def project(self, P) -> np.ndarray:
        P = np.atleast_2d(np.asarray(P, float))
        if P.shape[1] == 2:
            P = np.c_[P, np.zeros(len(P))]
        X = (self.R @ (P - self.cam_xyz).T).T
        z = np.maximum(X[:, 2], 1e-6)
        return np.c_[self.f * X[:, 0] / z + self.c[0], self.f * X[:, 1] / z + self.c[1]]

    def facing(self, centre, normal) -> bool:
        return float(np.dot(np.asarray(normal, float), self.cam_xyz - np.asarray(centre, float))) > 0


def draw_prism(img, cam, base: np.ndarray, z0: float, z1: float, colour, top_colour=None):
    """A vertical prism over a convex floor polygon (counter-clockwise)."""
    n = len(base)
    lo = np.c_[base, np.full(n, z0)]
    hi = np.c_[base, np.full(n, z1)]
    plo, phi = cam.project(lo), cam.project(hi)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        e = base[j] - base[i]
        nrm = np.array([e[1], -e[0], 0.0])
        nrm /= np.linalg.norm(nrm) + 1e-12
        mid = (lo[i] + lo[j] + hi[i] + hi[j]) / 4
        if cam.facing(mid, nrm):
            faces.append((np.linalg.norm(mid - cam.cam_xyz), i, j, nrm))
    for _, i, j, nrm in sorted(faces, reverse=True):
        k = 0.6 + 0.35 * max(0.0, float(nrm @ np.array([0.3, -0.7, 0.6])))
        _poly(img, [plo[i], plo[j], phi[j], phi[i]], _shade(colour, k))
    _poly(img, phi, top_colour or _shade(colour, 1.08))
    return phi


def circle(x, y, r, n=18):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.c_[x + r * np.cos(a), y + r * np.sin(a)]


class GameScene:
    def __init__(self, game: Game, width: int = 1280, height: int = 720):
        cv2 = _cv2()
        self.g = game
        F = game.field
        self.W, self.H = F["width_m"], F["height_m"]
        self.size = (width, height)
        self.cam = LookAtCamera((self.W / 2, -0.95, 1.25), (self.W / 2, self.H * 0.52, 0.0), width * 0.92, self.size)
        self.static = self._static(cv2)

    def _static(self, cv2) -> np.ndarray:
        Wp, Hp = self.size
        img = np.zeros((Hp, Wp, 3), np.uint8)
        img[:] = (58, 52, 48)
        # table surround
        _poly(img, self.cam.project(np.array([[-0.25, -0.25], [self.W + 0.25, -0.25], [self.W + 0.25, self.H + 0.3], [-0.25, self.H + 0.3]])), (90, 84, 80))
        n_x, n_y = int(self.W * TEX_PPM), int(self.H * TEX_PPM)
        tex = np.full((n_y, n_x, 3), (205, 206, 202), np.uint8)
        tile = 0.3048
        for i in range(int(self.W / tile) + 1):
            for j in range(int(self.H / tile) + 1):
                if (i + j) % 2:
                    x0, y0 = int(i * tile * TEX_PPM), n_y - int((j + 1) * tile * TEX_PPM)
                    tex[max(0, y0):max(0, y0 + int(tile * TEX_PPM)), x0:x0 + int(tile * TEX_PPM)] = (196, 197, 193)
        for k in range(1, int(self.W / tile) + 1):
            cv2.line(tex, (int(k * tile * TEX_PPM), 0), (int(k * tile * TEX_PPM), n_y), (178, 179, 175), 2)
        for k in range(1, int(self.H / tile) + 1):
            cv2.line(tex, (0, n_y - int(k * tile * TEX_PPM)), (n_x, n_y - int(k * tile * TEX_PPM)), (178, 179, 175), 2)
        for name, z in self.g.field.get("zones", {}).items():
            col = COL.get(z.get("colour", name), (120, 120, 120))
            x0, x1 = int(z["x"] * TEX_PPM), int((z["x"] + z["w"]) * TEX_PPM)
            y0, y1 = n_y - int((z["y"] + z["h"]) * TEX_PPM), n_y - int(z["y"] * TEX_PPM)
            over = tex.copy()
            cv2.rectangle(over, (x0, y0), (x1, y1), col, -1)
            tex = cv2.addWeighted(over, 0.45, tex, 0.55, 0)
            cv2.rectangle(tex, (x0, y0), (x1, y1), col, 6)
        for o in self.g.field.get("obstacles", []):
            cv2.rectangle(tex, (int(o["x"] * TEX_PPM), n_y - int((o["y"] + o["h"]) * TEX_PPM)),
                          (int((o["x"] + o["w"]) * TEX_PPM), n_y - int(o["y"] * TEX_PPM)), (60, 66, 72), -1)
        src = np.float32([[0, 0], [n_x, 0], [n_x, n_y], [0, n_y]])
        dst = np.float32(self.cam.project(np.array([[0, self.H], [self.W, self.H], [self.W, 0], [0, 0]])))
        Hm = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(tex, Hm, self.size, flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full((n_y, n_x), 255, np.uint8), Hm, self.size)
        img[mask > 128] = warped[mask > 128]
        # far and side walls (the near wall is drawn last, over everything)
        t = 0.025
        for x0, y0, x1, y1 in ((0, self.H, self.W, self.H + t), (-t, 0, 0, self.H + t), (self.W, 0, self.W + t, self.H + t)):
            draw_box(img, self.cam, (x0 + x1) / 2, (y0 + y1) / 2, 0.0, x1 - x0, y1 - y0, 0.0, WALL_H, COL["wall"], outline=False)
        for tag in self.g.field.get("tags", []):
            self._tag(img, tag)
        return img

    def _tag(self, img, tag):
        s, z0 = 0.05, 0.012
        x, y = tag["x"], tag["y"]
        nrm = {"north": (0, 1), "south": (0, -1), "east": (1, 0), "west": (-1, 0)}.get(tag.get("facing"), (0, 1))
        if nrm == (0, -1) and y > 0.01:
            pass
        along = np.array([-nrm[1], nrm[0]], float)
        p = np.array([x, y], float) + np.array(nrm, float) * 0.002
        q = [np.r_[p - along * s / 2, z0], np.r_[p + along * s / 2, z0], np.r_[p + along * s / 2, z0 + s], np.r_[p - along * s / 2, z0 + s]]
        if not self.cam.facing(np.r_[p, z0], np.r_[nrm, 0.0]):
            return
        pts = self.cam.project(np.array(q))
        _poly(img, pts, (245, 245, 245))
        c = pts.mean(0)
        _poly(img, c + (pts - c) * 0.7, (20, 20, 20))
        _poly(img, c + (pts - c) * 0.3, (245, 245, 245))

    def _barrel(self, img, x, y, kind, z=0.0):
        k = self.g.object_kind(kind)
        r, h = k.get("radius_m", 0.022), k.get("height_m", 0.05)
        col = COL.get(k.get("colour", "orange"), (100, 100, 100))
        cv2 = _cv2()
        sh = self.cam.project(circle(x + 0.006, y - 0.006, r * 1.1))
        cv2.fillPoly(img, [np.int32(sh)], (120, 120, 118), cv2.LINE_AA)
        draw_prism(img, self.cam, circle(x, y, r), z, z + h, col, _shade(col, 1.2))
        top = self.cam.project(np.c_[circle(x, y, r * 0.55), np.full(18, z + h + 0.0005)])
        cv2.polylines(img, [np.int32(top)], True, _shade(col, 0.7), 1, cv2.LINE_AA)

    def _robot(self, img, x, y, th):
        cv2 = _cv2()
        R = ROBOT_R
        sh = self.cam.project(circle(x + 0.01, y - 0.01, R * 1.05, 24))
        cv2.fillPoly(img, [np.int32(sh)], (110, 110, 108), cv2.LINE_AA)
        base = np.array([[x + R * math.cos(th + a), y + R * math.sin(th + a)] for a in np.linspace(0, 2 * math.pi, 8, endpoint=False) + math.pi / 8])
        draw_prism(img, self.cam, base, 0.0, 0.03, (70, 72, 76))
        body = np.array([[x + R * 0.94 * math.cos(th + a), y + R * 0.94 * math.sin(th + a)] for a in np.linspace(0, 2 * math.pi, 8, endpoint=False) + math.pi / 8])
        draw_prism(img, self.cam, body, 0.03, 0.10, COL["white"], (246, 246, 244))
        # LED ring and screen on top
        ring = self.cam.project(np.c_[circle(x, y, R * 0.6, 24), np.full(24, 0.1005)])
        cv2.polylines(img, [np.int32(ring)], True, (240, 190, 40), 2, cv2.LINE_AA)
        scr = [(x + math.cos(th) * 0.03 + math.cos(th + math.pi / 2) * dx + math.cos(th) * dy,
                y + math.sin(th) * 0.03 + math.sin(th + math.pi / 2) * dx + math.sin(th) * dy) for dx, dy in ((-0.025, -0.02), (0.025, -0.02), (0.025, 0.02), (-0.025, 0.02))]
        _poly(img, self.cam.project(np.c_[np.array(scr), np.full(4, 0.101)]), (60, 45, 35))
        # kicker / magnet at the front
        fx, fy = x + math.cos(th) * R * 0.9, y + math.sin(th) * R * 0.9
        draw_box(img, self.cam, fx, fy, th, 0.02, 0.07, 0.01, 0.05, (50, 50, 55), outline=False)

    def frame(self, t: float, pose, objs, kinds, hud: dict) -> np.ndarray:
        cv2 = _cv2()
        img = self.static.copy()
        items = [(pose[2], lambda: self._robot(img, pose[1], pose[2], pose[3]))]
        for (x, y, held), kind in zip(objs, kinds):
            items.append((y - (0.001 if held else 0), lambda x=x, y=y, kind=kind: self._barrel(img, x, y, kind)))
        for _, fn in sorted(items, key=lambda it: -it[0]):
            fn()
        t_ = 0.025
        draw_box(img, self.cam, self.W / 2, -t_ / 2, 0.0, self.W + 2 * t_, t_, 0.0, WALL_H, COL["wall"], outline=False)
        self._hud(img, t, hud)
        return img

    def _hud(self, img, t, hud):
        cv2 = _cv2()
        Wp, Hp = self.size
        cv2.rectangle(img, (0, 0), (Wp, 52), (24, 22, 20), -1)
        cv2.putText(img, hud.get("title", ""), (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240, 240, 240), 2, cv2.LINE_AA)
        left = max(0.0, hud["time_s"] - t)
        clock = f"{int(left) // 60}:{int(left) % 60:02d}"
        cv2.putText(img, clock, (Wp // 2 - 40, 37), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
        txt = f"{hud['points']:.0f} / {hud['max_points']:.0f}"
        cv2.putText(img, txt, (Wp - 170, 37), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (90, 220, 120), 2, cv2.LINE_AA)
        cv2.rectangle(img, (0, Hp - 34), (Wp, Hp), (24, 22, 20), -1)
        cv2.putText(img, hud.get("who", ""), (18, Hp - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(img, "SIMULATED", (Wp - 140, Hp - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 160, 255), 2, cv2.LINE_AA)


def render_run(game: Game, run_dir: "str | Path", out_path: "str | Path", *, fps: float = 20.0, width: int = 1280,
               height: int = 720, title: Optional[str] = None) -> Path:
    cv2 = _cv2()
    rd = Path(run_dir)
    rep = json.loads((rd / "replay.json").read_text())
    summ = rep["summary"]
    sc = GameScene(game, width, height)
    poses, objs, kinds = rep["poses"], rep["objects"], rep["object_kinds"]
    final = json.loads((rd / "run.json").read_text())
    out_path = Path(out_path)
    tmp = out_path.with_suffix(".tmp.mp4")
    vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), fps, sc.size)
    # points so far: count objects at rest inside a scoring zone (live, approximate until the end)
    def live_points(o_now):
        objs_now = [{"id": i, "kind": k, "x": o[0], "y": o[1], "held": bool(o[2])} for i, k, o in zip(rep["object_ids"], kinds, o_now)]
        return game.score_world(objs_now)["points"]

    who = f"{'Code' if summ['agent_kind'] == 'code' else 'Driver'}: {summ['agent_name']}  |  layout {summ['layout']}  |  seed {summ['seed']}"
    hud = {"title": title or game.raw["title"], "time_s": game.time_s, "max_points": summ["max_points"], "who": who}
    n = int(round(poses[-1][0] * fps)) + 1
    k = 0
    for i in range(n):
        t = i / fps
        while k + 1 < len(poses) and poses[k + 1][0] <= t:
            k += 1
        hud["points"] = live_points(objs[k])
        vw.write(sc.frame(t, poses[k], objs[k], kinds, hud))
    # hold on the final, scored state
    last = [[o["x"], o["y"], int(o["held"])] for o in final["objects"]]
    hud["points"] = final["summary"]["points"]
    endf = sc.frame(game.time_s, poses[-1], last, kinds, hud)
    cv2.putText(endf, f"Final: {final['summary']['points']:.0f} of {final['summary']['max_points']:.0f}", (sc.size[0] // 2 - 150, sc.size[1] // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, (20, 20, 20), 4, cv2.LINE_AA)
    for _ in range(int(fps * 2.5)):
        vw.write(endf)
    vw.release()
    ff = shutil.which("ffmpeg")
    if ff and subprocess.run([ff, "-loglevel", "error", "-y", "-i", str(tmp), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26",
                              "-movflags", "+faststart", str(out_path)]).returncode == 0:
        tmp.unlink()
    else:
        tmp.replace(out_path)
    return out_path
