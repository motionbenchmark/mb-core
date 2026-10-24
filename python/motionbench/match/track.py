"""A baseline tracker for fixed-camera match video: no tags, no training.

1. The empty field is the per-pixel median of frames spread over the clip
   (robots move, so they drop out of it; the overlay and structures stay).
2. Each frame, pixels that differ from the empty field are foreground.
3. For every floor position on a 5 cm grid, the image box a robot standing
   there would fill is known from the camera. A robot is where that box is full
   of foreground and the ring around it is not (an occupancy map, so tall robots
   leaning in perspective and robots in contact are handled the same way).
4. Peaks become detections; tracks follow them with a constant-velocity guess
   and one-to-one assignment, and coast through short occlusions.
5. Each track's alliance is the majority colour (red or blue) of the saturated
   foreground pixels low on the robot, where the plates are.

It is a reference point for the benchmark, not the last word: it has no notion
of what a robot looks like, so people leaning in, pushed blocks and long stops
all cost it accuracy. ``motionbench match eval`` measures exactly how much.
"""

from __future__ import annotations

import csv
import math
from typing import Callable, Optional

import numpy as np

from ..tracker.track import _cv2
from ..tracker.video import iter_frames, probe
from .eval import _assign
from .io import BROADCAST_CAMERA
from .render import FieldCamera

ROBOT_R = 0.23
ROBOT_H = 0.34
CELL = 0.05


class Occupancy:
    def __init__(self, cam: FieldCamera, F: float):
        self.cam, self.F = cam, F
        g = np.arange(ROBOT_R * 0.8, F - ROBOT_R * 0.8 + 1e-9, CELL)
        X, Y = np.meshgrid(g, g, indexing="xy")
        self.xy = np.c_[X.ravel(), Y.ravel()]
        self.shape = X.shape
        ang = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        ring = np.c_[np.cos(ang), np.sin(ang)] * ROBOT_R
        boxes = []
        for x, y in self.xy:
            pts = np.r_[np.c_[ring + (x, y), np.zeros(8)], np.c_[ring + (x, y), np.full(8, ROBOT_H)]]
            p = cam.project(pts)
            boxes.append([p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()])
        b = np.array(boxes)
        W, H = cam.size
        w, h = b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]
        inner = np.c_[b[:, 0] + 0.12 * w, b[:, 1] + 0.12 * h, b[:, 2] - 0.12 * w, b[:, 3] - 0.05 * h]
        outer = np.c_[b[:, 0] - 0.25 * w, b[:, 1] - 0.15 * h, b[:, 2] + 0.25 * w, b[:, 3] + 0.2 * h]
        clip = lambda a: np.c_[np.clip(a[:, 0], 0, W - 1), np.clip(a[:, 1], 0, H - 1), np.clip(a[:, 2], 1, W), np.clip(a[:, 3], 1, H)].astype(int)
        self.inner, self.outer = clip(inner), clip(outer)
        self.box = clip(b)
        self.a_in = np.maximum(1, (self.inner[:, 2] - self.inner[:, 0]) * (self.inner[:, 3] - self.inner[:, 1]))
        self.a_out = np.maximum(1, (self.outer[:, 2] - self.outer[:, 0]) * (self.outer[:, 3] - self.outer[:, 1])) - self.a_in

    @staticmethod
    def _sum(ii, b):
        return ii[b[:, 3], b[:, 2]] - ii[b[:, 1], b[:, 2]] - ii[b[:, 3], b[:, 0]] + ii[b[:, 1], b[:, 0]]

    def score(self, fg: np.ndarray) -> np.ndarray:
        cv2 = _cv2()
        ii = cv2.integral((fg > 0).astype(np.uint8)).astype(np.int64)
        s_in = self._sum(ii, self.inner)
        s_out = self._sum(ii, self.outer) - s_in
        return s_in / self.a_in - s_out / self.a_out


def detect(score: np.ndarray, occ: Occupancy, n_max: int, thr: float = 0.32, sep: float = 0.36) -> list[tuple[float, float, float, int]]:
    s = score.copy()
    out = []
    for _ in range(n_max):
        k = int(np.argmax(s))
        if s[k] < thr:
            break
        x, y = occ.xy[k]
        # sub-cell refinement: score-weighted mean of the neighbourhood
        near = np.hypot(occ.xy[:, 0] - x, occ.xy[:, 1] - y) < 0.12
        w = np.clip(score[near] - thr * 0.5, 0, None)
        if w.sum() > 0:
            x, y = (occ.xy[near] * w[:, None]).sum(0) / w.sum()
        out.append((float(x), float(y), float(score[k]), k))
        s[np.hypot(occ.xy[:, 0] - occ.xy[k, 0], occ.xy[:, 1] - occ.xy[k, 1]) < sep] = -1
    return out


class Track:
    _next = 0

    def __init__(self, x, y, t):
        self.id = Track._next
        Track._next += 1
        self.x, self.y, self.vx, self.vy = x, y, 0.0, 0.0
        self.t = t
        self.hits, self.miss_s = 1, 0.0
        self.colour = {"red": 0.0, "blue": 0.0}
        self.rows: list[tuple] = []

    def predict(self, t):
        dt = min(t - self.t, 0.5)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, x, y, t):
        dt = max(t - self.t, 1e-3)
        a = 0.5
        self.vx = (1 - a) * self.vx + a * (x - self.x) / dt
        self.vy = (1 - a) * self.vy + a * (y - self.y) / dt
        sp = math.hypot(self.vx, self.vy)
        if sp > 2.5:
            self.vx, self.vy = self.vx * 2.5 / sp, self.vy * 2.5 / sp
        self.x, self.y, self.t = x, y, t
        self.hits += 1
        self.miss_s = 0.0

    @property
    def alliance(self) -> Optional[str]:
        r, b = self.colour["red"], self.colour["blue"]
        if r + b < 5:
            return None
        return "red" if r > b else "blue"


def _colour_votes(frame_hsv, fg, box) -> tuple[float, float]:
    x0, y0, x1, y1 = box
    y0 = int(y0 + (y1 - y0) * 0.55)
    hsv = frame_hsv[y0:y1, x0:x1]
    m = fg[y0:y1, x0:x1] > 0
    if hsv.size == 0:
        return 0.0, 0.0
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    sat = m & (s > 120) & (v > 70)
    red = sat & ((h < 8) | (h > 170))
    blue = sat & (h > 98) & (h < 125)
    return float(red.sum()), float(blue.sum())


def background(video: str, size: tuple[int, int], start_s: float, end_s: float, n: int = 60) -> np.ndarray:
    cv2 = _cv2()
    frames = []
    step = max((end_s - start_s) / n, 0.2)
    for t, f in iter_frames(video, width=size[0], fps=1.0 / step, start_s=start_s, end_s=end_s):
        if f.shape[1] != size[0] or f.shape[0] != size[1]:
            f = cv2.resize(f, size)
        frames.append(f)
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def track_match(video: str, out_csv: str, *, camera: Optional[dict] = None, fps: float = 10.0, start_s: float = 0.0,
                end_s: Optional[float] = None, match_start_s: Optional[float] = None, n_robots: int = 4, diff_thr: int = 28,
                overlay: Optional[str] = None, progress: Optional[Callable[[str], None]] = None) -> dict:
    cv2 = _cv2()
    say = progress or (lambda m: None)
    info = probe(video)
    cam_d = dict(camera or BROADCAST_CAMERA)
    width = int(cam_d["frame_size"][0]) if info.width >= cam_d["frame_size"][0] else info.width
    cam = FieldCamera(cam_d, width)
    F = float(cam_d.get("field_m", 3.566))
    end_s = info.duration_s if end_s is None else min(end_s, info.duration_s)
    t0 = start_s if match_start_s is None else match_start_s
    say(f"empty field from {start_s:.1f}-{end_s:.1f} s")
    bg = background(video, cam.size, start_s, end_s)
    bg_blur = cv2.GaussianBlur(bg, (5, 5), 0).astype(np.int16)
    say("occupancy grid")
    occ = Occupancy(cam, F)
    # only the field and what stands on it can hold robots
    hull = cv2.convexHull(np.float32(np.r_[cam.project(np.array([[0, 0, 0], [F, 0, 0], [F, F, 0], [0, F, 0]])),
                                           cam.project(np.array([[0, 0, ROBOT_H], [F, 0, ROBOT_H], [F, F, ROBOT_H], [0, F, ROBOT_H]]))]))
    field_mask = np.zeros(cam.size[::-1], np.uint8)
    cv2.fillConvexPoly(field_mask, np.int32(hull), 255)
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    tracks: list[Track] = []
    dead: list[Track] = []
    vw = None
    if overlay:
        vw = cv2.VideoWriter(str(overlay), cv2.VideoWriter_fourcc(*"mp4v"), fps, cam.size)
    nf = 0
    det_counts = []
    for t, f in iter_frames(video, width=cam.size[0], fps=fps, start_s=start_s, end_s=end_s):
        if f.shape[1] != cam.size[0] or f.shape[0] != cam.size[1]:
            f = cv2.resize(f, cam.size)
        nf += 1
        d = np.abs(cv2.GaussianBlur(f, (5, 5), 0).astype(np.int16) - bg_blur).max(axis=2)
        fg = ((d > diff_thr).astype(np.uint8) * 255) & field_mask
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, ker)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, ker)
        sc = occ.score(fg)
        dets = detect(sc, occ, n_robots + 2)
        det_counts.append(len(dets))
        # associate: distance, plus a penalty when a detection's plate colour disagrees with the track's
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        votes = [_colour_votes(hsv, fg, occ.box[k]) for (x, y, s, k) in dets]
        live = [tr for tr in tracks]

        def cost(tr, j):
            x, y = dets[j][:2]
            px, py = tr.predict(t)
            c = math.hypot(px - x, py - y)
            r, b = votes[j]
            if tr.alliance and r + b > 15:
                seen = "red" if r > b else "blue"
                c += 0.25 if seen != tr.alliance else 0.0
            return c

        C = np.array([[cost(tr, j) for j in range(len(dets))] for tr in live]) if live and dets else np.zeros((len(live), len(dets)))
        pairs = [(i, j) for i, j in _assign(np.where(C > 0.7, 1e6, C)) if C[i, j] <= 0.7] if C.size else []
        used_t, used_d = {i for i, _ in pairs}, {j for _, j in pairs}
        for i, j in pairs:
            x, y, s, k = dets[j]
            tr = live[i]
            tr.update(x, y, t)
            r, b = votes[j]
            # one vote per frame, so a pile of same-coloured blocks can't outweigh many frames of plates
            if r > max(8.0, 1.5 * b):
                tr.colour["red"] += 1
            elif b > max(8.0, 1.5 * r):
                tr.colour["blue"] += 1
        for i, tr in enumerate(live):
            if i not in used_t:
                tr.miss_s += 1.0 / fps
                px, py = tr.predict(t)
                tr.x, tr.y = min(max(px, 0.2), F - 0.2), min(max(py, 0.2), F - 0.2)
                tr.vx *= 0.7
                tr.vy *= 0.7
                tr.t = t
        for tr in [tr for tr in tracks if tr.miss_s > 4.0]:
            tracks.remove(tr)
            dead.append(tr)
        for j, (x, y, s, k) in enumerate(dets):
            if j in used_d or s < 0.40:
                continue
            if len(tracks) >= n_robots:
                # replace the track that has been lost the longest, if any
                lost = [tr for tr in tracks if tr.miss_s > 1.0]
                if not lost:
                    continue
                old = max(lost, key=lambda tr: tr.miss_s)
                tracks.remove(old)
                dead.append(old)
            tracks.append(Track(x, y, t))
        for tr in tracks:
            if tr.hits >= 3:
                tr.rows.append((round(t - t0, 3), tr.x, tr.y, tr.miss_s == 0))
        if vw is not None:
            img = f.copy()
            img[fg > 0] = (img[fg > 0] * 0.6 + np.array([0, 90, 0])).astype(np.uint8)
            for tr in tracks:
                u, v = cam.project(np.array([[tr.x, tr.y, 0.0]]))[0]
                col = (40, 40, 230) if tr.alliance == "red" else (230, 120, 30) if tr.alliance == "blue" else (230, 230, 230)
                cv2.circle(img, (int(u), int(v)), 9, col, 2 if tr.miss_s else -1)
                cv2.putText(img, str(tr.id), (int(u) + 10, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            vw.write(img)
        if nf % 100 == 0:
            say(f"  {t:.1f} s: {len(tracks)} tracks")
    if vw is not None:
        vw.release()
    allt = [tr for tr in dead + tracks if len(tr.rows) >= 5]
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "track_id", "x_m", "y_m", "alliance", "observed"])
        for tr in allt:
            for t, x, y, seen in tr.rows:
                w.writerow([f"{t:.3f}", tr.id, f"{x:.4f}", f"{y:.4f}", tr.alliance or "", int(seen)])
    return {"tracks_csv": str(out_csv), "frames": nf, "tracks": len(allt), "fps": fps, "frame_size": list(cam.size),
            "mean_detections": round(float(np.mean(det_counts)), 2) if det_counts else 0,
            "alliances": {str(tr.id): tr.alliance for tr in allt}}
