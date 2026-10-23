"""Track a robot in an ordinary phone video, without tags.

    from motionbench.tracker.untagged import Calibration, track_untagged
    calib = Calibration.load("calib.json")          # from `motionbench calibrate`
    res = track_untagged("run.mov", calib, overlay="overlay.mp4")
    res.write("out/", task_id="MB-FIELD-HALLWAY", seed="field-01",
              agent_id="robot-vacuum", agent_class="product")

Pipeline
  1. Read upright frames with real timestamps (``video.iter_frames``).
  2. Align every frame to the first (``stabilize.Stabilizer``): hand-held is fine.
  3. Median of the aligned frames = the empty floor; the robot is whatever differs
     from it (darker, lighter or either, see ``robot_polarity``).
  4. Floor geometry from four tapped floor corners (``floorcam.FloorCam``). Focal
     length and scale come from tape measurements when given, otherwise from the
     robot's own size fitted to its silhouettes.
  5. The robot's centre is the lowest point of its silhouette (the near edge of its
     base, on the floor) plus half its diameter, away from the camera.
  6. Outlier jumps removed, gaps kept, positions smoothed, heading from travel.
  7. A coverage task is written for the visible floor: walls are obstacles, and
     floor the camera never saw goes into ``arena.unobserved``.

Accuracy depends on the calibration: with a measured width and length expect a
few centimetres; from the robot's size alone, a few percent of scale.
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .floorcam import FloorCam
from .stabilize import Stabilizer
from .video import _cv2, iter_frames, probe

CELL_M = 0.05


@dataclass
class Calibration:
    """What the tracker needs to know about one video. Coordinates are pixels in a
    frame of ``frame_size`` (the working resolution); they are rescaled if the
    video is processed at another size."""

    frame_size: list[int]  # [width, height] of the frame the points were tapped on
    floor_corners_px: list[list[float]]  # far-left, far-right, near-right, near-left
    floor_polygon_px: Optional[list[list[float]]] = None  # visible floor incl. nooks; default the corners
    measured_width_m: Optional[float] = None  # far-left -> far-right, by tape measure
    measured_length_m: Optional[float] = None  # near-left -> far-left
    robot_diameter_m: float = 0.35
    robot_height_m: float = 0.10
    robot_polarity: str = "dark"  # dark | light | any
    focal_px: Optional[float] = None
    max_y_px: Optional[float] = None  # ignore image rows below this (e.g. the photographer's feet)
    start_s: float = 0.0
    end_s: Optional[float] = None
    notes: str = ""

    @classmethod
    def load(cls, path: "str | Path") -> "Calibration":
        d = json.loads(Path(path).read_text())
        known = {k: d[k] for k in cls.__dataclass_fields__ if k in d}
        return cls(**known)

    def save(self, path: "str | Path") -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2))

    def scaled(self, size: tuple[int, int]) -> "Calibration":
        sx, sy = size[0] / self.frame_size[0], size[1] / self.frame_size[1]
        if abs(sx - 1) < 1e-9 and abs(sy - 1) < 1e-9:
            return self
        sc = lambda pts: None if pts is None else [[p[0] * sx, p[1] * sy] for p in pts]
        d = asdict(self)
        d.update(frame_size=list(size), floor_corners_px=sc(self.floor_corners_px), floor_polygon_px=sc(self.floor_polygon_px),
                 focal_px=None if self.focal_px is None else self.focal_px * sx,
                 max_y_px=None if self.max_y_px is None else self.max_y_px * sy)
        return Calibration(**d)

    def polygon(self) -> np.ndarray:
        return np.array(self.floor_polygon_px or self.floor_corners_px, np.float32)


@dataclass
class UntaggedResult:
    poses: list[tuple[float, float, float, float]]
    task: dict
    camera: dict
    stats: dict
    warnings: list[str] = field(default_factory=list)
    background: Optional[np.ndarray] = None

    def write(self, out_dir: "str | Path", *, task_id: Optional[str] = None, version: Optional[str] = None,
              seed: str = "field-01", agent_id: str = "robot", agent_class: str = "product", agent_name: str = "",
              agent_meta: Optional[dict] = None, notes: str = "") -> Path:
        """Write ``tasks/<ID@VERSION>.json``, ``trace/`` (.mbtrace), ``calibration.json`` and ``background.png``."""
        from ..trace import write_trace

        out = Path(out_dir)
        task = dict(self.task)
        if task_id:
            task["id"] = task_id
        if version:
            task["version"] = version
        if seed not in task.get("layouts", {}):
            task["layouts"] = {seed: next(iter(task["layouts"].values()))}
        key = f"{task['id']}@{task['version']}"
        (out / "tasks").mkdir(parents=True, exist_ok=True)
        (out / "tasks" / f"{key}.json").write_text(json.dumps(task, indent=1))
        meta = {"capture": "video-untagged", "camera": self.camera, "stats": self.stats, "synthetic": False}
        meta.update(agent_meta or {})
        write_trace(out / "trace", task=key, seed=seed, agent_id=agent_id, agent_class=agent_class,
                    agent_name=agent_name or agent_id, agent_meta=meta, trial=0, tier="F", capture="video-untagged",
                    poses=self.poses, events=[], world_final=None,
                    notes=(notes + " " if notes else "") + "Untagged single-camera track (motionbench.tracker.untagged). "
                    + " ".join(self.warnings))
        (out / "calibration.json").write_text(json.dumps({"camera": self.camera, "stats": self.stats, "warnings": self.warnings}, indent=2))
        if self.background is not None:
            _cv2().imwrite(str(out / "background.png"), self.background)
        return out


# ------------------------------------------------------------------------------------- helpers

def _savgol(y: np.ndarray, window: int = 9, order: int = 2) -> np.ndarray:
    """Savitzky-Golay smoothing (numpy only); short series are returned unchanged."""
    if len(y) < window:
        return y.copy()
    half = window // 2
    A = np.vander(np.arange(-half, half + 1), order + 1, increasing=True)
    coef = np.linalg.pinv(A)[0]
    pad = np.r_[y[half:0:-1], y, y[-2:-half - 2:-1]]
    return np.convolve(pad, coef[::-1], mode="valid")


def _runs(mask: np.ndarray, value, cell: float, label: Optional[str]) -> list[dict]:
    """Row-wise runs of ``mask == value`` as rectangles (metres)."""
    out = []
    ny, nx = mask.shape
    for j in range(ny):
        i = 0
        while i < nx:
            if mask[j, i] == value:
                k = i
                while k < nx and mask[j, k] == value:
                    k += 1
                r = {"x": round(i * cell, 3), "y": round(j * cell, 3), "w": round((k - i) * cell, 3), "h": cell}
                if label:
                    r["label"] = label
                out.append(r)
                i = k
            else:
                i += 1
    return out


def detect_robot(frame: np.ndarray, bg: np.ndarray, valid: np.ndarray, floor_mask: np.ndarray, polarity: str,
                 min_area: float) -> Optional[dict]:
    cv2 = _cv2()
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)
    bgg = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY).astype(np.int16)
    diff = cv2.absdiff(frame, bg).max(axis=2)
    if polarity == "dark":
        m = (bgg - g > 35) & (diff > 35)
    elif polarity == "light":
        m = (g - bgg > 35) & (diff > 35)
    else:
        m = diff > 40
    m = (m & valid).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best = None
    H, W = m.shape
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        pts = c.reshape(-1, 2)
        by = pts[:, 1].max()
        bottom = pts[pts[:, 1] >= by - 2]
        bx = float(bottom[:, 0].mean())
        if not floor_mask[min(H - 1, int(by)), min(W - 1, max(0, int(bx)))]:
            continue  # its lowest point is not on the floor: a person, a door, a shadow on the wall
        if best is None or a > best["area"]:
            x, y, bw, bh = cv2.boundingRect(c)
            below = valid[min(H - 1, y + bh + 2), min(W - 1, x + bw // 2)]
            touches = bool(x <= 1 or y <= 1 or x + bw >= W - 2 or y + bh >= H - 2 or not below)
            best = {"area": float(a), "bbox": [int(x), int(y), int(bw), int(bh)], "bottom": [bx, float(by)], "touches_edge": touches}
    return best


def fit_camera(calib: Calibration, dets: list[dict], size: tuple[int, int]) -> tuple[FloorCam, dict]:
    """Choose focal length and camera height. See the module docstring."""
    W, H = size
    corners = np.array(calib.floor_corners_px, float)
    D, HC = calib.robot_diameter_m, calib.robot_height_m
    fs = [calib.focal_px] if calib.focal_px else list(np.geomspace(0.45, 1.6, 64) * max(W, H))

    def widths(cam1):
        w = cam1.to_floor_many(corners)  # at h = 1
        width = (np.linalg.norm(w[1] - w[0]) + np.linalg.norm(w[2] - w[3])) / 2
        length = (np.linalg.norm(w[0] - w[3]) + np.linalg.norm(w[1] - w[2])) / 2
        return width, length

    obs = [d for d in dets if d and not d["touches_edge"]]
    if len(obs) > 240:
        obs = obs[:: max(1, len(obs) // 240)]
    ang = np.linspace(0, 2 * np.pi, 32, endpoint=False)
    cos, sin = np.cos(ang), np.sin(ang)

    def robot_err(cam: FloorCam) -> float:
        if not obs:
            return math.inf
        bott = np.array([d["bottom"] for d in obs])
        pb = cam.to_floor_many(bott)
        ok = np.isfinite(pb).all(1)
        if ok.sum() < 5:
            return math.inf
        pb = pb[ok]
        u = pb / np.linalg.norm(pb, axis=1, keepdims=True)
        c = pb + u * D / 2
        n = len(c)
        ring = np.stack([c[:, 0, None] + D / 2 * cos, c[:, 1, None] + D / 2 * sin], -1).reshape(-1, 2)
        P = np.r_[np.c_[ring, np.zeros(len(ring))], np.c_[ring, np.full(len(ring), HC)]]
        q = cam.project(P).reshape(2, n, len(ang), 2)
        q = np.concatenate([q[0], q[1]], axis=1)
        pw = q[:, :, 0].max(1) - q[:, :, 0].min(1)
        ph = q[:, :, 1].max(1) - q[:, :, 1].min(1)
        bb = np.array([d["bbox"] for d, k in zip(obs, ok) if k], float)
        e = np.log(np.clip(pw, 1e-3, None) / bb[:, 2]) ** 2 + np.log(np.clip(ph, 1e-3, None) / bb[:, 3]) ** 2
        return float(np.sqrt(np.median(e)))

    mw, ml = calib.measured_width_m, calib.measured_length_m
    best = None
    if mw and ml and not calib.focal_px:
        mode = "measured"  # tape-measured rectangle: focal from its aspect ratio, scale from its width
        for f in fs:
            cam1 = FloorCam.from_corners(corners, f, 1.0, size)
            w, l = widths(cam1)
            if not (np.isfinite(w) and np.isfinite(l)) or w <= 0:
                continue
            err = abs(math.log((l / w) / (ml / mw)))
            if best is None or err < best[0]:
                best = (err, f, mw / w)
        # refine around the best focal length
        if best:
            for f in np.geomspace(best[1] * 0.97, best[1] * 1.03, 41):
                cam1 = FloorCam.from_corners(corners, f, 1.0, size)
                w, l = widths(cam1)
                err = abs(math.log((l / w) / (ml / mw)))
                if err < best[0]:
                    best = (err, f, mw / w)
    elif mw or ml:
        mode = "robot+measured"  # one measured length fixes the scale, the robot fixes the focal length
        for f in fs:
            cam1 = FloorCam.from_corners(corners, f, 1.0, size)
            w, l = widths(cam1)
            hh = (mw / w) if mw else (ml / l)
            if not np.isfinite(hh) or hh <= 0:
                continue
            e = robot_err(FloorCam.from_corners(corners, f, hh, size))
            if best is None or e < best[0]:
                best = (e, f, hh)
    else:
        mode = "robot-size"  # the robot is the only ruler
        for f in fs:
            for hh in np.geomspace(0.4, 4.0, 60):
                e = robot_err(FloorCam.from_corners(corners, f, hh, size))
                if best is None or e < best[0]:
                    best = (e, f, hh)
        if best:  # refine
            f0, h0 = best[1], best[2]
            for f in np.geomspace(f0 * 0.96, f0 * 1.04, 9):
                for hh in np.geomspace(h0 * 0.96, h0 * 1.04, 9):
                    e = robot_err(FloorCam.from_corners(corners, f, hh, size))
                    if e < best[0]:
                        best = (e, f, hh)
    if best is None:
        raise ValueError("could not calibrate: check the floor corners (far-left, far-right, near-right, near-left)")
    cam = FloorCam.from_corners(corners, best[1], best[2], size)
    w, l = widths(FloorCam.from_corners(corners, best[1], 1.0, size))
    info = {"mode": mode, "fit_error": None if not np.isfinite(best[0]) else round(float(best[0]), 4),
            "robot_size_error_rms_log": round(robot_err(cam), 4) if obs else None,
            "rectangle_m": [round(w * best[2], 3), round(l * best[2], 3)], "silhouettes_used": len(obs)}
    return cam, info


def _arena_frame(cam: FloorCam, calib: Calibration):
    """Arena frame: x along near-left -> near-right, y towards the far edge, origin so the
    floor polygon starts at (0, 0)."""
    c = cam.to_floor_many(np.array(calib.floor_corners_px, float))
    nl, nr = c[3], c[2]
    ex = (nr - nl) / np.linalg.norm(nr - nl)
    ey = np.array([-ex[1], ex[0]])
    if (c[0] - nl) @ ey < 0:
        ey = -ey
    poly = cam.to_floor_many(calib.polygon())
    local = (poly - nl) @ np.stack([ex, ey], 1)
    origin = nl + ex * local[:, 0].min() + ey * local[:, 1].min()
    size = (local[:, 0].max() - local[:, 0].min(), local[:, 1].max() - local[:, 1].min())
    to_arena = lambda P: (np.atleast_2d(P) - origin) @ np.stack([ex, ey], 1)
    to_world = lambda A: origin + np.atleast_2d(A) @ np.stack([ex, ey], 0)
    return to_arena, to_world, size


def _cells(cam: FloorCam, calib: Calibration, to_world, size_m, frame_size) -> np.ndarray:
    """0 = visible floor, 1 = wall (seen, not floor), 2 = not seen."""
    cv2 = _cv2()
    W, H = frame_size
    ny, nx = math.ceil(size_m[1] / CELL_M - 1e-9), math.ceil(size_m[0] / CELL_M - 1e-9)
    ii, jj = np.meshgrid(np.arange(nx), np.arange(ny))
    A = np.c_[(ii.ravel() + 0.5) * CELL_M, (jj.ravel() + 0.5) * CELL_M]
    px = cam.project(to_world(A))
    poly = calib.polygon().reshape(-1, 1, 2)
    ymax = calib.max_y_px if calib.max_y_px else H
    kind = np.empty(len(A), np.uint8)
    for k, (x, y) in enumerate(px):
        if not (np.isfinite(x) and 0 <= x < W and 0 <= y < ymax):
            kind[k] = 2
        else:
            kind[k] = 0 if cv2.pointPolygonTest(poly, (float(x), float(y)), False) >= 0 else 1
    return kind.reshape(ny, nx)


# ------------------------------------------------------------------------------------- main entry

def track_untagged(
    video: "str | Path",
    calib: Calibration,
    *,
    fps: float = 15.0,
    stabilize: bool = True,
    tool_width_m: float = 0.30,
    edge_reach_m: Optional[float] = None,
    contact_tolerance_m: Optional[float] = 0.08,
    max_speed_mps: float = 1.5,
    overlay: "str | Path | None" = None,
    progress: Optional[Callable[[str], None]] = None,
) -> UntaggedResult:
    cv2 = _cv2()
    say = progress or (lambda s: None)
    info = probe(video)
    width = int(calib.frame_size[0])
    size = (width, int(round(info.height * width / info.width)))
    calib = calib.scaled(size)
    frames = lambda: iter_frames(video, width=width, fps=fps, start_s=calib.start_s, end_s=calib.end_s)
    warnings: list[str] = []

    # pass 1: alignment + empty floor
    say("aligning frames")
    Hs, ts, samples, stab = [], [], [], None
    n_est = max(1, int((min(info.duration_s, calib.end_s or 1e9) - calib.start_s) * fps))
    every = max(1, n_est // 120)
    for i, (t, f) in enumerate(frames()):
        if stab is None:
            stab = Stabilizer(f) if stabilize else None
            ref_shape = f.shape
        Hm = stab.homography(f) if stab else np.eye(3)
        Hs.append(Hm)
        ts.append(t)
        if i % every == 0:
            samples.append(cv2.warpPerspective(f, Hm, (size[0], size[1])))
    if not ts:
        raise ValueError(f"no frames read from {video}")
    shift = max((stab.corner_shift(h) for h in Hs), default=0.0) if stab else 0.0
    bg = np.median(np.stack(samples), axis=0).astype(np.uint8)
    say(f"{len(ts)} frames, camera moved up to {shift:.0f} px")

    # pass 2: detect
    floor_mask = np.zeros((size[1], size[0]), np.uint8)
    cv2.fillPoly(floor_mask, [calib.polygon().astype(np.int32)], 255)
    floor_mask = cv2.dilate(floor_mask, np.ones((9, 9), np.uint8)) > 0
    if calib.max_y_px:
        floor_mask[int(calib.max_y_px):, :] = False
    min_area = 0.0003 * size[0] * size[1]
    dets = []
    say("finding the robot")
    for i, (t, f) in enumerate(frames()):
        if i >= len(Hs):
            break
        warped = cv2.warpPerspective(f, Hs[i], size)
        valid = cv2.warpPerspective(np.full(f.shape[:2], 255, np.uint8), Hs[i], size) > 0
        d = detect_robot(warped, bg, valid, floor_mask, calib.robot_polarity, min_area)
        if d:
            d["t"] = ts[i]
        dets.append(d)
    n_det = sum(1 for d in dets if d)
    n_edge = sum(1 for d in dets if d and d["touches_edge"])
    if n_det < 0.5 * len(dets):
        warnings.append(f"robot found in only {n_det} of {len(dets)} frames")

    # calibration
    say("calibrating the floor")
    cam, cam_info = fit_camera(calib, [d for d in dets if d], size)
    to_arena, to_world, arena_size = _arena_frame(cam, calib)
    D = calib.robot_diameter_m

    # positions
    T, X, Y = [], [], []
    for d in dets:
        if not d or d["touches_edge"]:
            continue
        pb = cam.to_floor(*d["bottom"])
        if pb is None:
            continue
        c = pb + pb / np.linalg.norm(pb) * D / 2
        a = to_arena(c)[0]
        T.append(d["t"]); X.append(a[0]); Y.append(a[1])
    T, X, Y = map(np.asarray, (T, X, Y))
    if len(T) < 5:
        raise ValueError("robot track too short; check robot_polarity and the floor outline")
    keep = np.ones(len(T), bool)
    last = 0
    for k in range(1, len(T)):  # drop single-frame jumps (shadows, glare)
        v = math.hypot(X[k] - X[last], Y[k] - Y[last]) / max(T[k] - T[last], 1e-6)
        if v > max_speed_mps:
            keep[k] = False
        else:
            last = k
    T, X, Y = T[keep], X[keep], Y[keep]
    seg = np.r_[0, np.cumsum(np.diff(T) > 0.4)]
    Xs, Ys = X.copy(), Y.copy()
    for s in np.unique(seg):
        m = seg == s
        Xs[m], Ys[m] = _savgol(X[m]), _savgol(Y[m])
    vx, vy = (np.gradient(Xs, T), np.gradient(Ys, T)) if len(T) > 1 else (np.zeros(1), np.zeros(1))
    th, heading = np.zeros(len(T)), math.pi / 2
    for k in range(len(T)):
        if math.hypot(vx[k], vy[k]) > 0.05:
            heading = math.atan2(vy[k], vx[k])
        th[k] = heading
    t0 = T[0]
    poses = [(round(float(t - t0), 4), round(float(x), 4), round(float(y), 4), round(float(h), 4)) for t, x, y, h in zip(T, Xs, Ys, th)]
    gaps = np.diff(T)
    if len(gaps) and gaps.max() > 0.5:
        warnings.append(f"{int((gaps > 0.5).sum())} gaps over 0.5 s (longest {gaps.max():.1f} s) where the robot was hidden or out of frame")

    # task
    kind = _cells(cam, calib, to_world, arena_size, size)
    aw = round(kind.shape[1] * CELL_M, 3)
    ah = round(kind.shape[0] * CELL_M, 3)
    free_m2 = float((kind == 0).sum()) * CELL_M * CELL_M
    robot = {"radius_m": round(D / 2, 4)}
    if contact_tolerance_m is not None:
        robot["contact_tolerance_m"] = contact_tolerance_m
    goal = {"kind": "coverage", "cell_m": CELL_M, "tool_width_m": tool_width_m}
    if edge_reach_m:
        goal["edge_reach_m"] = edge_reach_m
    w_m, l_m = cam_info["rectangle_m"]
    task = {
        "id": "MB-FIELD-CAPTURE", "version": "0.1",
        "title": "Floor coverage (field capture, single camera, no tags)",
        "domain": "indoor-floor",
        "description": (f"Visible floor of a {w_m:.2f} m x {l_m:.2f} m area measured from one phone video without tags "
                        f"(calibration: {cam_info['mode']}). Walls are obstacles; floor the camera never saw is unobserved. "
                        f"Tool width {tool_width_m*100:.0f} cm."),
        "arena": {"width_m": aw, "height_m": ah, "obstacles": _runs(kind, 1, CELL_M, "wall"), "zones": {},
                  "unobserved": _runs(kind, 2, CELL_M, None)},
        "robot": robot,
        "budget": {"time_s": round(poses[-1][0] + 0.5, 1)},
        "goal": goal,
        "reference": {"floor": 0},
        "layouts": {"field-01": {"start": {"x": poses[0][1], "y": poses[0][2], "theta": poses[0][3]}}},
    }
    stats = {"frames": len(dets), "robot_found": n_det, "cut_off_by_frame_edge": n_edge, "poses": len(poses),
             "duration_s": poses[-1][0], "camera_shift_px": round(shift, 1), "visible_floor_m2": round(free_m2, 2),
             "frame_size": list(size), "fps": fps, "track_start_s": round(float(t0), 4)}
    camera = {**cam.to_dict(), **cam_info, "tilt_deg": round(cam.tilt_deg(), 1), "robot_diameter_m": D,
              "robot_height_m": calib.robot_height_m}
    if cam_info["mode"] == "robot-size":
        warnings.append("scale estimated from the robot's size; add a measured width for exact numbers")
    res = UntaggedResult(poses, task, camera, stats, warnings, bg)
    if overlay:
        say("drawing the overlay")
        write_overlay(video, calib, Hs, ts, res, cam, to_world, overlay, fps=fps, width=width)
    return res


def write_overlay(video, calib: Calibration, Hs, ts, res: UntaggedResult, cam: FloorCam, to_world, out,
                  *, fps: float, width: int) -> Path:
    """QA video: stabilized frames with the tracked footprint, trail and floor outline drawn on."""
    cv2 = _cv2()
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name(out.stem + ".raw.mp4")
    P = np.array(res.poses)
    t_start = res.stats["track_start_s"]
    D = calib.robot_diameter_m
    ang = np.linspace(0, 2 * np.pi, 40)
    vw, trail = None, []
    for i, (t, f) in enumerate(iter_frames(video, width=width, fps=fps, start_s=calib.start_s, end_s=calib.end_s)):
        if i >= len(Hs):
            break
        h, w = f.shape[:2]
        if vw is None:
            vw = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        g = cv2.warpPerspective(f, Hs[i], (w, h))
        rel = t - t_start
        k = int(np.clip(np.searchsorted(P[:, 0], rel), 0, len(P) - 1))
        if abs(P[k, 0] - rel) < 0.15:
            wc = to_world(P[k, 1:3])[0]
            ring = cam.project(np.c_[wc[0] + D / 2 * np.cos(ang), wc[1] + D / 2 * np.sin(ang), np.zeros(40)]).astype(np.int32)
            cv2.polylines(g, [ring], True, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.polylines(g, [ring], True, (91, 63, 210), 2, cv2.LINE_AA)
            trail.append(tuple(int(v) for v in cam.project(np.array([[wc[0], wc[1], 0.0]]))[0]))
        if len(trail) > 1:
            cv2.polylines(g, [np.array(trail, np.int32)], False, (91, 63, 210), 2, cv2.LINE_AA)
        cv2.polylines(g, [calib.polygon().astype(np.int32)], True, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(g, f"t={t:5.1f}s", (10, 24), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 2)
        vw.write(g)
    if vw:
        vw.release()
    if shutil.which("ffmpeg"):
        r = subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(raw), "-c:v", "libx264", "-crf", "23",
                            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], check=False)
        if r.returncode == 0 and out.exists():
            raw.unlink(missing_ok=True)
            return out
    raw.replace(out)
    return out
