from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

from ..trace import write_trace
from .field import FieldConfig


def _cv2():
    try:
        import cv2  # noqa: F401
    except ImportError as e:  # pragma: no cover
        raise ImportError("the tracker needs OpenCV: pip install 'motionbench[tracker]'") from e
    if not hasattr(cv2, "aruco"):
        raise ImportError("OpenCV was built without the aruco module; install opencv-contrib-python-headless")
    return cv2


def _detector(cv2, family: str):
    if family != "tag36h11":
        raise ValueError("only tag36h11 is supported in v0.1")
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, params)


@dataclass
class TrackResult:
    fps: float
    n_frames: int
    #: (t_s, x_m, y_m, theta_rad) for frames where the robot tag was found
    poses: list[tuple[float, float, float, float]] = field(default_factory=list)
    #: frames where all reference tags were visible and a homography was fitted
    calibrated_frames: int = 0
    #: RMS reprojection error of the homography on the reference tag corners (m)
    calibration_rms_m: Optional[float] = None
    warnings: list[str] = field(default_factory=list)

    @property
    def detection_rate(self) -> float:
        return len(self.poses) / self.n_frames if self.n_frames else 0.0

    def write(self, out_dir: "str | Path", *, task: str, seed: str, agent_id: str, agent_class: str, **kw) -> Path:
        """Write a `.mbtrace` with capture="video" and tracker diagnostics in the notes."""
        notes = (
            f"overhead video tracker v0.1: {self.n_frames} frames at {self.fps:.2f} fps, "
            f"robot detected in {self.detection_rate:.1%}, calibration RMS "
            f"{(self.calibration_rms_m or float('nan')) * 100:.1f} cm"
        )
        kw.setdefault("tier", "F")
        return write_trace(out_dir, task=task, seed=seed, agent_id=agent_id, agent_class=agent_class,
                           poses=self.poses, capture="video", notes=notes, **kw)


def _fit_homography(cv2, img_pts, field_pts):
    H, _ = cv2.findHomography(np.asarray(img_pts, np.float64), np.asarray(field_pts, np.float64), 0)
    if H is None:
        return None, None
    proj = cv2.perspectiveTransform(np.asarray(img_pts, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((proj - np.asarray(field_pts)) ** 2, axis=1))))
    return H, rms


def track_frames(frames: Iterable[np.ndarray], cfg: FieldConfig, fps: float, *,
                 max_gap_s: float = 0.25, recalibrate: bool = True) -> TrackResult:
    """Track the robot through an iterable of BGR or grayscale frames.

    The camera is assumed static. A homography (image -> field) is fitted from the
    reference tag corners on the first frame where all of them are visible and,
    with ``recalibrate``, refined whenever a better fit appears. Gaps in robot
    detection up to ``max_gap_s`` are linearly interpolated; longer gaps are left
    as gaps and reported.
    """
    cv2 = _cv2()
    det = _detector(cv2, cfg.family)
    res = TrackResult(fps=fps, n_frames=0)
    H = None
    best_rms = math.inf
    raw: list[tuple[float, float, float, float]] = []
    need = set(cfg.corner_tags)

    for i, frame in enumerate(frames):
        res.n_frames += 1
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = det.detectMarkers(gray)
        if ids is None:
            continue
        found = {int(k): c.reshape(4, 2) for k, c in zip(ids.flatten(), corners)}

        if need.issubset(found) and (H is None or recalibrate):
            img_pts, field_pts = [], []
            for tid in sorted(need):
                img_pts += found[tid].tolist()
                field_pts += cfg.tag_corners_field(tid)
            H_new, rms = _fit_homography(cv2, img_pts, field_pts)
            if H_new is not None:
                res.calibrated_frames += 1
                if rms < best_rms:
                    H, best_rms = H_new, rms

        if H is None or cfg.robot_tag_id not in found:
            continue
        c = found[cfg.robot_tag_id]
        center = c.mean(axis=0)
        top_mid = (c[0] + c[1]) / 2
        pts = cv2.perspectiveTransform(np.array([[center, top_mid]], np.float64), H)[0]
        (x, y), (tx, ty) = pts
        if cfg.camera_xyz_m and cfg.robot_tag_height_m > 0:
            # The tag sits above the floor, so its floor-plane projection is pushed
            # away from the camera. Pull it back along the ray: p = cam + (p' - cam)(1 - h/H).
            cx, cy, cz = cfg.camera_xyz_m
            k = 1 - cfg.robot_tag_height_m / cz
            x, y = cx + (x - cx) * k, cy + (y - cy) * k
            tx, ty = cx + (tx - cx) * k, cy + (ty - cy) * k
        theta = math.atan2(ty - y, tx - x) + cfg.robot_tag_yaw_offset
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        raw.append((i / fps, float(x), float(y), theta))

    res.calibration_rms_m = None if H is None else best_rms
    if H is None:
        res.warnings.append("reference tags never all visible: no calibration, no poses")
        return res
    res.poses, gaps = _fill_gaps(raw, fps, max_gap_s)
    if gaps:
        res.warnings.append(f"{gaps} detection gaps longer than {max_gap_s:.2f} s left unfilled")
    if res.detection_rate < 0.9:
        res.warnings.append(f"robot tag found in only {res.detection_rate:.0%} of frames")
    return res


def _fill_gaps(raw, fps, max_gap_s):
    if not raw:
        return [], 0
    out = [raw[0]]
    long_gaps = 0
    dt = 1 / fps
    for a, b in zip(raw, raw[1:]):
        gap = b[0] - a[0]
        if gap > 1.5 * dt:
            if gap <= max_gap_s + 1e-9:
                n = int(round(gap / dt))
                dth = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
                for k in range(1, n):
                    f = k / n
                    out.append((a[0] + gap * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, a[3] + dth * f))
            else:
                long_gaps += 1
        out.append(b)
    return out, long_gaps


def track_video(path: "str | Path", cfg: FieldConfig, **kw) -> TrackResult:
    """Track a video file. See :func:`track_frames` for options."""
    cv2 = _cv2()
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    def frames():
        while True:
            ok, f = cap.read()
            if not ok:
                return
            yield f

    try:
        return track_frames(frames(), cfg, fps, **kw)
    finally:
        cap.release()
