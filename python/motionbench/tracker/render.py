"""Render synthetic overhead-camera video of a run, for testing the tracker
without a real field. The field is drawn top-down, then warped by a perspective
transform that mimics a phone on a tripod at the field edge."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterator, Optional, Sequence

import numpy as np

from .field import FieldConfig
from .track import _cv2


def _tag_image(cv2, tag_id: int, px: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(d, tag_id, px)
    # white quiet zone (1/4 of the tag) so detection is reliable
    pad = max(2, px // 8)
    return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def render_frames(
    poses: Sequence[Sequence[float]],
    cfg: FieldConfig,
    *,
    fps: float = 30.0,
    px_per_m: float = 260.0,
    out_size: tuple[int, int] = (1280, 960),
    tilt: float = 0.18,
    noise: float = 4.0,
    seed: int = 0,
) -> Iterator[np.ndarray]:
    """Yield BGR frames at ``fps`` for a pose sequence ``(t, x, y, theta)``,
    linearly interpolated. ``tilt`` (0..0.4) controls the camera perspective."""
    cv2 = _cv2()
    rng = np.random.default_rng(seed)
    W = int(cfg.width_m * px_per_m)
    Hh = int(cfg.height_m * px_per_m)
    margin = 60

    def to_px(x, y):
        return margin + x * px_per_m, margin + (cfg.height_m - y) * px_per_m

    base = np.full((Hh + 2 * margin, W + 2 * margin, 3), (95, 105, 95), np.uint8)
    cv2.rectangle(base, (margin, margin), (margin + W, margin + Hh), (70, 80, 70), -1)
    for k in range(1, 6):  # foam tile seams
        x = margin + int(W * k / 6)
        y = margin + int(Hh * k / 6)
        cv2.line(base, (x, margin), (x, margin + Hh), (60, 70, 60), 1)
        cv2.line(base, (margin, y), (margin + W, y), (60, 70, 60), 1)

    tpx = int(cfg.tag_size_m * px_per_m)
    for tid, (cx, cy) in cfg.corner_tags.items():
        img = _tag_image(cv2, tid, tpx)
        pad = (img.shape[0] - tpx) // 2
        px, py = to_px(cx, cy)
        x0, y0 = int(round(px - tpx / 2)) - pad, int(round(py - tpx / 2)) - pad
        base[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img[..., None]

    rpx = int(cfg.robot_tag_size_m * px_per_m)
    rtag = _tag_image(cv2, cfg.robot_tag_id, rpx)
    body = int(0.40 * px_per_m)

    # Camera: a trapezoid warp, far edge narrower, as from a raised tripod.
    h0, w0 = base.shape[:2]
    src = np.float32([[0, 0], [w0, 0], [w0, h0], [0, h0]])
    ow, oh = out_size
    dst = np.float32([[ow * tilt, oh * 0.06], [ow * (1 - tilt), oh * 0.06], [ow * 0.98, oh * 0.97], [ow * 0.02, oh * 0.97]])
    P = cv2.getPerspectiveTransform(src, dst)

    ts = np.array([p[0] for p in poses])
    t_end = ts[-1]
    n = int(t_end * fps) + 1
    for i in range(n):
        t = i / fps
        j = int(np.searchsorted(ts, t, side="right") - 1)
        j = min(max(j, 0), len(poses) - 2)
        a, b = poses[j], poses[j + 1]
        f = 0.0 if b[0] == a[0] else min(max((t - a[0]) / (b[0] - a[0]), 0.0), 1.0)
        x = a[1] + (b[1] - a[1]) * f
        y = a[2] + (b[2] - a[2]) * f
        dth = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
        th = a[3] + dth * f

        frame = base.copy()
        px, py = to_px(x, y)
        # In image coords y points down, so a field heading of th is -th on screen.
        box = cv2.boxPoints(((px, py), (body, body), -math.degrees(th)))
        cv2.fillConvexPoly(frame, box.astype(np.int32), (40, 40, 160))
        # Tag top must face the robot's front (field heading th): the tag image's top
        # is screen-up (field +y, heading pi/2), so rotate by th - pi/2.
        M = cv2.getRotationMatrix2D((rtag.shape[1] / 2, rtag.shape[0] / 2), math.degrees(th - math.pi / 2), 1.0)
        size = int(rtag.shape[0] * 1.5)
        M[0, 2] += (size - rtag.shape[1]) / 2
        M[1, 2] += (size - rtag.shape[0]) / 2
        rot = cv2.warpAffine(rtag, M, (size, size), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpAffine(np.full_like(rtag, 255), M, (size, size), flags=cv2.INTER_NEAREST, borderValue=0)
        x0, y0 = int(round(px - size / 2)), int(round(py - size / 2))
        roi = frame[y0:y0 + size, x0:x0 + size]
        if roi.shape[:2] == (size, size):
            m = mask > 0
            roi[m] = rot[m][..., None]

        out = cv2.warpPerspective(frame, P, out_size, flags=cv2.INTER_LINEAR)
        if noise > 0:
            out = np.clip(out.astype(np.int16) + rng.normal(0, noise, out.shape).astype(np.int16), 0, 255).astype(np.uint8)
        yield out


def render_video(poses, cfg: FieldConfig, out_path: "str | Path", fps: float = 30.0, **kw) -> Path:
    """Write :func:`render_frames` to a video file (MJPG .avi is the most portable)."""
    cv2 = _cv2()
    out_path = Path(out_path)
    writer: Optional[object] = None
    for frame in render_frames(poses, cfg, fps=fps, **kw):
        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*("MJPG" if out_path.suffix == ".avi" else "mp4v"))
            writer = cv2.VideoWriter(str(out_path), fourcc, fps, (frame.shape[1], frame.shape[0]))
        writer.write(frame)
    if writer is not None:
        writer.release()
    return out_path
