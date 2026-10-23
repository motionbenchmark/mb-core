"""Video reading for the tracker: upright frames with real timestamps.

Phone videos are often stored sideways with a rotation flag and recorded at a
variable frame rate. ``iter_frames`` applies the rotation, reads each frame's
own timestamp (instead of assuming ``index / fps``), optionally downsamples to a
target rate and resizes to a working width.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np


def _cv2():
    try:
        import cv2  # noqa: F401
    except ImportError as e:  # pragma: no cover
        raise ImportError("the tracker needs OpenCV: pip install 'motionbench[tracker]'") from e
    import cv2

    return cv2


@dataclass
class VideoInfo:
    path: str
    width: int  # upright, after rotation
    height: int
    fps: float  # nominal
    frames: int  # nominal count from the container
    duration_s: float
    rotation_deg: float


def probe(path: "str | Path") -> VideoInfo:
    cv2 = _cv2()
    cap = _open(cv2, path)
    try:
        ok, f = cap.read()
        if not ok:
            raise ValueError(f"cannot read frames from {path}")
        h, w = f.shape[:2]
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        rot = float(cap.get(cv2.CAP_PROP_ORIENTATION_META) or 0.0) if hasattr(cv2, "CAP_PROP_ORIENTATION_META") else 0.0
        return VideoInfo(str(path), w, h, float(fps), n, n / fps if fps else 0.0, rot)
    finally:
        cap.release()


def _open(cv2, path):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"no such video: {path}")
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video {path}")
    if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 1)  # apply the phone's rotation flag
    return cap


def iter_frames(
    path: "str | Path",
    *,
    width: Optional[int] = None,
    fps: Optional[float] = None,
    start_s: float = 0.0,
    end_s: Optional[float] = None,
) -> Iterator[tuple[float, np.ndarray]]:
    """Yield ``(t_seconds, BGR frame)``.

    ``width`` resizes frames (keeping aspect); ``fps`` keeps at most that many
    frames per second, chosen by timestamp so variable-rate video stays even.
    """
    cv2 = _cv2()
    cap = _open(cv2, path)
    nominal = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = 1.0 / fps if fps else 0.0
    next_t = start_s
    i = -1
    try:
        while True:
            ok, f = cap.read()
            if not ok:
                return
            i += 1
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if t <= 0 and i > 0:  # backend without timestamps
                t = i / nominal
            if t < start_s - 1e-6:
                continue
            if end_s is not None and t > end_s:
                return
            if step and t + 1e-6 < next_t:
                continue
            next_t = (next_t + step) if step else t
            if step and next_t < t:  # dropped frames in the source: resync
                next_t = t + step
            if width and f.shape[1] != width:
                h = int(round(f.shape[0] * width / f.shape[1]))
                f = cv2.resize(f, (width, h), interpolation=cv2.INTER_AREA)
            yield t, f
    finally:
        cap.release()
