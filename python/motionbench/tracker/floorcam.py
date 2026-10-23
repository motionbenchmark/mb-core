"""Single-view floor geometry for a fixed (or stabilized) camera, without tags.

The floor plane is recovered from four image points that outline a rectangle on
the floor (a hallway, a taped area, a field): its two long sides meet at a
vanishing point that gives the floor's direction and tilt, and its far edge
gives the cross direction. What a single image cannot give, the focal length and
the scale, comes from measurements (``measured_width_m`` / ``measured_length_m``)
or from the robot itself, whose known size must match its silhouettes.

World frame (metres): origin on the floor below the camera, +x across the
rectangle (left to right in the image), +y along it (away from the camera),
+z up.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def _h(p) -> np.ndarray:
    return np.array([p[0], p[1], 1.0])


class FloorCam:
    def __init__(self, f: float, vp_h: Sequence[float], back_dir_line: Sequence[float], h: float = 1.0,
                 c: tuple[float, float] = (0.0, 0.0)):
        """``vp_h``: homogeneous vanishing point of the floor's +y direction;
        ``back_dir_line``: homogeneous image line of a floor line along +x."""
        self.f, self.c = float(f), (float(c[0]), float(c[1]))
        self.K = np.array([[f, 0, c[0]], [0, f, c[1]], [0, 0, 1.0]])
        self.Ki = np.linalg.inv(self.K)
        r2 = self.Ki @ np.asarray(vp_h, float)
        r2 /= np.linalg.norm(r2)
        if r2[2] < 0 or (abs(r2[2]) < 1e-9 and r2[1] > 0):
            r2 = -r2
        n = self.K.T @ np.asarray(back_dir_line, float)
        n /= np.linalg.norm(n)
        r1 = np.cross(r2, n)
        r1 /= np.linalg.norm(r1)
        if r1[0] < 0:
            r1 = -r1
        r1 -= (r1 @ r2) * r2
        r1 /= np.linalg.norm(r1)
        up = np.cross(r1, r2)
        if up[1] > 0:  # image y points down, so "up" has negative camera-y
            up = -up
        self.R = np.stack([r1, r2, up], 1)
        self.h = float(h)

    # -- construction from four floor corners -------------------------------------------------
    @staticmethod
    def corner_lines(corners: Sequence[Sequence[float]]):
        """corners: far-left, far-right, near-right, near-left (image px)."""
        fl, fr, nr, nl = (_h(p) for p in corners)
        left, right = np.cross(nl, fl), np.cross(nr, fr)
        vp = np.cross(left, right)  # homogeneous; may be at infinity
        back = np.cross(fl, fr)
        return vp, back

    @classmethod
    def from_corners(cls, corners, f: float, h: float, size: tuple[int, int]) -> "FloorCam":
        vp, back = cls.corner_lines(corners)
        return cls(f, vp, back, h, (size[0] / 2.0, size[1] / 2.0))

    # -- geometry -------------------------------------------------------------------------------
    def to_floor(self, px: float, py: float) -> Optional[np.ndarray]:
        d = self.R.T @ (self.Ki @ np.array([px, py, 1.0]))
        if d[2] >= -1e-9:
            return None  # at or above the horizon
        t = -self.h / d[2]
        return np.array([t * d[0], t * d[1]])

    def to_floor_many(self, pts: np.ndarray) -> np.ndarray:
        pts = np.atleast_2d(pts).astype(float)
        d = (self.R.T @ (self.Ki @ np.c_[pts, np.ones(len(pts))].T)).T
        t = -self.h / np.where(d[:, 2] < -1e-9, d[:, 2], np.nan)
        return np.c_[t * d[:, 0], t * d[:, 1]]

    def project(self, P: np.ndarray) -> np.ndarray:
        P = np.atleast_2d(P).astype(float)
        if P.shape[1] == 2:
            P = np.c_[P, np.zeros(len(P))]
        Xc = self.R @ (P - np.array([0.0, 0.0, self.h])).T
        p = self.K @ Xc
        return (p[:2] / p[2]).T

    def tilt_deg(self) -> float:
        """Angle of the optical axis below the horizon."""
        return float(np.degrees(np.arcsin(np.clip(-(self.R[:, 2] @ np.array([0, 0, 1.0])), -1, 1))))

    def to_dict(self) -> dict:
        return {"f_px": self.f, "camera_height_m": self.h, "principal_point_px": list(self.c), "R": self.R.tolist()}
