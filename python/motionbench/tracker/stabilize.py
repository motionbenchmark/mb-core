"""Remove hand-held camera shake: align every frame to a reference frame.

Uses ORB features and a RANSAC homography. For a phone held roughly in place
the motion is mostly rotation, which a homography models exactly for every
plane in view; the moving robot is a small minority of features and is
rejected as outliers.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .video import _cv2


class Stabilizer:
    """``model``: "similarity" (rotation, scale, shift: robust, the default for a
    hand-held phone) or "homography" (full perspective, for larger camera motion)."""

    def __init__(self, reference: np.ndarray, n_features: int = 3000, model: str = "similarity"):
        self.model = model
        cv2 = _cv2()
        self.cv2 = cv2
        self.orb = cv2.ORB_create(n_features)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        g = self._gray(reference)
        self.k0, self.d0 = self.orb.detectAndCompute(g, None)
        self.shape = reference.shape[:2]
        self.last = np.eye(3)

    def _gray(self, f):
        return f if f.ndim == 2 else self.cv2.cvtColor(f, self.cv2.COLOR_BGR2GRAY)

    def homography(self, frame: np.ndarray) -> np.ndarray:
        """Homography mapping ``frame`` pixels onto the reference frame."""
        cv2 = self.cv2
        k, d = self.orb.detectAndCompute(self._gray(frame), None)
        if d is None or self.d0 is None or len(k) < 12:
            return self.last
        m = sorted(self.bf.match(d, self.d0), key=lambda x: x.distance)[:800]
        if len(m) < 12:
            return self.last
        p = np.float32([k[x.queryIdx].pt for x in m])
        q = np.float32([self.k0[x.trainIdx].pt for x in m])
        if self.model == "homography":
            H, inl = cv2.findHomography(p, q, cv2.RANSAC, 2.0)
        else:
            A, inl = cv2.estimateAffinePartial2D(p, q, method=cv2.RANSAC, ransacReprojThreshold=2.0)
            H = None if A is None else np.vstack([A, [0, 0, 1]])
        if H is None or inl is None or inl.sum() < 12:
            return self.last
        self.last = H
        return H

    def warp(self, frame: np.ndarray, H: Optional[np.ndarray] = None, border=(0, 0, 0)) -> np.ndarray:
        h, w = self.shape
        return self.cv2.warpPerspective(frame, self.homography(frame) if H is None else H, (w, h),
                                        borderMode=self.cv2.BORDER_CONSTANT, borderValue=border)

    def corner_shift(self, H: np.ndarray) -> float:
        """Largest displacement of a frame corner under ``H`` (pixels): how much the camera moved."""
        h, w = self.shape
        c = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]]).reshape(-1, 1, 2)
        return float(np.abs(self.cv2.perspectiveTransform(c, H) - c).max())
