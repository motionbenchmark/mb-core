"""Printable tag sheets for the overhead tracker."""

from __future__ import annotations

from pathlib import Path

from .field import FieldConfig
from .track import _cv2

DPI = 300


def write_tags(cfg: FieldConfig, out_dir: "str | Path") -> list[Path]:
    """Write one PNG per tag at print size (300 dpi), with a white border and a label.
    Print at 100% scale and check the black square measures the configured size."""
    cv2 = _cv2()
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    jobs = [(tid, cfg.tag_size_m, f"reference tag {tid} - top edge faces far (+y) wall") for tid in sorted(cfg.corner_tags)]
    jobs.append((cfg.robot_tag_id, cfg.robot_tag_size_m, f"robot tag {cfg.robot_tag_id} - top edge faces robot front"))
    for tid, size_m, label in jobs:
        px = int(round(size_m / 0.0254 * DPI))
        img = cv2.aruco.generateImageMarker(d, tid, px)
        pad = px // 4
        img = cv2.copyMakeBorder(img, pad, pad + 60, pad, pad, cv2.BORDER_CONSTANT, value=255)
        cv2.putText(img, f"{label} | {size_m * 100:.0f} cm | tag36h11", (pad, img.shape[0] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
        p = out / f"tag36h11-{tid:02d}.png"
        cv2.imwrite(str(p), img)
        paths.append(p)
    cfg.save(out / "field.json")
    return paths
