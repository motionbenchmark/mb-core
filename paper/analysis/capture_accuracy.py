"""Capture accuracy on rendered video with known ground truth (synthetic only).

For each condition and render seed: per-frame position error against the true path,
reported as median, 90th percentile and maximum, plus the share of frames with no
position (failure rate). Writes paper/results/capture-accuracy.json.
"""
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

from motionbench.tracker import FieldConfig, track_frames
from motionbench.tracker.render import render_floor_video, render_frames
from motionbench.tracker.untagged import Calibration, track_untagged

OUT = Path(__file__).resolve().parents[1] / "results" / "capture-accuracy.json"
SEEDS = [0, 1, 2]


def lawnmower(x0=-0.4, x1=0.4, y_start=1.0, y_end=2.4, lane=0.35, v=0.3):
    t, y, left = 0.0, y_start, True
    path, cur = [(0.0, x0, y_start)], (x0, y_start)
    while y < y_end:
        nx = x1 if left else x0
        t += abs(nx - cur[0]) / v
        cur = (nx, y)
        path.append((t, *cur))
        y += lane
        t += lane / v
        cur = (cur[0], y)
        path.append((t, *cur))
        left = not left
    return path


def summarize(errs, n_expected):
    e = np.asarray(errs)
    return {"frames_tracked": int(len(e)), "frames_expected": int(n_expected),
            "failure_rate": round(1 - min(1.0, len(e) / n_expected), 4),
            "median_m": round(float(np.median(e)), 4), "p90_m": round(float(np.percentile(e, 90)), 4),
            "max_m": round(float(e.max()), 4)}


def tagged(seed):
    cfg = FieldConfig.vex()
    poses = [(i * 0.05, 1.8 + math.cos(i / 40 * math.pi / 2 - math.pi / 2), 1.8 + math.sin(i / 40 * math.pi / 2 - math.pi / 2),
              i / 40 * math.pi / 2) for i in range(41)]
    res = track_frames(render_frames(poses, cfg, fps=15, seed=seed), cfg, fps=15)
    errs = [math.hypot(x - poses[min(int(round(t / 0.05)), 40)][1], y - poses[min(int(round(t / 0.05)), 40)][2]) for t, x, y, _ in res.poses]
    return summarize(errs, int(2.0 * 15) + 1)


def untagged(seed, measured: bool):
    path = lawnmower()
    with tempfile.TemporaryDirectory() as d:
        v = Path(d) / "v.mp4"
        info = render_floor_video([(t, x, y, 0.0) for t, x, y in path], v, floor_m=(-0.6, 0.8, 0.6, 3.0), seed=seed)
        cal = Calibration([540, 960], info["corners_px"], measured_width_m=1.2, measured_length_m=2.2) if measured else \
            Calibration([540, 960], info["corners_px"])
        res = track_untagged(v, cal)
    T, X, Y = (np.array(a) for a in zip(*path))
    tt = np.array([p[0] + res.stats["track_start_s"] for p in res.poses])
    e = np.hypot(np.array([p[1] for p in res.poses]) - (np.interp(tt, T, X) + 0.6), np.array([p[2] for p in res.poses]) - (np.interp(tt, T, Y) - 0.8))
    return summarize(e, int(T[-1] * 15) + 1) | {"mode": res.camera.get("mode")}


def main():
    out = {"note": "Synthetic rendered video only; real-world validation is planned.", "seeds": SEEDS, "conditions": {}}
    for name, fn in (("tags", tagged), ("no tags, measured floor", lambda s: untagged(s, True)),
                     ("no tags, robot size only", lambda s: untagged(s, False))):
        runs = [fn(s) for s in SEEDS]
        out["conditions"][name] = {"runs": runs, "median_m": round(float(np.median([r["median_m"] for r in runs])), 4),
                                   "p90_m": round(float(np.median([r["p90_m"] for r in runs])), 4),
                                   "failure_rate": round(float(np.mean([r["failure_rate"] for r in runs])), 4)}
        print(name, out["conditions"][name]["median_m"], out["conditions"][name]["p90_m"], out["conditions"][name]["failure_rate"], file=sys.stderr)
    OUT.write_text(json.dumps(out, indent=1))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
