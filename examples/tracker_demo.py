"""Render synthetic overhead footage of a run, track it, and score the result.

    pip install "motionbench[tracker]"
    python examples/tracker_demo.py data/synthetic-v0.1/MB-VEX-NAV@1.0/dev-01/team-auton-a/trial-0
"""

import csv
import json
import sys
import tempfile
from pathlib import Path

import motionbench as mb
from motionbench.tracker import FieldConfig, track_video
from motionbench.tracker.render import render_video

src = Path(sys.argv[1])
manifest = json.loads((src / "manifest.json").read_text())
with open(src / "poses.csv") as f:
    poses = [tuple(float(r[k]) for k in ("t_s", "x_m", "y_m", "theta_rad")) for r in csv.DictReader(f)]

cfg = FieldConfig.vex()
with tempfile.TemporaryDirectory() as d:
    video = render_video(poses, cfg, Path(d) / "run.avi", fps=30)
    print(f"rendered {video}")
    res = track_video(video, cfg)
    print(f"tracked {len(res.poses)} poses, detection {res.detection_rate:.0%}, "
          f"calibration RMS {res.calibration_rms_m * 100:.2f} cm")
    out = res.write(Path(d) / "tracked", task=manifest["task"], seed=manifest["seed"],
                    agent_id=manifest["agent"]["id"] + "-video", agent_class=manifest["agent"]["class"])
    original = mb.score(src)
    tracked = mb.score(out)

for name, r in (("original trace", original), ("from video", tracked)):
    print(f"{name:<15} success {r['success']['score']:.0f}/{r['success']['max_score']:.0f}  "
          f"path {r['efficiency']['path_length_m']:.2f} m  efficiency {r['efficiency']['path_efficiency']:.3f}")
