import json
import math
from pathlib import Path

import pytest

import motionbench as mb
from motionbench.synth import generate

TINY_PLAN = {
    "MB-VEX-NAV@1.0": {"seeds": ["dev-01"], "trials": 2, "roster": "vex"},
    "MB-COVER@0.1": {"seeds": ["dev-01"], "trials": 1, "roster": "room"},
}


@pytest.fixture(scope="module")
def dataset(tmp_path_factory):
    out = tmp_path_factory.mktemp("synth")
    generate(out, master_seed=7, plan=TINY_PLAN, quiet=True)
    return out


def test_tasks_builtin():
    keys = {f"{t['id']}@{t['version']}" for t in mb.tasks()}
    assert {"MB-VEX-NAV@1.0", "MB-VEX-PICK@1.0", "MB-COVER@0.1"} <= keys


def test_write_and_score_roundtrip(tmp_path):
    task = mb.load_task("MB-VEX-NAV@1.0")
    lay = task["layouts"]["dev-01"]
    pts = [(lay["start"]["x"], lay["start"]["y"], lay["start"]["theta"])] + [(w["x"], w["y"], w["theta"]) for w in lay["waypoints"]]
    poses, t = [], 0.0
    for (ax, ay, _), (bx, by, bth) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / 0.04))
        for i in range(1, n + 1):
            t += 0.05
            poses.append((t, ax + (bx - ax) * i / n, ay + (by - ay) * i / n, bth))
    poses.insert(0, (0.0, *pts[0]))
    d = mb.write_trace(tmp_path / "run", task="MB-VEX-NAV@1.0", seed="dev-01", agent_id="ideal",
                       agent_class="human_auton", poses=poses)
    r = mb.score(d)
    assert r["success"]["score"] == 6
    assert r["success"]["completed"]


def test_tampered_trace_is_rejected(tmp_path, dataset):
    src = next(dataset.glob("MB-VEX-NAV@1.0/dev-01/driver-01/trial-0"))
    p = src / "poses.csv"
    p.write_text(p.read_text().replace("0.0", "0.1", 1))
    with pytest.raises(ValueError, match="sha256"):
        mb.score(src)


def test_synth_is_deterministic(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    plan = {"MB-VEX-PICK@1.0": {"seeds": ["dev-01"], "trials": 1, "roster": "vex"}}
    generate(a, 11, plan, quiet=True)
    generate(b, 11, plan, quiet=True)
    for f in a.rglob("*.csv"):
        assert f.read_bytes() == (b / f.relative_to(a)).read_bytes()


def test_aggregate_produces_human_normalized_board(dataset):
    reports, errors = mb.score_dir(dataset)
    # one trace was tampered with in another test that may have run first
    assert len(reports) + len(errors) == 20 * 2 + 10
    lb = mb.aggregate(reports)
    nav = next(b for b in lb["tasks"] if b["task"] == "MB-VEX-NAV@1.0")
    assert nav["human_reference"]["n_drivers"] >= 11
    walk = next(r for r in nav["rows"] if r["agent_id"] == "random-walk")
    assert walk["hns"] == pytest.approx(0.0)
    cover = next(b for b in lb["tasks"] if b["task"] == "MB-COVER@0.1")
    assert cover["floor_source"] == "baseline"
    assert lb["overall"][0]["rank"] == 1


cv2 = pytest.importorskip("cv2")


def test_tracker_recovers_rendered_run(tmp_path):
    from motionbench.tracker import FieldConfig, track_frames
    from motionbench.tracker.render import render_frames

    cfg = FieldConfig.vex()
    # a quarter-circle arc with the heading tangent to it
    poses = []
    for i in range(41):
        a = i / 40 * math.pi / 2
        poses.append((i * 0.05, 1.8 + math.cos(a - math.pi / 2), 1.8 + math.sin(a - math.pi / 2), a))
    res = track_frames(render_frames(poses, cfg, fps=15), cfg, fps=15)
    assert res.detection_rate > 0.95
    assert res.calibration_rms_m < 0.01
    errs = []
    for t, x, y, th in res.poses:
        i = min(int(round(t / 0.05)), 40)
        errs.append(math.hypot(x - poses[i][1], y - poses[i][2]))
    assert max(errs) < 0.03, max(errs)
