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
    assert {"MB-VEX-NAV@1.0", "MB-VEX-PICK@1.0", "MB-COVER@0.1", "MB-AIM-NAV@0.1", "MB-AIM-SORT@0.1"} <= keys


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


def test_viz_builds_self_contained_page(dataset):
    from motionbench.viz import build

    html = build(dataset, title="tiny")
    assert html.startswith('<meta charset="utf-8">')
    assert '"dataset":"tiny"' in html and '"synthetic":true' in html
    assert html.count('"report":') >= 40  # 50 traces; one may be tampered with by another test
    assert "</script>" in html and "<\\/" not in html.split('id="mbdata"')[0]


def test_viz_write_is_a_standards_mode_document(tmp_path, dataset):
    from motionbench.viz import write

    out = write(dataset, tmp_path / "e.html", embed=True)
    text = out.read_text()
    assert text.startswith("<!doctype html>")
    assert "window.MB_EMBED = true" in text


def test_aim_sort_counts_only_matching_colour(tmp_path):
    plan = {"MB-AIM-SORT@0.1": {"seeds": ["dev-01"], "trials": 1, "roster": "aim"}}
    generate(tmp_path, master_seed=3, plan=plan, quiet=True)
    task = mb.load_task("MB-AIM-SORT@0.1")
    zones = task["arena"]["zones"]
    targets = {o["id"]: zones[o["zone"]] for o in task["layouts"]["dev-01"]["objects"]}
    for run in sorted((tmp_path / "MB-AIM-SORT@0.1" / "dev-01").glob("*/trial-0")):
        world = json.loads((run / "world_final.json").read_text())
        expected = sum(
            1 for o in world["objects"]
            if targets[o["id"]]["x"] <= o["x"] <= targets[o["id"]]["x"] + targets[o["id"]]["w"]
            and targets[o["id"]]["y"] <= o["y"] <= targets[o["id"]]["y"] + targets[o["id"]]["h"]
        )
        assert mb.score(run)["success"]["score"] == expected


def _lawnmower(x0=-0.4, x1=0.4, y_start=1.0, y_end=2.4, lane=0.35, v=0.3):
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


def _err_vs_truth(res, path, origin):
    P = [p for p in res.poses]
    tt = [p[0] + res.stats["track_start_s"] for p in P]
    import numpy as np

    T, X, Y = (np.array(v) for v in zip(*path))
    tx = np.interp(tt, T, X) - origin[0]
    ty = np.interp(tt, T, Y) - origin[1]
    e = np.hypot(np.array([p[1] for p in P]) - tx, np.array([p[2] for p in P]) - ty)
    return float(np.median(e))


def test_untagged_tracker_on_rendered_handheld_video(tmp_path):
    """Hand-held, untagged video of a dark disc robot on planks: positions within a few cm."""
    from motionbench.tracker.render import render_floor_video
    from motionbench.tracker.untagged import Calibration, track_untagged

    path = _lawnmower()
    info = render_floor_video([(t, x, y, 0.0) for t, x, y in path], tmp_path / "v.mp4", floor_m=(-0.6, 0.8, 0.6, 3.0))
    corners = info["corners_px"]
    exact = track_untagged(tmp_path / "v.mp4", Calibration([540, 960], corners, measured_width_m=1.2, measured_length_m=2.2))
    assert exact.camera["mode"] == "measured"
    assert _err_vs_truth(exact, path, (-0.6, 0.8)) < 0.04
    assert exact.task["arena"]["unobserved"], "floor outside the frame should be marked unobserved"
    rough = track_untagged(tmp_path / "v.mp4", Calibration([540, 960], corners))
    assert rough.camera["mode"] == "robot-size"
    assert _err_vs_truth(rough, path, (-0.6, 0.8)) < 0.12
    out = exact.write(tmp_path / "out", task_id="T-UNTAGGED", seed="s1", agent_id="disc", agent_class="product")
    r = mb.score(out / "trace", tasks_dir=out / "tasks")
    assert 0 < r["coverage"]["covered_pct"] <= 100


def test_calibration_page_is_self_contained(tmp_path):
    from motionbench.tracker.calibrate import write_calibration_page
    from motionbench.tracker.render import render_floor_video

    render_floor_video([(0, 0, 1.5, 0), (3, 0.3, 1.8, 0)], tmp_path / "v.mp4", floor_m=(-0.6, 0.8, 0.6, 3.0))
    page = write_calibration_page(tmp_path / "v.mp4", tmp_path / "c.html", png=tmp_path / "bg.png")
    html = page.read_text()
    assert "far-left" in html and "data:image/jpeg;base64," in html and "calib.json" in html
    assert "http://" not in html and "https://" not in html  # works offline, sends nothing anywhere
    assert (tmp_path / "bg.png").exists()
