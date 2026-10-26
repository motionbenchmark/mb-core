import json
import math
from pathlib import Path

import numpy as np
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


def test_aggregate_scores_every_agent_against_the_goal(dataset):
    reports, errors = mb.score_dir(dataset)
    # one trace was tampered with in another test that may have run first
    assert len(reports) + len(errors) == 20 * 2 + 10
    lb = mb.aggregate(reports)
    assert lb["spec_version"] == "0.2"
    assert "overall" not in lb  # no ranking across different tasks
    nav = next(b for b in lb["tasks"] if b["task"] == "MB-VEX-NAV@1.0")
    assert all(0.0 <= r["goal"] <= 1.0 for r in nav["rows"])
    walk = next(r for r in nav["rows"] if r["agent_id"] == "random-walk")
    assert walk["goal"] == pytest.approx(0.0)
    # drivers and code share the robot, so people are a fair comparison here
    assert nav["people_comparable"] and nav["people"]["n_people"] >= 11
    cover = next(b for b in lb["tasks"] if b["task"] == "MB-COVER@0.1")
    assert cover["chance_source"] == "baseline" and cover["chance"] > 0.3
    # a robot vacuum and a person with a stick vacuum use different means
    assert not cover["people_comparable"] and cover["people"] is None
    assert all(r["vs_people"] is None for r in cover["rows"])


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


# ----------------------------------------------------------------------------- multi-robot matches
@pytest.fixture(scope="module")
def match_dir(tmp_path_factory):
    from motionbench.match import simulate, write_match

    return write_match(simulate(3), tmp_path_factory.mktemp("match") / "m0003")


def test_match_sim_is_deterministic_and_complete(match_dir):
    from motionbench.match import MatchData, simulate

    a, b = simulate(3), simulate(3)
    for rid in a.poses:
        assert (a.poses[rid] == b.poses[rid]).all()
    m = MatchData(match_dir)
    assert sorted(r["id"] for r in m.robots) == ["blue1", "blue2", "red1", "red2"]
    assert m.meta["synthetic"] is True
    F = m.meta["field"]["size_m"]
    for p in m.poses.values():
        assert p[-1, 0] == pytest.approx(123.0, abs=0.06)
        assert (p[:, 1:3] > 0).all() and (p[:, 1:3] < F).all()
        # auton: each robot stays on its own half
    for r in m.robots:
        p = m.poses[r["id"]]
        auton = p[p[:, 0] < 15]
        assert ((auton[:, 1] < F / 2) if r["alliance"] == "red" else (auton[:, 1] > F / 2)).all()
        # nothing moves during the pause
        pause = p[(p[:, 0] > 15.3) & (p[:, 0] < 18)]
        assert np.ptp(pause[:, 1]) < 0.03
    # every robot trace is a valid .mbtrace the standard scorer accepts
    r = mb.score(match_dir / "robots" / "red1", tasks_dir=match_dir / "tasks")
    assert r["coverage"]["covered_pct"] > 5


def test_match_report_counts_contacts_and_periods(match_dir):
    from motionbench.match.report import report

    r = report(match_dir)
    assert set(r["robots"]) == {"red1", "red2", "blue1", "blue2"}
    for rr in r["robots"].values():
        assert set(rr["periods"]) == {"auton", "driver"}
        assert rr["periods"]["auton"]["opponent_half_pct"] == 0.0
        assert rr["match"]["distance_m"] > 5
    assert sum(p["contacts"] for p in r["pairs"]) > 0
    assert r["alliances"]["red"]["deliveries"] > 0


def _oracle(match_dir, path, swap_after=None, drop=0.0):
    import csv as _csv

    from motionbench.match import MatchData

    m = MatchData(match_dir)
    rng = np.random.default_rng(1)
    with open(path, "w", newline="") as f:
        w = _csv.writer(f)
        w.writerow(["t_s", "track_id", "x_m", "y_m", "alliance"])
        for t in np.arange(0, 123, 0.2):
            for k, r in enumerate(m.robots):
                if rng.random() < drop:
                    continue
                tid = k
                if swap_after is not None and t > swap_after and k < 2:
                    tid = 1 - k
                x, y, _ = m.pose(r["id"], t)
                w.writerow([round(float(t), 3), tid, x, y, r["alliance"]])
    return path


def test_match_eval_scores_identities(match_dir, tmp_path):
    from motionbench.match.eval import evaluate

    perfect = evaluate(match_dir, str(_oracle(match_dir, tmp_path / "a.csv")))
    assert perfect["overall"]["recall"] == 1.0 and perfect["overall"]["precision"] == 1.0
    assert perfect["overall"]["median_error_m"] < 1e-3 and perfect["overall"]["id_switches"] == 0
    assert perfect["idf1"] == 1.0 and perfect["alliance_accuracy"] == 1.0
    swapped = evaluate(match_dir, str(_oracle(match_dir, tmp_path / "b.csv", swap_after=60, drop=0.1)))
    assert swapped["overall"]["id_switches"] == 2
    assert 0.85 < swapped["overall"]["recall"] < 0.95
    assert swapped["idf1"] < 0.85


def test_match_render_and_baseline_tracker(match_dir, tmp_path):
    from motionbench.match.eval import evaluate
    from motionbench.match.render import render_match
    from motionbench.match.track import track_match

    # 40 s is enough for the empty-field median to lose the robots
    info = render_match(match_dir, tmp_path / "clip.mp4", fps=3, start_s=20, end_s=60, width=680, seed=1)
    assert info["frames"] == 121 and Path(info["gt"]).exists()
    out = track_match(info["video"], str(tmp_path / "tracks.csv"), fps=3, match_start_s=-20)
    assert out["tracks"] >= 4
    r = evaluate(match_dir, str(tmp_path / "tracks.csv"), t_range=(21, 60))
    assert r["overall"]["recall"] > 0.8
    assert r["overall"]["median_error_m"] < 0.12


def test_match_camera_from_calib_and_from_tracks(match_dir, tmp_path):
    from motionbench.match import BROADCAST_CAMERA
    from motionbench.match.io import camera_from_calib, match_from_tracks
    from motionbench.match.report import report

    calib = tmp_path / "calib.json"
    calib.write_text(json.dumps({"frame_size": BROADCAST_CAMERA["frame_size"], "floor_corners_px": BROADCAST_CAMERA["floor_corners_px"],
                                 "measured_width_m": 3.566, "measured_length_m": 3.566}))
    cam = camera_from_calib(calib)
    assert cam["f_px"] == pytest.approx(BROADCAST_CAMERA["f_px"], rel=0.01)
    assert cam["camera_height_m"] == pytest.approx(BROADCAST_CAMERA["camera_height_m"], rel=0.01)
    # a track that breaks for half a second is stitched back into one robot
    src = _oracle(match_dir, tmp_path / "t.csv")
    lines = open(src).read().splitlines()
    out = [lines[0]]
    for ln in lines[1:]:
        t, tid, *rest = ln.split(",")
        if tid == "0" and 50 < float(t) < 50.5:
            continue
        if tid == "0" and float(t) >= 50.5:
            tid = "9"
        out.append(",".join([t, tid, *rest]))
    (tmp_path / "broken.csv").write_text("\n".join(out) + "\n")
    d = match_from_tracks(tmp_path / "broken.csv", tmp_path / "tracked")
    meta = json.loads((d / "match.json").read_text())
    assert sorted(r["id"] for r in meta["robots"]) == ["blue1", "blue2", "red1", "red2"]
    assert any(r["tracks"] == ["0", "9"] for r in meta["robots"])
    assert report(d)["robots"]["red1"]["match"]["distance_m"] > 5


# ----------------------------------------------------------------------------- games: code vs simulated drivers
GAME_DIR = Path(__file__).resolve().parents[2] / "games" / "aim-sort"


def test_game_definition_validates_and_scores():
    from motionbench.game import Game

    g = Game.load(GAME_DIR)
    assert g.key == "AIM-SORT@0.1" and g.max_points == 6
    s = g.score_world([{"id": "o1", "kind": "barrel-orange", "x": 0.1, "y": 0.6},
                       {"id": "b1", "kind": "barrel-blue", "x": 0.1, "y": 0.6},
                       {"id": "b2", "kind": "barrel-blue", "x": 1.7, "y": 0.6, "held": True}])
    assert s["points"] == 1


def test_generated_layouts_are_new_valid_and_repeatable():
    from motionbench.game import Game

    g = Game.load(GAME_DIR)
    a, b = g.layout("gen-11")[1], g.layout("gen-11")[1]
    assert a == b and a["objects"] != g.layout("dev-01")[1]["objects"]
    assert len(a["objects"]) == 6 and a["start"] == g.layout("dev-01")[1]["start"]
    for o in a["objects"]:
        assert not g.zone_of(o["x"], o["y"])  # nothing starts already sorted


def test_vexcode_aim_emulator_moves_like_the_api(tmp_path):
    from motionbench.game import Game, simulate

    g = Game.load(GAME_DIR)
    code = "from vex import *\nrobot=Robot()\nrobot.set_move_velocity(100)\nrobot.move_for(300, 90)\nrobot.turn_to(90)\nwait(1, SECONDS)\n"
    r = simulate(g, code=code, layout="dev-01", noise=0.0)
    t, x, y, th = r.poses[-1]
    assert x == pytest.approx(0.9144 + 0.3, abs=0.01) and y == pytest.approx(0.12, abs=0.01)  # 300 mm to the right
    assert th == pytest.approx(0.0, abs=0.03)  # heading 90 = facing +x
    assert 3.0 < r.ended_s < 5.0  # 1.5 s at 200 mm/s, a 90 deg turn at the default 75 deg/s, and the wait
    # runaway loops still end when the clock does; crashes are reported, not raised
    assert simulate(g, code="from vex import *\nwhile True:\n    pass\n", layout="dev-01").status == "time"
    bad = simulate(g, code="from vex import *\nrobot=Robot()\nrobot.fly()\n", layout="dev-01")
    assert bad.status == "error" and "fly" in bad.error


def test_game_runs_agree_with_the_standard_scorer(tmp_path):
    from motionbench.game import CHARACTERS, Game, simulate, write_run

    g = Game.load(GAME_DIR)
    code = (GAME_DIR / "routines" / "planner_v4.py").read_text()
    a = simulate(g, code=code, layout="dev-01", seed=4)
    b = simulate(g, code=code, layout="dev-01", seed=4)
    assert a.poses == b.poses and a.points == b.points  # deterministic per seed
    assert a.points >= 3
    write_run(a, tmp_path / "run", g)
    rep = mb.score(tmp_path / "run" / "trace")
    assert rep["success"]["score"] == a.points
    d = simulate(g, driver=CHARACTERS[-1], layout="dev-01", seed=1)
    assert d.points >= 2 and d.agent_kind == "driver"


def test_paired_bootstrap_and_outcomes():
    from motionbench.game.verdict import outcome, paired_ci

    rng = np.random.default_rng(0)
    d, lo, hi = paired_ci([3, 3, 4, 4], [2, 2, 3, 3], None, rng)
    assert d == 1 and lo == 1 and hi == 1  # identical paired differences: no spread
    d, lo, hi = paired_ci([3, 4, 3, 4], [3, 3, 4, 4], [0, 0, 1, 1], rng)
    assert lo <= d <= hi
    assert outcome(0.1, 0.9, 0.5) == "better" and outcome(-0.9, -0.1, 0.5) == "worse"
    assert outcome(-0.3, 0.4, 0.5) == "equivalent" and outcome(-0.8, 0.4, 0.5) == "no difference found"
    assert outcome(-0.5, 0.4, 0.5) == "no difference found"  # touching the margin is not equivalence


def test_game_verdict_compares_code_with_drivers():
    from motionbench.game import CHARACTERS, Game
    from motionbench.game.verdict import verdict

    g = Game.load(GAME_DIR)
    code = (GAME_DIR / "routines" / "planner_v4.py").read_text()
    r = verdict(g, {"v4.py": code}, layout="dev-01", runs=4, class_size=4, class_runs=1,
                drivers=[CHARACTERS[0], CHARACTERS[-1]], unseen=2)
    comp = r["comparisons"]["v4.py"]
    pub = r["public"]["comparisons"]["v4.py"]
    assert set(pub) == {"typical student", "Rookie Riley", "Pro Priya"}
    assert pub["Rookie Riley"]["verdict"] in ("better", "worse", "equivalent", "no difference found")
    assert r["unseen"]["change_from_public"]["v4.py"]["verdict"] in ("better", "worse", "equivalent", "no difference found")
    assert r["design"]["paired"] and r["design"]["unseen_layouts"] == 2
    assert r["simulated"] is True and comp["headline"]


# -- robo-bench games: AprilTag signs, the targets goal, the AIM WebSocket client -------------------------
GAMES = Path(__file__).resolve().parents[2] / "games"


def _tag1_trace(tmp_path, events):
    task = mb.load_task("MB-AIM-TAG1@0.1")
    s = task["layouts"]["photo"]["start"]
    poses = [(i * 0.1, s["x"] - i * 0.002, s["y"], s["theta"]) for i in range(50)]
    return mb.write_trace(tmp_path / "t", task="MB-AIM-TAG1@0.1", seed="photo", agent_id="t", agent_class="ai",
                          poses=poses, events=events, tier="S", capture="sim")


def test_targets_goal_counts_only_valid_knocks(tmp_path):
    assert mb.score(_tag1_trace(tmp_path / "a", [(2.0, "target_down", "T1", "ball")]))["success"]["score"] == 1
    r = mb.score(_tag1_trace(tmp_path / "b", [(2.0, "target_down", "T1", "rammed"), (2.0, "rule_flag", "T1", "rammed")]))
    assert r["success"]["score"] == 0 and r["safety"]["rule_flags"] == 1
    assert mb.score(_tag1_trace(tmp_path / "c", []))["success"]["completion"] == 0


def _held_ball_facing_tag(game_dir, seed=0):
    from motionbench.game.spec import Game
    from motionbench.game.world import World

    g = Game.load(GAMES / game_dir)
    w = World(g, seed=seed)
    tag = next(o for o in w.objects if o.shape == "tag" and o.id == "T1")
    ball = next(o for o in w.objects if o.shape == "ball")
    face = math.radians(90 - tag.facing)  # stand 12 cm in front of the sign, facing it
    w.x, w.y = tag.x + math.cos(face) * 0.12, tag.y + math.sin(face) * 0.12
    w.th = face + math.pi
    ball.held, w.held = True, ball
    return g, w, tag


def test_kicked_ball_knocks_the_sign_and_ramming_is_flagged():
    g, w, tag = _held_ball_facing_tag("rb-tag1")
    w.kick("HARD")
    for _ in range(200):
        w.step()
    assert tag.down and tag.knocked_by == "ball"
    assert g.score_world(w.final_objects(), w.events)["points"] == 1
    g, w, tag = _held_ball_facing_tag("rb-tag1")
    w.held.held, w.held = False, None
    w.objects[0].x = 0.05  # ball out of the way
    w.cmd_v = (math.cos(w.th) * 0.1, math.sin(w.th) * 0.1)
    for _ in range(200):
        w.step()
    assert tag.down and tag.knocked_by == "rammed"
    assert any(e[1] == "rule_flag" for e in w.events)
    assert g.score_world(w.final_objects(), w.events)["points"] == 0


def test_turning_against_a_wall_knocks_the_held_ball_loose():
    from motionbench.game.spec import Game
    from motionbench.game.world import World

    w = World(Game.load(GAMES / "rb-tag1"))
    ball = w.objects[0]
    w.x, w.y, w.th = w.W - w.R - 0.001, 0.25, 0.0  # nose against the right wall
    ball.held, w.held = True, ball
    w.cmd_w = 2.0
    for _ in range(50):
        w.step()
    assert not ball.held and any(e[1] == "dislodge" for e in w.events)


def test_robo_bench_games_validate_and_link_tasks():
    from motionbench.game.spec import Game

    keys = {f"{t['id']}@{t['version']}" for t in mb.tasks()}
    for name in ("rb-tag1", "rb-tag2", "rb-tag2-barrels", "rb-sort"):
        g = Game.load(GAMES / name)
        assert g.raw["motionbench_task"] in keys
        assert g.max_points == {"rb-tag1": 1, "rb-tag2": 2, "rb-tag2-barrels": 2, "rb-sort": 6}[name]


AIM_CLIENT = Path(__import__("os").environ.get("AIM_CLIENT_DIR", Path(__file__).resolve().parents[3] / "robo-bench"))


@pytest.mark.skipif(not (AIM_CLIENT / "vex" / "aim.py").exists(), reason="AIM WebSocket client not found (set AIM_CLIENT_DIR)")
def test_aim_websocket_client_runs_unchanged_against_the_simulated_robot():
    from motionbench.game.aim_websocket import run_program
    from motionbench.game.spec import Game
    from motionbench.game.world import World

    code = """
from vex import *
robot = Robot()
timer = Timer()
robot.vision.model_detection(True)
ball = robot.vision.get_data(AiObjdesc(0))
print("ball", len(ball), round(ball[0].bearing) if ball else None)
def ticker():
    for _ in range(3):
        print("tick", round(timer.time(SECONDS)))
        wait(1, SECONDS)
Thread(ticker)
robot.turn_for(RIGHT, 90, 50)
print("heading", round(robot.inertial.get_heading()))
robot.move_for(2000, 0, 10)
robot.kicker.kick(HARD)
"""
    g = Game.load(GAMES / "rb-tag1")
    r = run_program(code, World(g, seed=0), AIM_CLIENT, filename="p.py")
    out = [ln for _, ln in r["console"]]
    assert out[0].startswith("Welcome to the AIM Websocket Python Client")
    assert out[1].startswith("ball 1")
    assert any(ln.startswith("tick") for ln in out) and any(ln.startswith("heading 9") for ln in out)
    assert any("wait timed out, stopping" in ln for ln in out)  # the client's 10 s limit on blocking moves
    assert r["status"] == "error" and "HARD" in r["error"]  # HARD isn't exported by `from vex import *`, as on the robot
    assert r["status_log"] and r["ended_s"] < 20
