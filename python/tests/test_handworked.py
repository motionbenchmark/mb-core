"""Hand-worked scoring cases (docs/handworked-cases.md).

Each case is a tiny task and trace whose correct result is worked out by hand from the
task rules, not by running the scorer. The arithmetic is in the comments so it can be
checked with a pencil. Case numbers match the table in docs/handworked-cases.md.

Tasks (defined in TASKS below; not part of the benchmark):

* HW-WAY@0.1: 2 m x 2 m, robot radius 0.10 m (contact tolerance 0.03 m, so contact when the
  centre is within 0.07 m of a wall), budget 10 s, start (0.2, 0.2, heading 0),
  W1 = (1.0, 0.2, heading 0), W2 = (1.0, 1.0, heading pi/2); tolerances 0.05 m and
  0.0873 rad (5 degrees). People comparable.
* HW-DEL@0.1: 2 m x 2 m, budget 10 s, start (0.2, 0.2); objects o1 = (0.5, 0.5) and
  o2 = (1.0, 0.5), both for zone G = x in [1.5, 2.0], y in [1.5, 2.0] (centre (1.75, 1.75)).
* HW-COV@0.1: 1 m x 1 m, cells 0.1 m (centres 0.05, 0.15, ..., 0.95), tool 0.3 m (radius
  0.15 m), budget 30 s.
"""
from __future__ import annotations

import json
import math

import pytest

mb = pytest.importorskip("motionbench")

Q = math.pi / 2
W1, W2 = (1.0, 0.2, 0.0), (1.0, 1.0, Q)
TASKS = {
    "HW-WAY@0.1": {
        "id": "HW-WAY", "version": "0.1", "title": "hand-worked waypoints", "domain": "test", "description": "test",
        "arena": {"width_m": 2.0, "height_m": 2.0, "obstacles": [], "zones": {}},
        "robot": {"radius_m": 0.10}, "budget": {"time_s": 10},
        "goal": {"kind": "waypoints", "pos_tol_m": 0.05, "heading_tol_rad": 0.0873},
        "reference": {"floor": 0, "people_comparable": True},
        "layouts": {"L": {"start": {"x": 0.2, "y": 0.2, "theta": 0},
                          "waypoints": [{"x": 1.0, "y": 0.2, "theta": 0}, {"x": 1.0, "y": 1.0, "theta": Q}]}},
    },
    "HW-DEL@0.1": {
        "id": "HW-DEL", "version": "0.1", "title": "hand-worked delivery", "domain": "test", "description": "test",
        "arena": {"width_m": 2.0, "height_m": 2.0, "obstacles": [],
                  "zones": {"G": {"x": 1.5, "y": 1.5, "w": 0.5, "h": 0.5, "label": "goal"}}},
        "robot": {"radius_m": 0.10}, "budget": {"time_s": 10}, "goal": {"kind": "delivery"},
        "reference": {"floor": 0},
        "layouts": {"L": {"start": {"x": 0.2, "y": 0.2, "theta": 0},
                          "objects": [{"id": "o1", "x": 0.5, "y": 0.5, "zone": "G"},
                                      {"id": "o2", "x": 1.0, "y": 0.5, "zone": "G"}]}},
    },
    "HW-COV@0.1": {
        "id": "HW-COV", "version": "0.1", "title": "hand-worked coverage", "domain": "test", "description": "test",
        "arena": {"width_m": 1.0, "height_m": 1.0, "obstacles": [], "zones": {}},
        "robot": {"radius_m": 0.10}, "budget": {"time_s": 30},
        "goal": {"kind": "coverage", "cell_m": 0.1, "tool_width_m": 0.3},
        "reference": {"floor": 0},
        "layouts": {"L": {"start": {"x": 0.25, "y": 0.52, "theta": 0}}},
    },
}

START = (0.0, 0.2, 0.2, 0.0)
BOTH = [START, (2.0, *W1), (3.0, 1.0, 0.2, Q), (5.0, *W2)]  # start -> W1, turn, -> W2


@pytest.fixture(scope="module")
def tasks_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("tasks")
    for key, t in TASKS.items():
        (d / f"{key}.json").write_text(json.dumps(t))
    return d


@pytest.fixture
def score(tmp_path, tasks_dir):
    n = [0]

    def f(task, poses, *, events=(), world_final=None, cls="human", agent="a"):
        n[0] += 1
        tr = mb.write_trace(tmp_path / f"t{n[0]}", task=task, seed="L", agent_id=agent, agent_class=cls, poses=poses,
                            events=events, world_final=world_final, trial=n[0], decimals=5)
        return mb.score(tr, tasks_dir)
    return f


def board(reports, tasks_dir):
    t = mb.aggregate(reports, tasks_dir)["tasks"][0]
    return t, {r["agent_id"]: r for r in t["rows"]}


# --- 1-8: waypoints -----------------------------------------------------------------------

def test_01_both_waypoints(score):
    # Start (0.2,0.2) -> W1 (1.0,0.2): 0.8 m; turn in place; -> W2 (1.0,1.0): 0.8 m.
    # Score 2/2. Reference = 0.8 + 0.8 = 1.6 m; driven to the last hit = 1.6 m; efficiency 1.0.
    r = score("HW-WAY@0.1", BOTH)
    assert r["success"]["score"] == 2 and r["success"]["completed"]
    assert r["efficiency"]["oracle_length_m"] == pytest.approx(1.6)
    assert r["efficiency"]["path_efficiency"] == pytest.approx(1.0)


def test_02_just_inside_tolerance(score):
    # Stops at (1.049, 0.2): 0.049 m from W1 <= 0.05 -> hit. Score 1.
    assert score("HW-WAY@0.1", [START, (2.0, 1.049, 0.2, 0.0)])["success"]["score"] == 1


def test_03_just_outside_tolerance(score):
    # Stops at (1.051, 0.2): 0.051 m > 0.05 -> missed. Score 0.
    assert score("HW-WAY@0.1", [START, (2.0, 1.051, 0.2, 0.0)])["success"]["score"] == 0


def test_04_wrong_heading(score):
    # At W1's position with heading 0.2 rad: 0.2 > 0.0873 -> missed. Score 0.
    assert score("HW-WAY@0.1", [START, (2.0, 1.0, 0.2, 0.2)])["success"]["score"] == 0


def test_05_out_of_order(score):
    # Through W2 first (cannot count while W1 is unmet), then W1 (counts). Score 1.
    r = score("HW-WAY@0.1", [(0.0, 0.2, 0.2, Q), (2.0, *W2), (3.0, 1.0, 1.0, 0.0), (5.0, *W1)])
    assert r["success"]["score"] == 1


def test_06_at_the_time_limit(score):
    # W2 reached at exactly t = 10.0 s = budget: samples with t <= budget count. Score 2.
    assert score("HW-WAY@0.1", BOTH[:3] + [(10.0, *W2)])["success"]["score"] == 2


def test_07_after_the_time_limit(score):
    # The same path, but W2 reached at t = 10.5 s: that sample is ignored, with a warning. Score 1.
    r = score("HW-WAY@0.1", BOTH[:3] + [(10.5, *W2)])
    assert r["success"]["score"] == 1
    assert any("budget" in w for w in r["warnings"])


def test_08_gap_in_the_record(score):
    # Samples at (0.2,0.2) then (1.8,0.2): the straight line passes through W1, but no
    # recorded sample is within 0.05 m of it (0.8 m either side). The scorer does not
    # interpolate: score 0.
    assert score("HW-WAY@0.1", [START, (4.0, 1.8, 0.2, 0.0)])["success"]["score"] == 0


# --- 9-12: delivery -----------------------------------------------------------------------

PATH_DEL = [START, (4.0, 1.75, 1.75, 0.0)]


def test_09_one_of_two(score):
    # o1 at the zone centre (1.75,1.75); o2 left at (1.0,0.5). Score 1/2.
    r = score("HW-DEL@0.1", PATH_DEL, world_final={"objects": [{"id": "o1", "x": 1.75, "y": 1.75},
                                                               {"id": "o2", "x": 1.0, "y": 0.5}]})
    assert r["success"]["score"] == 1


def test_10_on_the_zone_edge(score):
    # o2 at (1.5, 1.75): exactly on the zone's left edge. The zone is a closed rectangle: 2/2.
    r = score("HW-DEL@0.1", PATH_DEL, world_final={"objects": [{"id": "o1", "x": 1.75, "y": 1.75},
                                                               {"id": "o2", "x": 1.5, "y": 1.75}]})
    assert r["success"]["score"] == 2 and r["success"]["completed"]


def test_11_just_outside_the_edge(score):
    # o2 at (1.49, 1.75): 1 cm outside. Score 1/2.
    r = score("HW-DEL@0.1", PATH_DEL, world_final={"objects": [{"id": "o1", "x": 1.75, "y": 1.75},
                                                               {"id": "o2", "x": 1.49, "y": 1.75}]})
    assert r["success"]["score"] == 1


def test_12_delivery_reference_length(score):
    # Greedy single-carry reference from the start: nearest object o1, 0.3*sqrt(2) = 0.42426 m;
    # o1 to the zone centre, 1.25*sqrt(2) = 1.76777 m; after one delivery 2.19203 m.
    a, b = 0.3 * math.sqrt(2), 1.25 * math.sqrt(2)
    final = {"objects": [{"id": "o1", "x": 1.75, "y": 1.75}, {"id": "o2", "x": 1.0, "y": 0.5}]}
    # Driving exactly that path, with the delivery at its end: efficiency 1.0.
    r = score("HW-DEL@0.1", [START, (2.0, 0.5, 0.5, 0.0), (6.0, 1.75, 1.75, 0.0)],
              events=[(6.0, "deliver", "o1", "")], world_final=final)
    assert r["efficiency"]["oracle_length_m"] == pytest.approx(a + b, abs=1e-5)
    assert r["efficiency"]["oracle_length_m"] == pytest.approx(2.19203, abs=1e-5)
    assert r["efficiency"]["path_efficiency"] == pytest.approx(1.0, abs=1e-5)
    # start -> o1 -> centre -> start -> o1 -> centre: the centre-to-start leg is 1.55*sqrt(2)
    # = 2.19203 m, so the path is 3 x 2.19203 m and the delivery (t = 9 s, inside the budget)
    # comes at its end: efficiency 1/3.
    r = score("HW-DEL@0.1", [START, (1.0, 0.5, 0.5, 0.0), (3.0, 1.75, 1.75, 0.0), (5.0, 0.2, 0.2, 0.0),
                             (6.0, 0.5, 0.5, 0.0), (9.0, 1.75, 1.75, 0.0)],
              events=[(9.0, "deliver", "o1", "")], world_final=final)
    assert r["efficiency"]["path_efficiency"] == pytest.approx(1 / 3, abs=1e-5)


# --- 13-15: boards ------------------------------------------------------------------------

ONE = [START, (2.0, *W1)]
ZERO = [START, (2.0, 0.3, 0.2, 0.0)]


def test_13_consistency(score, tasks_dir):
    # c = 1 - sample sd / mean, clamped to [0,1]; 0 when the mean is 0.
    # 1,1,1,1,1: sd 0 -> 1.0.
    # 2,2,1,2,2: mean 1.8; squared deviations 4 x 0.04 + 0.64 = 0.8; sample variance 0.8/4 = 0.2;
    #   sd 0.44721; c = 1 - 0.44721/1.8 = 0.75155.
    # 0,0,0,0,0: mean 0 -> 0.
    reps = [score("HW-WAY@0.1", ONE, agent="ones") for _ in range(5)]
    reps += [score("HW-WAY@0.1", p, agent="mixed") for p in (BOTH, BOTH, ONE, BOTH, BOTH)]
    reps += [score("HW-WAY@0.1", ZERO, agent="zeros") for _ in range(5)]
    _, rows = board(reps, tasks_dir)
    assert rows["ones"]["consistency"] == pytest.approx(1.0)
    assert rows["mixed"]["consistency"] == pytest.approx(0.75155, abs=1e-5)
    assert rows["zeros"]["consistency"] == 0


def test_14_chance(score, tasks_dir):
    # Baseline agent scoring 0,0,1,1,0 waypoints: mean 0.4 of 2 -> chance shown as a goal share
    # 0.4/2 = 0.2. Not subtracted: an agent at 2/2 keeps goal 1.0.
    reps = [score("HW-WAY@0.1", p, agent="rw", cls="baseline") for p in (ZERO, ZERO, ONE, ONE, ZERO)]
    reps += [score("HW-WAY@0.1", BOTH, agent="code", cls="human_auton") for _ in range(5)]
    t, rows = board(reps, tasks_dir)
    assert t["chance"] == pytest.approx(0.2)
    assert rows["code"]["goal"] == pytest.approx(1.0)


def test_15_people_comparison_guard(score, tasks_dir):
    # vs_people = (S - S_chance) / (S_people - S_chance), in task units, reported only when
    # S_people - S_chance >= 10% of the maximum (0.2 waypoints).
    # People scoring 1: (2 - 0.4) / (1 - 0.4) = 1.6 / 0.6 = 2.667.
    base = [score("HW-WAY@0.1", p, agent="rw", cls="baseline") for p in (ZERO, ZERO, ONE, ONE, ZERO)]
    code = [score("HW-WAY@0.1", BOTH, agent="code", cls="human_auton") for _ in range(5)]
    people = [score("HW-WAY@0.1", ONE, agent="person", cls="human") for _ in range(5)]
    _, rows = board(base + code + people, tasks_dir)
    assert rows["code"]["vs_people"] == pytest.approx(1.6 / 0.6, abs=1e-6)
    # People scoring 0: 0 - 0.4 < 0.2, so no comparison is reported.
    people0 = [score("HW-WAY@0.1", ZERO, agent="person", cls="human") for _ in range(5)]
    _, rows = board(base + code + people0, tasks_dir)
    assert rows["code"]["vs_people"] is None


# --- 16-17: coverage and contact ----------------------------------------------------------

def test_16_single_coverage_pass(score):
    # Straight pass at y = 0.52 from x = 0.25 to 0.75, tool radius 0.15 m. A cell counts if its
    # centre comes within 0.15 m of the path:
    #   row y=0.55 (dy 0.03): reach sqrt(0.15^2 - 0.03^2) = 0.147 -> x in [0.103, 0.897] -> 8 cells
    #   row y=0.45 (dy 0.07): reach 0.133 -> x in [0.117, 0.883] -> 8 cells
    #   row y=0.65 (dy 0.13): reach 0.075 -> x in [0.175, 0.825] -> 6 cells
    #   row y=0.35 (dy 0.17 > 0.15): none
    # 22 of 100 cells: 22%.
    r = score("HW-COV@0.1", [(0.0, 0.25, 0.52, 0.0), (5.0, 0.75, 0.52, 0.0)])
    assert r["success"]["score"] == pytest.approx(22.0)


def test_17_contact_episodes(score):
    # Contact when the centre is within 0.10 - 0.03 = 0.07 m of a wall. x = 0.05 (contact),
    # 0.5 (clear), 0.06 (contact), 0.06 (same contact continues): two episodes.
    r = score("HW-WAY@0.1", [(0.0, 0.05, 1.0, 0.0), (1.0, 0.5, 1.0, 0.0), (2.0, 0.06, 1.0, 0.0), (3.0, 0.06, 1.2, 0.0)])
    assert r["safety"]["boundary_violations"] == 2
