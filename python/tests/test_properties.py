"""Properties of the scorer that hold by construction, checked on random traces.

These complement the hand-worked cases: instead of exact expected values, each test checks
a property that must hold for every trace, on many randomly generated ones (fixed seeds, so
the tests are repeatable). Tasks are the hand-worked ones (test_handworked.TASKS).

  1. determinism        the same trace always gives the same report
  2. bounds             goal share and path efficiency lie in [0, 1]
  3. agent-blindness    the agent's class, id and name do not change a run's score
  4. time origin        shifting every timestamp by a constant does not change the score
  5. waypoints only grow appending samples to a run never lowers its waypoint score
  6. coverage only grows extending a path never lowers its coverage
  7. delivery counts    moving one object into its zone raises the score by exactly 1
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from test_handworked import TASKS

mb = pytest.importorskip("motionbench")
N = 40


@pytest.fixture(scope="module")
def tasks_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("tasks")
    for key, t in TASKS.items():
        (d / f"{key}.json").write_text(json.dumps(t))
    return d


@pytest.fixture
def score(tmp_path, tasks_dir):
    n = [0]

    def f(task, poses, *, events=(), world_final=None, cls="human", agent="a", name=""):
        n[0] += 1
        tr = mb.write_trace(tmp_path / f"t{n[0]}", task=task, seed="L", agent_id=agent, agent_class=cls,
                            agent_name=name, poses=poses, events=events, world_final=world_final, decimals=5)
        return mb.score(tr, tasks_dir)
    return f


def random_path(rng, size=2.0, n=None, t_end=9.0, start=(0.2, 0.2)):
    """A random walk in a size x size arena, sometimes passing near the waypoints."""
    n = n or int(rng.integers(3, 40))
    t = np.sort(rng.uniform(0, t_end, n - 1))
    pts = [(0.0, start[0], start[1], 0.0)]
    for k, tk in enumerate(t):
        if rng.random() < 0.3:  # visit a waypoint, roughly
            x, y, th = [(1.0, 0.2, 0.0), (1.0, 1.0, math.pi / 2)][int(rng.integers(2))]
            x, y, th = x + rng.normal(0, 0.03), y + rng.normal(0, 0.03), th + rng.normal(0, 0.05)
        else:
            x, y, th = rng.uniform(0, size), rng.uniform(0, size), rng.uniform(-math.pi, math.pi)
        pts.append((float(tk) + 1e-3 * (k + 1), float(x), float(y), float(th)))
    return pts


def key(r):
    return (r["success"], r["efficiency"], r["safety"])


def test_1_determinism(score):
    rng = np.random.default_rng(1)
    for _ in range(N):
        p = random_path(rng)
        a, b = score("HW-WAY@0.1", p), score("HW-WAY@0.1", p)
        assert {k: v for k, v in a.items() if k != "trial"} == {k: v for k, v in b.items() if k != "trial"}


def test_2_bounds(score):
    rng = np.random.default_rng(2)
    for _ in range(N):
        for task in ("HW-WAY@0.1", "HW-COV@0.1"):
            size = 1.0 if task == "HW-COV@0.1" else 2.0
            r = score(task, random_path(rng, size=size, start=(0.25, 0.52) if size == 1.0 else (0.2, 0.2)))
            assert 0.0 <= r["success"]["completion"] <= 1.0
            assert 0.0 <= r["efficiency"]["path_efficiency"] <= 1.0


def test_3_agent_blindness(score):
    rng = np.random.default_rng(3)
    for _ in range(N):
        p = random_path(rng)
        ref = key(score("HW-WAY@0.1", p, cls="human", agent="person", name="A person"))
        for cls in ("human_auton", "ai", "hybrid", "product", "baseline"):
            assert key(score("HW-WAY@0.1", p, cls=cls, agent=f"x-{cls}", name="Someone else")) == ref


def test_4_time_origin(score):
    rng = np.random.default_rng(4)
    for _ in range(N):
        p = random_path(rng)
        shift = float(rng.uniform(1, 1000))
        q = [(t + shift, x, y, th) for t, x, y, th in p]
        a, b = score("HW-WAY@0.1", p), score("HW-WAY@0.1", q)
        assert a["success"] == b["success"]
        assert a["efficiency"]["path_efficiency"] == pytest.approx(b["efficiency"]["path_efficiency"], abs=1e-9)


def test_5_waypoints_only_grow(score):
    rng = np.random.default_rng(5)
    for _ in range(N):
        p = random_path(rng, t_end=5.0)
        extra = [(t + p[-1][0] + 0.01, x, y, th) for t, x, y, th in random_path(rng, t_end=4.5)[1:]]
        assert score("HW-WAY@0.1", p + extra)["success"]["score"] >= score("HW-WAY@0.1", p)["success"]["score"]


def test_6_coverage_only_grows(score):
    rng = np.random.default_rng(6)
    for _ in range(N):
        p = random_path(rng, size=1.0, t_end=10.0, start=(0.25, 0.52))
        extra = [(t + p[-1][0] + 0.01, x, y, th) for t, x, y, th in random_path(rng, size=1.0, t_end=15.0)[1:]]
        assert score("HW-COV@0.1", p + extra)["success"]["score"] >= score("HW-COV@0.1", p)["success"]["score"] - 1e-9


def test_7_delivery_counts(score):
    rng = np.random.default_rng(7)
    path = [(0.0, 0.2, 0.2, 0.0), (5.0, 1.75, 1.75, 0.0)]
    for _ in range(N):
        out = lambda: {"x": float(rng.uniform(0, 1.4)), "y": float(rng.uniform(0, 1.4))}
        inside = lambda: {"x": float(rng.uniform(1.5, 2.0)), "y": float(rng.uniform(1.5, 2.0))}
        o1, o2 = (inside() if rng.random() < 0.5 else out()), out()
        before = score("HW-DEL@0.1", path, world_final={"objects": [{"id": "o1", **o1}, {"id": "o2", **o2}]})
        after = score("HW-DEL@0.1", path, world_final={"objects": [{"id": "o1", **o1}, {"id": "o2", **inside()}]})
        assert after["success"]["score"] == before["success"]["score"] + 1
