"""Simulate one run of a game, by code or by a simulated driver, and keep everything:
the robot's path, the events, where every object ended up, the game score and
(when the game is linked to a MotionBench task) the standard scorer's report."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from ..trace import write_trace
from .driver import Driver, run_driver
from .spec import Game
from .vexcode_aim import run_code
from .world import World


@dataclass
class RunResult:
    game: str
    layout: str
    agent: str  # code file name or driver id
    agent_kind: str  # "code" | "driver"
    agent_name: str
    seed: int
    status: str
    error: Optional[str]
    points: float
    max_points: float
    finish_s: Optional[float]  # when the last scoring object was placed, if everything scored
    ended_s: float
    path_m: float
    bumps: int
    objects: list
    poses: list
    events: list
    score_detail: dict = field(default_factory=dict)
    mb_report: Optional[dict] = None
    object_track: list = field(default_factory=list)
    object_ids: list = field(default_factory=list)

    def summary(self) -> dict:
        return {k: getattr(self, k) for k in ("game", "layout", "agent", "agent_kind", "agent_name", "seed", "status", "error",
                                               "points", "max_points", "finish_s", "ended_s", "path_m", "bumps")}


def _path_len(poses) -> float:
    import math
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))


def _finish(world: World, detail: dict) -> Optional[float]:
    if detail["points"] < detail["max_points"]:
        return None
    t = [e[0] for e in world.events if e[1] in ("place", "kick")]
    return max(t) if t else None


def simulate(game: Game, *, code: Optional[str] = None, code_name: str = "routine.py", driver: Optional[Driver] = None,
             layout: Optional[str] = None, seed: int = 0, noise: float = 1.0) -> RunResult:
    if (code is None) == (driver is None):
        raise ValueError("give either code or a driver")
    world = World(game, layout, seed=seed, noise_level=noise)
    if code is not None:
        info = run_code(code, world, filename=code_name)
        agent, kind, name = code_name, "code", code_name
    else:
        info = run_driver(world, driver, seed=seed + 1)
        agent, kind, name = driver.id, "driver", driver.name
    world.settle()
    objs = world.final_objects()
    detail = game.score_world(objs)
    return RunResult(game.key, world.layout_name, agent, kind, name, seed, info["status"], info.get("error"), detail["points"],
                     detail["max_points"], _finish(world, detail), info["ended_s"], round(_path_len(world.poses), 3), world.bumps,
                     objs, world.poses, world.events, detail, None, world.object_track, [o.id for o in world.objects])


def write_run(res: RunResult, out: "str | Path", game: Game, *, agent_class: Optional[str] = None, score: bool = True) -> Path:
    """Write a run folder: run.json (game score and summary) and trace/ (a .mbtrace the standard scorer reads)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    cls = agent_class or ("human" if res.agent_kind == "driver" else "human_auton")
    task = game.raw.get("motionbench_task")
    if task:
        trace = write_trace(out / "trace", task=task, seed=res.layout, agent_id=_slug(res.agent), agent_class=cls,
                            agent_name=res.agent_name, poses=res.poses, events=res.events, trial=res.seed, tier="S", capture="sim",
                            world_final={"objects": [{"id": o["id"], "x": o["x"], "y": o["y"]} for o in res.objects]},
                            agent_meta={"simulated": True, "game": res.game, "kind": res.agent_kind},
                            notes="Simulated run (motionbench game simulate).", decimals=4)
        if score:
            try:
                import motionbench as mb
                res.mb_report = mb.score(trace)
            except Exception as e:  # the game score still stands
                res.mb_report = {"error": str(e)}
    (out / "run.json").write_text(json.dumps({"summary": res.summary(), "score": res.score_detail, "objects": res.objects,
                                              "events": res.events, "motionbench": res.mb_report}, indent=1))
    (out / "replay.json").write_text(json.dumps({"poses": res.poses, "object_ids": res.object_ids,
                                                 "object_kinds": [o["kind"] for o in res.objects], "objects": res.object_track,
                                                 "events": res.events, "summary": res.summary()}, separators=(",", ":")))
    return out


def _slug(s: str) -> str:
    base = "".join(c if c.isalnum() else "-" for c in s.lower()).strip("-")[:40] or "agent"
    return base if len(base) > 2 else base + "-" + hashlib.sha1(s.encode()).hexdigest()[:4]
