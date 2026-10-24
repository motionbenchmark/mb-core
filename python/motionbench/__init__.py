"""MotionBench: an open benchmark for goal-directed agents in bounded spaces.

Score any run - a student driving a VEX robot, a team's autonomous routine, an
AI-written program, a driver with robot assistance, a robot vacuum - against the
task's goals, using one trace format and one deterministic scorer (Rust core).

    >>> import motionbench as mb
    >>> report = mb.score("runs/driver-01/MB-VEX-NAV@1.0/dev-01/t0")
    >>> report["success"]["score"]
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Optional, Union

from . import _core
from .trace import write_trace

__all__ = [
    "__version__",
    "spec_version",
    "tasks",
    "load_task",
    "score",
    "score_dir",
    "aggregate",
    "write_trace",
]

__version__: str = _core.scorer_version()
spec_version: str = _core.spec_version()

PathLike = Union[str, Path]


def _opt(p: Optional[PathLike]) -> Optional[str]:
    return None if p is None else str(p)


def tasks(tasks_dir: Optional[PathLike] = None) -> list[dict[str, Any]]:
    """All available tasks (built-in plus any ``<ID@VERSION>.json`` in ``tasks_dir``)."""
    return json.loads(_core.list_tasks(_opt(tasks_dir)))


def load_task(key: str, tasks_dir: Optional[PathLike] = None) -> dict[str, Any]:
    """Load one task by ``ID@VERSION``, e.g. ``"MB-VEX-NAV@1.0"``."""
    for t in tasks(tasks_dir):
        if f"{t['id']}@{t['version']}" == key:
            return t
    raise KeyError(f"unknown task {key!r}")


def score(trace_dir: PathLike, tasks_dir: Optional[PathLike] = None) -> dict[str, Any]:
    """Score one ``.mbtrace`` directory and return the report."""
    return json.loads(_core.score_trace(str(trace_dir), _opt(tasks_dir)))


def score_dir(root: PathLike, tasks_dir: Optional[PathLike] = None) -> tuple[list[dict], list[dict]]:
    """Score every trace under ``root``. Returns ``(reports, errors)``."""
    reports, errors = _core.score_dir(str(root), _opt(tasks_dir))
    return json.loads(reports), json.loads(errors)


def aggregate(reports: Iterable[dict[str, Any]], tasks_dir: Optional[PathLike] = None) -> dict[str, Any]:
    """Build per-task boards from reports: goal score, MBI, chance level, shared ranks, and a
    comparison with people on tasks that allow one."""
    return json.loads(_core.aggregate(json.dumps(list(reports)), _opt(tasks_dir)))
