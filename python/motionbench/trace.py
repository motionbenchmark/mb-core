"""Write `.mbtrace` directories (spec v0.1).

A trace is a directory:

    manifest.json      task, seed, agent, tier, capture, sha256 of each file
    poses.csv          t_s,x_m,y_m,theta_rad
    events.csv         t_s,type,object_id,detail          (optional)
    world_final.json   {"objects": [{"id","x","y"}]}      (optional)
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

SPEC_VERSION = "0.1"
AGENT_CLASSES = ("human", "human_auton", "ai", "hybrid", "product", "baseline")


def _sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write_trace(
    out_dir: "str | Path",
    *,
    task: str,
    seed: str,
    agent_id: str,
    agent_class: str,
    poses: Sequence[Sequence[float]],
    agent_name: str = "",
    agent_meta: Optional[dict[str, Any]] = None,
    trial: int = 0,
    tier: str = "S",
    capture: str = "sim",
    events: Iterable[Sequence[Any]] = (),
    world_final: Optional[dict[str, Any]] = None,
    notes: str = "",
    decimals: int = 3,
) -> Path:
    """Write one run. ``poses`` rows are ``(t_s, x_m, y_m, theta_rad)``;
    ``events`` rows are ``(t_s, type, object_id, detail)``."""
    if agent_class not in AGENT_CLASSES:
        raise ValueError(f"agent_class must be one of {AGENT_CLASSES}")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    fmt = f"{{:.{decimals}f}}"
    with open(out / "poses.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_m", "y_m", "theta_rad"])
        for t, x, y, th in poses:
            w.writerow([f"{t:.3f}", fmt.format(x), fmt.format(y), fmt.format(th)])
    files = ["poses.csv"]

    events = list(events)
    if events:
        with open(out / "events.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "type", "object_id", "detail"])
            for t, kind, oid, detail in events:
                w.writerow([f"{t:.3f}", kind, oid, detail])
        files.append("events.csv")

    if world_final is not None:
        (out / "world_final.json").write_text(json.dumps(world_final, indent=1))
        files.append("world_final.json")

    manifest = {
        "spec_version": SPEC_VERSION,
        "task": task,
        "seed": seed,
        "agent": {"id": agent_id, "class": agent_class, "name": agent_name or agent_id, "meta": agent_meta or {}},
        "trial": trial,
        "tier": tier,
        "capture": capture,
        "notes": notes,
        "files": {name: _sha256(out / name) for name in files},
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return out
