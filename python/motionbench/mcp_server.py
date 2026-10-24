"""MotionBench MCP server: convert, score and submit runs from an AI assistant.

Runs locally over stdio, so it can read videos and files on your computer.

    pip install "motionbench[tracker,mcp]"
    motionbench-mcp            # add this command to your assistant's MCP settings

Claude Desktop / Claude Code (``claude_desktop_config.json`` or ``.mcp.json``)::

    {"mcpServers": {"motionbench": {"command": "motionbench-mcp",
                                     "env": {"MOTIONBENCH_SITE": "https://motionbench.org"}}}}

Tools
  list_tasks         the benchmark tasks
  inspect_video      duration/size, plus the empty-floor picture and a page to tap the floor corners
  make_calibration   write calib.json from four floor corners (pixel coordinates on that picture)
  convert_video      no-tag tracking -> task + .mbtrace + score (+ overlay video)
  score_run          score a .mbtrace
  submit_run         upload a converted run (or a raw video) to the MotionBench site for review
  submission_status  check a submission
"""

from __future__ import annotations

import json
import os
import tempfile
import zipfile
from pathlib import Path
from typing import Any, Optional

import motionbench as mb

SITE = os.environ.get("MOTIONBENCH_SITE", "https://motionbench.org").rstrip("/")
VIDEO_EXT = {".mov", ".mp4", ".m4v", ".webm", ".avi", ".mkv"}

try:  # mcp >= 2
    from mcp.server.mcpserver import MCPServer as _Server
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server  # type: ignore

server = _Server(
    "motionbench",
    instructions=(
        "Tools for MotionBench, an open benchmark that scores how much of a task's goal a run "
        "achieves in a bounded space (VEX fields, rooms, hallways), whether a person, robot code, "
        "AI-written code or an assisted driver ran it. To score a phone video "
        "without tags: inspect_video -> ask the user to open the calibration page and tap the floor "
        "corners (or read corners off the picture) -> make_calibration -> convert_video -> show the "
        "score -> submit_run only after the user agrees and confirms they may share the footage."
    ),
)


def _p(path: str) -> Path:
    p = Path(os.path.expanduser(path)).resolve()
    if not p.exists():
        raise FileNotFoundError(f"not found: {p}")
    return p


@server.tool()
def list_tasks() -> list[dict[str, Any]]:
    """List the built-in benchmark tasks (id, title, goal, arena size, time budget)."""
    out = []
    for t in mb.tasks():
        out.append({"task": f"{t['id']}@{t['version']}", "title": t["title"], "goal": t["goal"]["kind"],
                    "arena_m": [t["arena"]["width_m"], t["arena"]["height_m"]], "budget_s": t["budget"]["time_s"]})
    return out


@server.tool()
def inspect_video(video_path: str, out_dir: Optional[str] = None, robot_diameter_m: float = 0.35) -> dict[str, Any]:
    """Look at a video and prepare calibration: writes the empty-floor picture (robot removed)
    and an offline page where the user taps the floor's four corners and downloads calib.json."""
    from .tracker.calibrate import write_calibration_page
    from .tracker.video import probe

    v = _p(video_path)
    info = probe(v)
    out = Path(out_dir).expanduser() if out_dir else v.parent / f"{v.stem}-motionbench"
    out.mkdir(parents=True, exist_ok=True)
    page = write_calibration_page(v, out / "calibrate.html", png=out / "empty-floor.png",
                                  defaults={"robot_diameter_m": robot_diameter_m})
    return {"video": str(v), "width": info.width, "height": info.height, "duration_s": round(info.duration_s, 2),
            "fps": round(info.fps, 2), "rotation_deg": info.rotation_deg, "calibration_page": str(page),
            "empty_floor_png": str(out / "empty-floor.png"), "working_frame_size": [540, round(info.height * 540 / info.width)],
            "next": "Open calibration_page in a browser, tap far-left, far-right, near-right, near-left corners of a "
                    "floor rectangle, add a tape-measured width if known, download calib.json, then call convert_video."}


@server.tool()
def make_calibration(out_path: str, frame_size: list[int], floor_corners_px: list[list[float]],
                     floor_polygon_px: Optional[list[list[float]]] = None, measured_width_m: Optional[float] = None,
                     measured_length_m: Optional[float] = None, robot_diameter_m: float = 0.35,
                     robot_height_m: float = 0.10, robot_polarity: str = "dark", max_y_px: Optional[float] = None) -> dict:
    """Write calib.json directly. Corners are pixels on the empty-floor picture, in the order
    far-left, far-right, near-right, near-left; frame_size is that picture's [width, height]."""
    from .tracker.untagged import Calibration

    if len(floor_corners_px) != 4:
        raise ValueError("need exactly four corners: far-left, far-right, near-right, near-left")
    c = Calibration(frame_size=list(frame_size), floor_corners_px=floor_corners_px, floor_polygon_px=floor_polygon_px,
                    measured_width_m=measured_width_m, measured_length_m=measured_length_m,
                    robot_diameter_m=robot_diameter_m, robot_height_m=robot_height_m, robot_polarity=robot_polarity,
                    max_y_px=max_y_px)
    p = Path(out_path).expanduser()
    c.save(p)
    return {"calib": str(p)}


@server.tool()
def convert_video(video_path: str, calib_path: str, out_dir: Optional[str] = None, tool_width_m: float = 0.30,
                  edge_reach_m: Optional[float] = None, contact_tolerance_m: float = 0.08, overlay: bool = True,
                  agent_class: str = "product", agent_name: str = "", task_id: str = "MB-FIELD-CAPTURE") -> dict:
    """Track the robot in an untagged video and score it as floor coverage. Writes tasks/, trace/,
    report.json and (optionally) overlay.mp4 for checking the track. Takes a few minutes."""
    from .tracker.untagged import Calibration, track_untagged

    v, c = _p(video_path), _p(calib_path)
    out = Path(out_dir).expanduser() if out_dir else v.parent / f"{v.stem}-motionbench"
    res = track_untagged(v, Calibration.load(c), tool_width_m=tool_width_m, edge_reach_m=edge_reach_m,
                         contact_tolerance_m=contact_tolerance_m, overlay=out / "overlay.mp4" if overlay else None)
    res.write(out, task_id=task_id, seed="field-01", agent_id=(agent_name or "robot").lower().replace(" ", "-")[:40],
              agent_class=agent_class, agent_name=agent_name or "robot", agent_meta={"source_video": v.name})
    report = mb.score(out / "trace", tasks_dir=out / "tasks")
    (out / "report.json").write_text(json.dumps(report, indent=1))
    return {"out_dir": str(out), "coverage": report.get("coverage"), "path_length_m": report["efficiency"]["path_length_m"],
            "duration_s": res.stats["duration_s"], "calibration": {k: res.camera[k] for k in ("mode", "rectangle_m", "tilt_deg")},
            "warnings": res.warnings, "overlay": str(out / "overlay.mp4") if overlay else None}


@server.tool()
def score_run(trace_dir: str, tasks_dir: Optional[str] = None) -> dict:
    """Score a .mbtrace directory with the official scorer (same numbers as the leaderboard)."""
    return mb.score(_p(trace_dir), tasks_dir=str(_p(tasks_dir)) if tasks_dir else None)


def _bundle(path: Path) -> Path:
    if path.is_file():
        return path
    tmp = Path(tempfile.mkdtemp()) / f"{path.name}.zip"
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for sub in ("tasks", "trace"):
            for f in sorted((path / sub).glob("*")):
                z.write(f, f"{sub}/{f.name}")
        for name in ("report.json", "calibration.json", "overlay.mp4", "background.png"):
            if (path / name).exists():
                z.write(path / name, name)
    return tmp


@server.tool()
def submit_run(path: str, name: str, email: str, title: str, consent_owner: bool, consent_public: bool,
               description: str = "", robot: str = "", agent_class: str = "product",
               calib_path: Optional[str] = None) -> dict:
    """Submit to the MotionBench site for review. ``path`` is a convert_video output folder, a .zip of
    one, or a raw video (then attach calib_path if you have it). Only call this after the user has
    agreed, filmed the footage themselves (or has permission), and is fine with it being published."""
    import httpx

    if not (consent_owner and consent_public):
        raise ValueError("the user must confirm they own the footage and agree to publication")
    p = _p(path)
    f = _bundle(p)
    kind = "video" if f.suffix.lower() in VIDEO_EXT else "run"
    data = {"name": name, "email": email, "title": title, "description": description, "robot": robot,
            "agent_class": agent_class, "kind": kind, "consent_owner": "true", "consent_public": "true", "source": "mcp"}
    if calib_path:
        data["calib"] = _p(calib_path).read_text()
    with open(f, "rb") as fh:
        r = httpx.post(f"{SITE}/api/submissions", data=data, files={"file": (f.name, fh)}, timeout=httpx.Timeout(600, connect=20))
    try:
        body = r.json()
    except ValueError:
        body = {"error": r.text[:300]}
    if r.status_code >= 400:
        raise RuntimeError(f"submission failed ({r.status_code}): {body.get('error', body)}")
    return body


@server.tool()
def submission_status(submission_id: str, token: str) -> dict:
    """Check a submission's review status (token comes from submit_run)."""
    import httpx

    r = httpx.get(f"{SITE}/api/submissions/{submission_id}", params={"token": token}, timeout=30)
    return r.json()


def main() -> None:
    server.run()


if __name__ == "__main__":
    main()
