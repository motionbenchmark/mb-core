"""`motionbench match ...` subcommands.

    motionbench match synth -o data/match-sim --seeds 1-5          simulate matches (ground truth)
    motionbench match render data/match-sim/m0001 -o m0001.mp4      broadcast-style video + gt.csv
    motionbench match report data/match-sim/m0001 [--png paths.png] per-robot metrics by period
    motionbench calibrate match.mp4 --vex-field --width 1020 --start 9 -o calib.html   tap the field corners
    motionbench match track match.mp4 --calib calib.json --start 9 -o tracks.csv      baseline tracker (no tags)
    motionbench match from-tracks tracks.csv -o my-match/            tracks -> match folder (then `match report`)
    motionbench match eval data/match-sim/m0001 tracks.csv          score a tracker against ground truth
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _seeds(s: str) -> list[int]:
    out: list[int] = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def add_parser(sub) -> None:
    p = sub.add_parser("match", help="multi-robot matches: synth, render, report, track, eval")
    ms = p.add_subparsers(dest="match_cmd", required=True)

    q = ms.add_parser("synth", help="simulate 2 v 2 matches with full ground truth")
    q.add_argument("-o", "--out", required=True, help="dataset folder (one subfolder per match)")
    q.add_argument("--seeds", default="1", help="e.g. 1-5 or 3,7,9")

    q = ms.add_parser("render", help="render a match as broadcast-style overhead video (+ ground truth csv)")
    q.add_argument("match")
    q.add_argument("-o", "--out", required=True, help="video path (.mp4); ground truth goes to <name>.gt.csv")
    q.add_argument("--fps", type=float, default=15.0)
    q.add_argument("--width", type=int, help="frame width (default: the broadcast camera's 1020)")
    q.add_argument("--start", type=float, default=0.0, help="match time to start at (s)")
    q.add_argument("--end", type=float, help="match time to stop at (s)")
    q.add_argument("--intro", type=float, default=0.0, help="seconds of title card before the overhead shot")
    q.add_argument("--seed", type=int, default=0, help="look: lighting, people, noise")
    q.add_argument("--no-overlay", action="store_true")
    q.add_argument("--no-people", action="store_true")

    q = ms.add_parser("report", help="per-robot match metrics (distance, speed, zones, contacts) by period")
    q.add_argument("match")
    q.add_argument("--json", action="store_true")
    q.add_argument("--png", help="also draw every robot's path to this PNG")

    q = ms.add_parser("track", help="baseline multi-robot tracker for fixed-camera match video (no tags)")
    q.add_argument("video")
    q.add_argument("-o", "--out", required=True, help="tracks.csv (t_s,track_id,x_m,y_m,alliance)")
    q.add_argument("--camera", help="camera.json (frame_size, f_px, camera_height_m, floor_corners_px, field_m); default: the broadcast camera")
    q.add_argument("--calib", help="calib.json from `motionbench calibrate VIDEO --vex-field` (tap the four field corners)")
    q.add_argument("--fps", type=float, default=10.0)
    q.add_argument("--start", type=float, default=0.0, help="video time where the overhead shot starts (s)")
    q.add_argument("--end", type=float)
    q.add_argument("--match-start", type=float, help="video time of match t=0 (default: --start)")
    q.add_argument("--robots", type=int, default=4)
    q.add_argument("--overlay", help="write a video with the tracks drawn on it")

    q = ms.add_parser("from-tracks", help="turn tracks.csv into a match folder (so `match report` works on real video)")
    q.add_argument("tracks")
    q.add_argument("-o", "--out", required=True)
    q.add_argument("--robots", type=int, default=4)
    q.add_argument("--id", default="tracked", help="match id")
    q.add_argument("--name", action="append", default=[], metavar="ROBOT=NAME", help="e.g. red1=1234A (repeatable)")

    q = ms.add_parser("eval", help="score tracker output against a match's ground truth")
    q.add_argument("match")
    q.add_argument("tracks")
    q.add_argument("--gate", type=float, default=0.45, help="max distance for a track to count as a robot (m)")
    q.add_argument("--offset", type=float, default=0.0, help="add this to track times to get match time (s)")
    q.add_argument("--json", action="store_true")


def run(a) -> int:
    if a.match_cmd == "synth":
        from .io import write_match
        from .sim import simulate

        out = Path(a.out)
        for s in _seeds(a.seeds):
            res = simulate(s)
            d = write_match(res, out / f"m{s:04d}")
            n = sum(1 for e in res.events if e[2] == "score")
            print(f"{d}: 4 robots, {len(res.poses['red1'])} poses each, {n} deliveries, {len(res.events)} events")
    elif a.match_cmd == "render":
        from .render import render_match

        info = render_match(a.match, a.out, fps=a.fps, width=a.width, start_s=a.start, end_s=a.end, intro_s=a.intro, seed=a.seed,
                            overlay=not a.no_overlay, people=not a.no_people,
                            progress=lambda f: print(f"  {f:.0%}", file=sys.stderr))
        print(json.dumps(info, indent=1))
    elif a.match_cmd == "report":
        from .report import draw_paths, print_report, report

        r = report(a.match)
        print(json.dumps(r, indent=1)) if a.json else print_report(r)
        if a.png:
            draw_paths(a.match, a.png)
    elif a.match_cmd == "track":
        from .track import track_match

        if a.calib:
            from .io import camera_from_calib

            cam = camera_from_calib(a.calib)
            print(f"camera: f {cam['f_px']:.0f} px, {cam['camera_height_m']:.2f} m up, tilted {cam['tilt_deg']:.0f} deg", file=sys.stderr)
        else:
            cam = json.loads(Path(a.camera).read_text()) if a.camera else None
        info = track_match(a.video, a.out, camera=cam, fps=a.fps, start_s=a.start, end_s=a.end, match_start_s=a.match_start,
                           n_robots=a.robots, overlay=a.overlay, progress=lambda m: print(m, file=sys.stderr))
        print(json.dumps(info, indent=1))
    elif a.match_cmd == "from-tracks":
        from .io import match_from_tracks

        names = dict(n.split("=", 1) for n in a.name)
        d = match_from_tracks(a.tracks, a.out, n_robots=a.robots, match_id=a.id, names=names)
        meta = json.loads((d / "match.json").read_text())
        for r in meta["robots"]:
            print(f"{r['id']:>6}  {r['alliance']:<7} tracks {', '.join(r['tracks'])}")
        print(f"wrote {d}; next: motionbench match report {d}")
    elif a.match_cmd == "eval":
        from .eval import evaluate, print_eval

        r = evaluate(a.match, a.tracks, gate_m=a.gate, offset_s=a.offset)
        print(json.dumps(r, indent=1)) if a.json else print_eval(r)
    return 0
