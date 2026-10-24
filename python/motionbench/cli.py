"""`motionbench` command line.

    motionbench tasks
    motionbench score RUN.mbtrace [--json]
    motionbench aggregate DATA_DIR -o leaderboard.json
    motionbench synth --out data/synthetic-v0.1
    motionbench field --vex -o field.json
    motionbench tags --config field.json --out tags/
    motionbench track run.mp4 --config field.json --task MB-VEX-NAV@1.0 --seed dev-01 \
        --agent-id driver-01 --agent-class human -o runs/driver-01-t0
    motionbench calibrate run.mov -o calib.html   # tap the floor corners, save calib.json
    motionbench track run.mov --untagged --calib calib.json -o out/   # no tags needed
    motionbench render RUN.mbtrace -o run.avi     # synthetic camera footage of a trace
    motionbench viz DATA_DIR -o explorer.html     # interactive explorer + independent checks
    motionbench match synth|render|report|track|eval ...   # multi-robot matches (see `motionbench match -h`)
    motionbench game create|simulate|verdict|render ...    # games: code vs simulated drivers (see `motionbench game -h`)
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import motionbench as mb


def _print_report(r: dict) -> None:
    s, e = r["success"], r["efficiency"]
    print(f"{r['task']}  seed {r['seed']}  agent {r['agent_name']} ({r['agent_class']})")
    print(f"  success     {s['score']:.2f} / {s['max_score']:.0f}  ({s['completion'] * 100:.0f}%)" + ("  completed" if s["completed"] else ""))
    ttg = f"{e['time_to_goal_s']:.2f} s" if e["time_to_goal_s"] is not None else "-"
    print(f"  efficiency  path {e['path_length_m']:.2f} m, efficiency {e['path_efficiency']:.2f}, time-to-goal {ttg}")
    if r.get("coverage"):
        print(f"  coverage    {r['coverage']['covered_pct']:.1f}% covered, {r['coverage']['redundant_pct']:.1f}% redundant")
    print(f"  safety      {r['safety']['total']} violations")
    for w in r["warnings"]:
        print(f"  warning: {w}")


def _field(a) -> "object":
    from .tracker import FieldConfig

    if getattr(a, "config", None):
        return FieldConfig.load(a.config)
    return FieldConfig.vex()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="motionbench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action="version", version=f"motionbench {mb.__version__} (spec {mb.spec_version})")
    ap.add_argument("--tasks-dir", help="extra task specs (<ID@VERSION>.json)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("tasks", help="list tasks")

    p = sub.add_parser("score", help="score one .mbtrace directory")
    p.add_argument("trace")
    p.add_argument("--json", action="store_true")

    p = sub.add_parser("aggregate", help="score every trace under a directory and build a leaderboard")
    p.add_argument("root")
    p.add_argument("-o", "--out")
    p.add_argument("--reports", help="also write per-run reports (JSON lines)")

    p = sub.add_parser("synth", help="generate the synthetic dataset")
    p.add_argument("--out", default="data/synthetic-v0.1")
    p.add_argument("--seed", type=int, default=2026)

    p = sub.add_parser("field", help="write a tracker field config")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--vex", action="store_true", help="12 ft VEX field (default)")
    g.add_argument("--aim", action="store_true", help="6 x 4 ft VEX AIM table arena (tags 20-24)")
    g.add_argument("--room", nargs=2, type=float, metavar=("WIDTH_M", "HEIGHT_M"))
    p.add_argument("-o", "--out", default="field.json")

    p = sub.add_parser("tags", help="write printable AprilTag PNGs for a field config")
    p.add_argument("--config")
    p.add_argument("--out", default="tags")

    p = sub.add_parser("calibrate", help="make the empty-floor picture of a video and a page to tap the floor corners")
    p.add_argument("video")
    p.add_argument("-o", "--out", default="calib.html")
    p.add_argument("--png", help="also save the empty-floor picture")
    p.add_argument("--width", type=int, default=540, help="working width in pixels (default 540)")
    p.add_argument("--robot-diameter", type=float, default=0.35)
    p.add_argument("--start", type=float, default=0.0, help="ignore the video before this time (s), e.g. a broadcast intro")
    p.add_argument("--end", type=float, help="ignore the video after this time (s)")
    p.add_argument("--field", type=float, nargs=2, metavar=("WIDTH_M", "LENGTH_M"), help="pre-fill the measured floor size")
    p.add_argument("--vex-field", action="store_true", help="pre-fill a VEX V5 field (3.566 x 3.566 m inside the walls)")

    p = sub.add_parser("track", help="track a video into a .mbtrace (tags, or --untagged with a calib.json)")
    p.add_argument("video")
    p.add_argument("--config", help="field.json (default: VEX field)")
    p.add_argument("--untagged", action="store_true", help="no tags: needs --calib from `motionbench calibrate`")
    p.add_argument("--calib", help="calib.json (untagged mode)")
    p.add_argument("--task", help="task key (tagged mode); untagged mode writes its own coverage task")
    p.add_argument("--task-id", default="MB-FIELD-CAPTURE", help="untagged: id for the generated task")
    p.add_argument("--task-version", default="0.1")
    p.add_argument("--tool-width", type=float, default=0.30, help="untagged: cleaning width (m)")
    p.add_argument("--edge-reach", type=float, help="untagged: side-brush reach from the centre (m)")
    p.add_argument("--contact-tolerance", type=float, default=0.08, help="untagged: allowed wall contact (m)")
    p.add_argument("--fps", type=float, default=15.0, help="untagged: frames per second to analyse")
    p.add_argument("--overlay", action="store_true", help="untagged: also write overlay.mp4 to check the track")
    p.add_argument("--seed", default="field-01")
    p.add_argument("--agent-id", default="robot")
    p.add_argument("--agent-class", default="product", choices=["human", "human_auton", "ai", "hybrid", "product", "baseline"])
    p.add_argument("--agent-name", default="")
    p.add_argument("--trial", type=int, default=0)
    p.add_argument("--tier", default="F", choices=["S", "F", "V"])
    p.add_argument("--json", action="store_true", help="print a JSON summary (for scripts)")
    p.add_argument("-o", "--out", required=True)

    p = sub.add_parser("viz", help="write an interactive HTML explorer for a dataset (verification)")
    p.add_argument("root")
    p.add_argument("-o", "--out", default="explorer.html")
    p.add_argument("--title")
    p.add_argument("--embed", action="store_true", help="variant for an iframe on another site (light theme, no header)")

    p = sub.add_parser("render", help="render synthetic overhead video of a trace (for testing the tracker)")
    p.add_argument("trace")
    p.add_argument("--config")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("-o", "--out", default="run.avi")

    from .game.cli import add_parser as _game_parser
    from .match.cli import add_parser as _match_parser

    _match_parser(sub)
    _game_parser(sub)

    a = ap.parse_args(argv)
    try:
        if a.cmd == "match":
            from .match.cli import run as _match_run

            return _match_run(a)
        if a.cmd == "game":
            from .game.cli import run as _game_run

            return _game_run(a)
        if a.cmd == "tasks":
            for t in mb.tasks(a.tasks_dir):
                key = f"{t['id']}@{t['version']}"
                print(f"{key:<18} {t['goal']['kind']:<10} {t['domain']:<13} {t['budget']['time_s']:>5.0f}s  {t['title']}")
        elif a.cmd == "score":
            r = mb.score(a.trace, a.tasks_dir)
            print(json.dumps(r, indent=2)) if a.json else _print_report(r)
        elif a.cmd == "aggregate":
            reports, errors = mb.score_dir(a.root, a.tasks_dir)
            for e in errors:
                print(f"warning: {e['path']}: {e['error']}", file=sys.stderr)
            print(f"scored {len(reports)} runs ({len(errors)} failed)", file=sys.stderr)
            if a.reports:
                Path(a.reports).write_text("".join(json.dumps(r) + "\n" for r in reports))
            lb = json.dumps(mb.aggregate(reports, a.tasks_dir), indent=2)
            Path(a.out).write_text(lb) if a.out else print(lb)
        elif a.cmd == "synth":
            from .synth import generate

            n = generate(a.out, a.seed)
            print(f"wrote {n} traces to {a.out}")
        elif a.cmd == "field":
            from .tracker import FieldConfig

            cfg = FieldConfig.room(*a.room) if a.room else FieldConfig.aim() if a.aim else FieldConfig.vex()
            cfg.save(a.out)
            print(f"wrote {a.out}")
        elif a.cmd == "tags":
            from .tracker.tags import write_tags

            for p in write_tags(_field(a), a.out):
                print(p)
        elif a.cmd == "calibrate":
            from .tracker.calibrate import write_calibration_page

            defaults = {"robot_diameter_m": a.robot_diameter}
            size = (3.566, 3.566) if a.vex_field else a.field
            if size:
                defaults.update(measured_width_m=size[0], measured_length_m=size[1])
            out = write_calibration_page(a.video, a.out, width=a.width, png=a.png, defaults=defaults,
                                         start_s=a.start, end_s=a.end)
            print(f"wrote {out}: open it, tap the floor corners, and download calib.json")
        elif a.cmd == "track" and a.untagged:
            from .tracker.untagged import Calibration, track_untagged

            if not a.calib:
                raise ValueError("--untagged needs --calib calib.json (make one with `motionbench calibrate VIDEO`)")
            calib = Calibration.load(a.calib)
            out = Path(a.out)
            res = track_untagged(a.video, calib, fps=a.fps, tool_width_m=a.tool_width, edge_reach_m=a.edge_reach,
                                 contact_tolerance_m=a.contact_tolerance, overlay=out / "overlay.mp4" if a.overlay else None,
                                 progress=None if a.json else (lambda m: print(m, file=sys.stderr)))
            res.write(out, task_id=a.task_id, version=a.task_version, seed=a.seed, agent_id=a.agent_id,
                      agent_class=a.agent_class, agent_name=a.agent_name)
            key = f"{a.task_id}@{a.task_version}"
            report = mb.score(out / "trace", tasks_dir=out / "tasks")
            (out / "report.json").write_text(json.dumps(report, indent=1))
            summary = {"out": str(out), "task": key, "poses": len(res.poses), "camera": res.camera, "stats": res.stats,
                       "warnings": res.warnings, "coverage": report.get("coverage"), "success": report["success"],
                       "path_length_m": report["efficiency"]["path_length_m"], "safety": report["safety"]}
            if a.json:
                print(json.dumps(summary, default=float))
            else:
                for w in res.warnings:
                    print(f"warning: {w}", file=sys.stderr)
                _print_report(report)
                print(f"wrote {out}/ (tasks/, trace/, report.json{', overlay.mp4' if a.overlay else ''}); "
                      f"rescore with: motionbench --tasks-dir {out}/tasks score {out}/trace")
        elif a.cmd == "track":
            from .tracker import track_video

            if not a.task:
                raise ValueError("tagged tracking needs --task (or use --untagged)")
            res = track_video(a.video, _field(a))
            for w in res.warnings:
                print(f"warning: {w}", file=sys.stderr)
            if not res.poses:
                print("error: no poses tracked", file=sys.stderr)
                return 1
            out = res.write(a.out, task=a.task, seed=a.seed, agent_id=a.agent_id, agent_class=a.agent_class,
                            agent_name=a.agent_name, trial=a.trial, tier=a.tier)
            print(f"tracked {len(res.poses)} poses ({res.detection_rate:.0%} of frames), "
                  f"calibration RMS {res.calibration_rms_m * 100:.2f} cm -> {out}")
        elif a.cmd == "viz":
            from .viz import write

            out = write(a.root, a.out, title=a.title, tasks_dir=a.tasks_dir, embed=a.embed)
            print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB); open it in a browser")
        elif a.cmd == "render":
            from .tracker.render import render_video

            with open(Path(a.trace) / "poses.csv") as f:
                poses = [tuple(float(r[k]) for k in ("t_s", "x_m", "y_m", "theta_rad")) for r in csv.DictReader(f)]
            print(render_video(poses, _field(a), a.out, fps=a.fps))
    except (ValueError, KeyError, FileNotFoundError, ImportError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
