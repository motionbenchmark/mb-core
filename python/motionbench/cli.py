"""`motionbench` command line.

    motionbench tasks
    motionbench score RUN.mbtrace [--json]
    motionbench aggregate DATA_DIR -o leaderboard.json
    motionbench synth --out data/synthetic-v0.1
    motionbench field --vex -o field.json
    motionbench tags --config field.json --out tags/
    motionbench track run.mp4 --config field.json --task MB-VEX-NAV@1.0 --seed dev-01 \
        --agent-id driver-01 --agent-class human -o runs/driver-01-t0
    motionbench render RUN.mbtrace -o run.avi     # synthetic camera footage of a trace
    motionbench viz DATA_DIR -o explorer.html     # interactive explorer + independent checks
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
    g.add_argument("--room", nargs=2, type=float, metavar=("WIDTH_M", "HEIGHT_M"))
    p.add_argument("-o", "--out", default="field.json")

    p = sub.add_parser("tags", help="write printable AprilTag PNGs for a field config")
    p.add_argument("--config")
    p.add_argument("--out", default="tags")

    p = sub.add_parser("track", help="track a video into a .mbtrace")
    p.add_argument("video")
    p.add_argument("--config", help="field.json (default: VEX field)")
    p.add_argument("--task", required=True)
    p.add_argument("--seed", required=True)
    p.add_argument("--agent-id", required=True)
    p.add_argument("--agent-class", required=True, choices=["human", "human_auton", "ai", "hybrid", "product", "baseline"])
    p.add_argument("--agent-name", default="")
    p.add_argument("--trial", type=int, default=0)
    p.add_argument("--tier", default="F", choices=["S", "F", "V"])
    p.add_argument("-o", "--out", required=True)

    p = sub.add_parser("viz", help="write an interactive HTML explorer for a dataset (verification)")
    p.add_argument("root")
    p.add_argument("-o", "--out", default="explorer.html")
    p.add_argument("--title")

    p = sub.add_parser("render", help="render synthetic overhead video of a trace (for testing the tracker)")
    p.add_argument("trace")
    p.add_argument("--config")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("-o", "--out", default="run.avi")

    a = ap.parse_args(argv)
    try:
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

            cfg = FieldConfig.room(*a.room) if a.room else FieldConfig.vex()
            cfg.save(a.out)
            print(f"wrote {a.out}")
        elif a.cmd == "tags":
            from .tracker.tags import write_tags

            for p in write_tags(_field(a), a.out):
                print(p)
        elif a.cmd == "track":
            from .tracker import track_video

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

            out = write(a.root, a.out, title=a.title, tasks_dir=a.tasks_dir)
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
