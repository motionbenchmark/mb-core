"""`motionbench game ...`: games are created once, then simulated and scored many times.

    motionbench game create games/my-game --id MY-GAME --title "..."   start a definition (then fill it in)
    motionbench game validate games/aim-sort                          check a definition
    motionbench game drivers                                          the simulated driver characters
    motionbench game simulate games/aim-sort --code my_routine.py -o runs/try1 [--video try1.mp4]
    motionbench game simulate games/aim-sort --driver pro-priya -o runs/priya
    motionbench game score games/aim-sort runs/try1                   game points for a run folder
    motionbench game verdict games/aim-sort --code v1.py --code v2.py  code vs simulated drivers
    motionbench game render games/aim-sort runs/try1 -o try1.mp4
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def add_parser(sub) -> None:
    p = sub.add_parser("game", help="games: create once, then simulate code or drivers, score, verdict, render")
    gs = p.add_subparsers(dest="game_cmd", required=True)

    q = gs.add_parser("create", help="write a new game definition to fill in")
    q.add_argument("dir")
    q.add_argument("--id", required=True)
    q.add_argument("--title", required=True)
    q.add_argument("--platform", default="vex-aim")
    q.add_argument("--field", type=float, nargs=2, default=(1.8288, 1.2192), metavar=("WIDTH_M", "HEIGHT_M"))
    q.add_argument("--time", type=float, default=60.0)
    q.add_argument("--description", default="")

    q = gs.add_parser("validate", help="check a game definition")
    q.add_argument("game")

    gs.add_parser("drivers", help="list the simulated driver characters")

    q = gs.add_parser("simulate", help="simulate one run (or several) of a routine or a driver")
    q.add_argument("game")
    g = q.add_mutually_exclusive_group(required=True)
    g.add_argument("--code", help="a VEXcode AIM Python file")
    g.add_argument("--driver", help="a driver character id (see `game drivers`)")
    q.add_argument("--layout")
    q.add_argument("--seed", type=int, default=0)
    q.add_argument("--runs", type=int, default=1)
    q.add_argument("--noise", type=float, default=1.0, help="how imperfect the robot is (0 = perfect, 1 = default)")
    q.add_argument("-o", "--out", help="run folder (with --runs > 1: one subfolder per run)")
    q.add_argument("--video", help="also render the (first) run to this .mp4")
    q.add_argument("--json", action="store_true")

    q = gs.add_parser("score", help="game points for a run folder (simulated, or tracked from video)")
    q.add_argument("game")
    q.add_argument("run")

    q = gs.add_parser("verdict", help="run routines and simulated drivers many times and compare")
    q.add_argument("game")
    q.add_argument("--code", action="append", required=True, help="a routine (repeat to compare versions)")
    q.add_argument("--layout")
    q.add_argument("--runs", type=int, default=20)
    q.add_argument("--driver-runs", type=int, default=12)
    q.add_argument("--class-size", type=int, default=20)
    q.add_argument("-o", "--out", help="folder for verdict.json and every run")
    q.add_argument("--json", action="store_true")

    q = gs.add_parser("render", help="render a run folder as video")
    q.add_argument("game")
    q.add_argument("run")
    q.add_argument("-o", "--out", required=True)
    q.add_argument("--fps", type=float, default=20.0)
    q.add_argument("--title")


def run(a) -> int:
    from .spec import Game, create_game

    if a.game_cmd == "create":
        p = create_game(a.dir, id=a.id, title=a.title, platform=a.platform, width_m=a.field[0], height_m=a.field[1],
                        time_s=a.time, description=a.description)
        print(f"wrote {p}: add zones, objects, layouts and scoring rules, then `motionbench game validate {a.dir}`")
        return 0
    if a.game_cmd == "drivers":
        from .driver import CHARACTERS

        for d in CHARACTERS:
            print(f"{d.id:<15} {d.name:<15} reaction {d.reaction_s:.2f}s  speed {d.speed_use:.0%}  shake {d.steer_shake_deg:.0f}deg  "
                  f"precision {d.precision:.2f}  decisions {d.decisions:.2f}  mistakes {d.mistakes:.0%}\n{'':<16}{d.blurb}")
        return 0
    if a.game_cmd == "validate":
        import json as _j
        raw = _j.loads((Path(a.game) / "game.json" if Path(a.game).is_dir() else Path(a.game)).read_text())
        errs = Game(raw).validate()
        if errs:
            for e in errs:
                print(f"error: {e}", file=sys.stderr)
            return 1
        g = Game(raw)
        print(f"{g.key}: {g.raw['title']} ({g.raw['platform']}), {len(g.raw['layouts'])} layouts, "
              f"{len(g.raw['scoring']['rules'])} scoring rules, max {g.max_points:g} points in {g.time_s:g} s: ok")
        return 0
    game = Game.load(a.game)
    if a.game_cmd == "simulate":
        from .driver import by_id
        from .run import simulate, write_run

        code = Path(a.code).read_text() if a.code else None
        drv = by_id(a.driver) if a.driver else None
        results = []
        for i in range(a.runs):
            r = simulate(game, code=code, code_name=Path(a.code).name if a.code else "", driver=drv, layout=a.layout,
                         seed=a.seed + i, noise=a.noise)
            if a.out:
                d = Path(a.out) if a.runs == 1 else Path(a.out) / f"seed-{r.seed:03d}"
                write_run(r, d, game)
                if a.video and i == 0:
                    from .render import render_run
                    render_run(game, d, a.video)
            results.append(r)
            if not a.json:
                fin = f", all done at {r.finish_s:.1f} s" if r.finish_s else ""
                err = f"  [{r.error}]" if r.error else ""
                print(f"seed {r.seed}: {r.points:g} / {r.max_points:g} points{fin}, path {r.path_m:.1f} m, {r.bumps} bumps{err}")
        if a.json:
            print(json.dumps([r.summary() for r in results], indent=1))
        if a.video and not a.out:
            print("note: --video needs -o (the run folder it renders from)", file=sys.stderr)
        return 0
    if a.game_cmd == "score":
        rd = Path(a.run)
        if (rd / "run.json").exists():
            objs = json.loads((rd / "run.json").read_text())["objects"]
        else:  # a trace folder: final object positions from world_final.json, kinds from the game's layout
            trace = rd / "trace" if (rd / "trace").exists() else rd
            man = json.loads((trace / "manifest.json").read_text())
            _, lay = game.layout(man.get("seed"))
            kinds = {o["id"]: o["kind"] for o in lay["objects"]}
            objs = [dict(o, kind=kinds[o["id"]]) for o in json.loads((trace / "world_final.json").read_text())["objects"]]
        s = game.score_world(objs)
        for o in s["objects"]:
            print(f"  {o['id']:<6} {o['kind']:<15} {('in ' + o['zone']) if o['zone'] else ('held' if o['held'] else '-'):<12} {o['points']:g}")
        print(f"{s['points']:g} / {s['max_points']:g} points")
        return 0
    if a.game_cmd == "verdict":
        from .verdict import print_verdict, verdict

        routines = {Path(c).name: Path(c).read_text() for c in a.code}
        r = verdict(game, routines, layout=a.layout, runs=a.runs, driver_runs=a.driver_runs, class_size=a.class_size, out=a.out,
                    progress=None if a.json else (lambda m: print(m, file=sys.stderr)))
        print(json.dumps(r, indent=1)) if a.json else print_verdict(r)
        return 0
    if a.game_cmd == "render":
        from .render import render_run

        print(render_run(game, a.run, a.out, fps=a.fps, title=a.title))
        return 0
    return 1
