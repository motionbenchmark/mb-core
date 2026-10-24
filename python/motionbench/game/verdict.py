"""Code against drivers, in simulation: does this routine do better than people would?

Each routine runs many times (every run draws its own wheel slip, vision noise
and kick scatter). Each driver character plays the same game many times, and a
made-up class of drivers gives a picture of the typical student. Then every
comparison is a difference in mean points with a 95% bootstrap range:

  better       the whole range is above zero
  worse        the whole range is below zero
  can't tell   the range includes zero (more runs, or the two really are close)

Verdicts also run on generated layouts nobody has seen, so a routine that only learned
the public layout by heart shows it. (Official verdicts keep those layouts' seeds private.)

A verdict from simulation says what to try on the real robot first. It is not
a result: real fields, batteries and drivers differ from any model.
"""

from __future__ import annotations

import json
import statistics as st
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .driver import CHARACTERS, Driver, class_roster
from .run import RunResult, simulate, write_run
from .spec import Game


def _stats(runs: list[RunResult]) -> dict:
    p = [r.points for r in runs]
    fin = [r.finish_s for r in runs if r.finish_s is not None]
    return {
        "runs": len(runs),
        "mean_points": round(float(np.mean(p)), 3),
        "median_points": float(np.median(p)),
        "worst_points": float(min(p)),
        "best_points": float(max(p)),
        "full_score_runs": sum(1 for r in runs if r.points >= r.max_points),
        "mean_finish_s": round(float(np.mean(fin)), 2) if fin else None,
        "mean_path_m": round(float(np.mean([r.path_m for r in runs])), 2),
        "mean_bumps": round(float(np.mean([r.bumps for r in runs])), 2),
        "errors": sorted({r.error for r in runs if r.error}),
    }


def _diff_ci(a: list[float], b: list[float], rng: np.random.Generator, n: int = 4000) -> tuple[float, float, float]:
    a, b = np.asarray(a, float), np.asarray(b, float)
    ia = rng.integers(0, len(a), (n, len(a)))
    ib = rng.integers(0, len(b), (n, len(b)))
    d = a[ia].mean(1) - b[ib].mean(1)
    return float(a.mean() - b.mean()), float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


def _word(lo: float, hi: float) -> str:
    return "better" if lo > 0 else "worse" if hi < 0 else "can't tell"


def verdict(game: Game, routines: dict[str, str], *, layout: Optional[str] = None, runs: int = 20, driver_runs: int = 12,
            class_size: int = 20, class_runs: int = 3, drivers: Optional[list[Driver]] = None, seed: int = 0,
            unseen: int = 10, unseen_seed: int = 5000, out: Optional[str] = None,
            progress: Optional[Callable[[str], None]] = None) -> dict:
    """``unseen``: also play on this many generated layouts nobody has seen (seeds from
    ``unseen_seed``; official verdicts keep that seed private). 0 turns it off."""
    say = progress or (lambda m: None)
    rng = np.random.default_rng(seed)
    lay, _ = game.layout(layout)
    drivers = drivers or CHARACTERS
    outp = Path(out) if out else None

    def keep(r: RunResult, sub: str):
        if outp:
            write_run(r, outp / "runs" / sub / f"seed-{r.seed:03d}", game, score=False)

    code_runs: dict[str, list[RunResult]] = {}
    for name, code in routines.items():
        say(f"code {name}: {runs} runs")
        code_runs[name] = []
        for s in range(runs):
            r = simulate(game, code=code, code_name=name, layout=lay, seed=1000 + s)
            code_runs[name].append(r)
            keep(r, f"code-{Path(name).stem}")
    drv_runs: dict[str, list[RunResult]] = {}
    for d in drivers:
        say(f"driver {d.name}: {driver_runs} runs")
        drv_runs[d.id] = []
        for s in range(driver_runs):
            r = simulate(game, driver=d, layout=lay, seed=2000 + s)
            drv_runs[d.id].append(r)
            keep(r, f"driver-{d.id}")
    say(f"a class of {class_size} simulated drivers, {class_runs} runs each")
    klass = class_roster(class_size, seed=seed)
    class_means, class_all = [], []
    for d in klass:
        rs = [simulate(game, driver=d, layout=lay, seed=3000 + 17 * i) for i in range(class_runs)]
        class_means.append(float(np.mean([r.points for r in rs])))
        class_all += rs
    order = np.argsort(class_means)
    median_student = klass[int(order[len(order) // 2])]

    report = {
        "game": game.key, "title": game.raw["title"], "layout": lay, "max_points": game.max_points, "time_s": game.time_s,
        "simulated": True,
        "code": {n: _stats(rs) for n, rs in code_runs.items()},
        "drivers": {d.id: {"name": d.name, "blurb": d.blurb, "traits": d.as_dict(), **_stats(drv_runs[d.id])} for d in drivers},
        "class": {"size": class_size, "runs_each": class_runs, "median_of_means": float(np.median(class_means)),
                  "top_quarter": float(np.percentile(class_means, 75)), "bottom_quarter": float(np.percentile(class_means, 25)),
                  "median_student": median_student.as_dict()},
        "comparisons": {},
    }
    # the median student, run as often as the code, is the main yardstick
    med_runs = [simulate(game, driver=median_student, layout=lay, seed=4000 + s) for s in range(runs)]
    for n, rs in code_runs.items():
        a = [r.points for r in rs]
        comp = {}
        for label, others in [("median student", med_runs)] + [(drv_runs[d.id][0].agent_name, drv_runs[d.id]) for d in drivers]:
            diff, lo, hi = _diff_ci(a, [r.points for r in others], rng)
            comp[label] = {"diff_points": round(diff, 2), "ci95": [round(lo, 2), round(hi, 2)], "verdict": _word(lo, hi)}
        beats = [k for k, v in comp.items() if v["verdict"] == "better" and k != "median student"]
        loses = [k for k, v in comp.items() if v["verdict"] == "worse" and k != "median student"]
        m = comp["median student"]
        if m["verdict"] == "better":
            head = f"Better than the typical student driver by {m['diff_points']:+.1f} points (95% range {m['ci95'][0]:+.1f} to {m['ci95'][1]:+.1f})."
        elif m["verdict"] == "worse":
            head = f"Worse than the typical student driver by {abs(m['diff_points']):.1f} points (95% range {m['ci95'][0]:+.1f} to {m['ci95'][1]:+.1f})."
        else:
            head = f"Can't tell apart from the typical student driver ({m['diff_points']:+.1f} points, 95% range {m['ci95'][0]:+.1f} to {m['ci95'][1]:+.1f})."
        best = max(drivers, key=lambda d: np.mean([r.points for r in drv_runs[d.id]]))
        bv = comp[best.name]
        tail = (f" It also beats the best simulated driver, {best.name}." if bv["verdict"] == "better" else
                f" The best simulated driver, {best.name}, still does better." if bv["verdict"] == "worse" else
                f" Against the best simulated driver, {best.name}, it's too close to call.")
        report["comparisons"][n] = {"headline": head + tail, "vs": comp, "beats": beats, "loses_to": loses}
    if unseen and game.raw.get("layout_rules"):
        report["unseen"] = _unseen(game, routines, code_runs, drivers, drv_runs, median_student, runs, driver_runs, unseen,
                                   unseen_seed, rng, say)
        for n, u in report["unseen"]["comparisons"].items():
            report["comparisons"][n]["headline"] += " " + u["headline"]
    if outp:
        outp.mkdir(parents=True, exist_ok=True)
        (outp / "verdict.json").write_text(json.dumps(report, indent=1))
    return report


def _unseen(game, routines, code_runs, drivers, drv_runs, median_student, runs, driver_runs, n_layouts, base, rng, say) -> dict:
    """The same comparisons on generated layouts: does the code still hold up on a field it hasn't seen?"""
    lays = [f"gen-{base + i}" for i in range(n_layouts)]
    say(f"unseen layouts: {n_layouts} generated layouts")
    u_code = {n: [simulate(game, code=c, code_name=n, layout=lays[s % n_layouts], seed=6000 + s) for s in range(runs)]
              for n, c in routines.items()}
    u_drv = {d.id: [simulate(game, driver=d, layout=lays[s % n_layouts], seed=7000 + s) for s in range(driver_runs)] for d in drivers}
    u_med = [simulate(game, driver=median_student, layout=lays[s % n_layouts], seed=8000 + s) for s in range(runs)]
    best = max(drivers, key=lambda d: np.mean([r.points for r in u_drv[d.id]]))
    out = {"layouts": n_layouts, "seed_base": base, "code": {n: _stats(rs) for n, rs in u_code.items()},
           "drivers": {d.id: {"name": d.name, **_stats(u_drv[d.id])} for d in drivers},
           "median_student": _stats(u_med), "comparisons": {}}
    for n, rs in u_code.items():
        a = [r.points for r in rs]
        dm, lm, hm = _diff_ci(a, [r.points for r in u_med], rng)
        db, lb, hb = _diff_ci(a, [r.points for r in u_drv[best.id]], rng)
        dp, lp, hp = _diff_ci(a, [r.points for r in code_runs[n]], rng)
        w = _word(lm, hm)
        head = (f"On layouts it hasn't seen it scores {np.mean(a):.1f} (vs {np.mean([r.points for r in code_runs[n]]):.1f} on the public one): "
                f"{'better than' if w == 'better' else 'worse than' if w == 'worse' else 'level with'} the typical student there.")
        out["comparisons"][n] = {
            "headline": head,
            "vs median student": {"diff_points": round(dm, 2), "ci95": [round(lm, 2), round(hm, 2)], "verdict": w},
            "vs best driver": {"driver": best.name, "diff_points": round(db, 2), "ci95": [round(lb, 2), round(hb, 2)], "verdict": _word(lb, hb)},
            "change_from_public": {"diff_points": round(dp, 2), "ci95": [round(lp, 2), round(hp, 2)], "verdict": _word(lp, hp)},
        }
    return out


def print_verdict(r: dict) -> None:
    print(f"{r['title']} ({r['game']}, layout {r['layout']}) - SIMULATED, {r['time_s']:.0f} s, max {r['max_points']:.0f} points")
    rows = [(f"code: {n}", s) for n, s in r["code"].items()] + [(f"driver: {v['name']}", v) for v in r["drivers"].values()]
    print(f"{'':<34}{'mean':>6}{'median':>8}{'worst':>7}{'best':>6}{'all in':>8}{'path m':>8}{'bumps':>7}")
    for label, s in rows:
        print(f"{label[:33]:<34}{s['mean_points']:>6.2f}{s['median_points']:>8.1f}{s['worst_points']:>7.0f}{s['best_points']:>6.0f}"
              f"{s['full_score_runs']:>5}/{s['runs']:<2}{s['mean_path_m']:>8.1f}{s['mean_bumps']:>7.1f}")
    c = r["class"]
    print(f"class of {c['size']} simulated students: median {c['median_of_means']:.2f} points, middle half "
          f"{c['bottom_quarter']:.2f}-{c['top_quarter']:.2f}")
    for n, comp in r["comparisons"].items():
        print(f"\nVERDICT for {n}: {comp['headline']}")
        for k, v in comp["vs"].items():
            print(f"   vs {k:<16} {v['diff_points']:+.2f} points  [{v['ci95'][0]:+.2f}, {v['ci95'][1]:+.2f}]  {v['verdict']}")
        for e in r["code"][n]["errors"]:
            print(f"   note: the routine stopped with {e}")
    u = r.get("unseen")
    if u:
        print(f"\nUNSEEN LAYOUTS ({u['layouts']} generated, same start, barrels placed at random)")
        for n, c in u["comparisons"].items():
            s = u["code"][n]
            print(f"   {n:<22} mean {s['mean_points']:.2f}  change from public {c['change_from_public']['diff_points']:+.2f}  "
                  f"vs typical student {c['vs median student']['diff_points']:+.2f} ({c['vs median student']['verdict']})  "
                  f"vs {c['vs best driver']['driver']} {c['vs best driver']['diff_points']:+.2f} ({c['vs best driver']['verdict']})")
        print(f"   typical student on unseen layouts: {u['median_student']['mean_points']:.2f}")
