"""Code against drivers, in simulation: does this routine do better than people would?

Each routine runs many times (every run draws its own wheel slip, vision noise
and kick scatter). Each driver character plays the same game many times, and a
made-up class of drivers gives a picture of the typical student. Then every
comparison is a difference in mean points with a 95% bootstrap range:

  better       the whole range is above zero
  worse        the whole range is below zero
  can't tell   the range includes zero (more runs, or the two really are close)

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
            out: Optional[str] = None, progress: Optional[Callable[[str], None]] = None) -> dict:
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
    if outp:
        outp.mkdir(parents=True, exist_ok=True)
        (outp / "verdict.json").write_text(json.dumps(report, indent=1))
    return report


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
