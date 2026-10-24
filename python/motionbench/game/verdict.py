"""Code against a simulated-driver baseline: does this routine do better than the
parameterized drivers would, before it runs on the robot?

Design of the comparison
------------------------
* **Paired runs.** Run *i* of every agent uses the same seed, so the same layout and the
  same draw of robot imperfections (wheel slip, drift, camera noise, kick scatter). A
  comparison is the mean of paired differences, which removes the variation the agents
  share.
* **Two settings.** The public layout, and ``unseen`` layouts generated from the game's
  layout rules (official verdicts keep their seed private).
* **Resampling unit.** On the public layout the bootstrap resamples paired runs. On
  unseen layouts it is hierarchical: resample layouts, then runs within each chosen
  layout, so the range reflects variation across layouts as well as runs.
* **Four outcomes**, with an equivalence margin ``margin`` (default 0.5 points):

    better / worse        the 95% range of the paired difference excludes zero
    equivalent            the 95% range lies inside [-margin, +margin]
    no difference found   neither: the range includes zero but is wider than the margin

  "No difference found" is not "the same": more runs may separate them.

The drivers are a parameterized, uncalibrated baseline (see driver.py). A verdict says
which version of the code to try on the robot first; it is not a comparison with real
people. The change of a routine between public and unseen layouts does not depend on
the drivers at all.
"""

from __future__ import annotations

import json
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
        "mean_points": round(float(np.mean(p)), 4),
        "median_points": float(np.median(p)),
        "worst_points": float(min(p)),
        "best_points": float(max(p)),
        "full_score_runs": sum(1 for r in runs if r.points >= r.max_points),
        "mean_finish_s": round(float(np.mean(fin)), 2) if fin else None,
        "mean_path_m": round(float(np.mean([r.path_m for r in runs])), 2),
        "mean_bumps": round(float(np.mean([r.bumps for r in runs])), 2),
        "errors": sorted({r.error for r in runs if r.error}),
    }


def paired_ci(a: list[float], b: list[float], groups: Optional[list[int]], rng: np.random.Generator,
              n: int = 4000) -> tuple[float, float, float]:
    """Mean paired difference a-b with a 95% bootstrap range. ``groups`` (layout of each
    pair) makes the bootstrap hierarchical: layouts first, then pairs within layouts."""
    d = np.asarray(a, float) - np.asarray(b, float)
    if groups is None:
        idx = rng.integers(0, len(d), (n, len(d)))
        boots = d[idx].mean(1)
    else:
        g = np.asarray(groups)
        ids = np.unique(g)
        members = [np.nonzero(g == k)[0] for k in ids]
        boots = np.empty(n)
        for i in range(n):
            pick = rng.integers(0, len(ids), len(ids))
            vals = [d[members[k]][rng.integers(0, len(members[k]), len(members[k]))] for k in pick]
            boots[i] = np.concatenate(vals).mean()
    return float(d.mean()), float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))


def unpaired_ci(a: list[float], b: list[float], rng: np.random.Generator, n: int = 4000) -> tuple[float, float, float]:
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = a[rng.integers(0, len(a), (n, len(a)))].mean(1) - b[rng.integers(0, len(b), (n, len(b)))].mean(1)
    return float(a.mean() - b.mean()), float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


def outcome(lo: float, hi: float, margin: float) -> str:
    if lo > 0:
        return "better"
    if hi < 0:
        return "worse"
    if -margin <= lo and hi <= margin:
        return "equivalent"
    return "no difference found"


def _cmp(a: list[RunResult], b: list[RunResult], groups, rng, margin) -> dict:
    diff, lo, hi = paired_ci([r.points for r in a], [r.points for r in b], groups, rng)
    return {"diff_points": round(diff, 3), "ci95": [round(lo, 3), round(hi, 3)], "verdict": outcome(lo, hi, margin)}


def _change(after: list[RunResult], before: list[RunResult], rng, margin) -> dict:
    """Different layouts, so not paired: an ordinary two-sample bootstrap."""
    diff, lo, hi = unpaired_ci([r.points for r in after], [r.points for r in before], rng)
    return {"diff_points": round(diff, 3), "ci95": [round(lo, 3), round(hi, 3)], "verdict": outcome(lo, hi, margin)}


def _phrase(c: dict, who: str) -> str:
    d, (lo, hi) = c["diff_points"], c["ci95"]
    rng_ = f"95% range {lo:+.2f} to {hi:+.2f}"
    return {
        "better": f"scores {abs(d):.2f} points more than {who} ({rng_})",
        "worse": f"scores {abs(d):.2f} points less than {who} ({rng_})",
        "equivalent": f"is equivalent to {who} within the margin ({d:+.2f}, {rng_})",
        "no difference found": f"shows no detectable difference from {who} ({d:+.2f}, {rng_})",
    }[c["verdict"]]


def verdict(game: Game, routines: dict[str, str], *, layout: Optional[str] = None, runs: int = 20,
            driver_runs: Optional[int] = None, class_size: int = 20, class_runs: int = 3,
            drivers: Optional[list[Driver]] = None, seed: int = 0, unseen: int = 10, unseen_seed: int = 5000,
            margin: float = 0.5, out: Optional[str] = None, progress: Optional[Callable[[str], None]] = None) -> dict:
    """``driver_runs`` defaults to ``runs`` so that every comparison is fully paired."""
    say = progress or (lambda m: None)
    rng = np.random.default_rng(seed)
    lay, _ = game.layout(layout)
    drivers = drivers or CHARACTERS
    driver_runs = driver_runs or runs
    outp = Path(out) if out else None
    use_unseen = bool(unseen and game.raw.get("layout_rules"))
    lays_u = [f"gen-{unseen_seed + i}" for i in range(unseen)] if use_unseen else []

    def keep(r: RunResult, sub: str):
        if outp:
            write_run(r, outp / "runs" / sub / f"{r.layout}-seed-{r.seed:04d}", game, score=False)

    # the yardstick: the median member of a parameterized class of drivers
    say(f"a class of {class_size} simulated drivers, {class_runs} runs each")
    klass = class_roster(class_size, seed=seed)
    class_means = [float(np.mean([simulate(game, driver=d, layout=lay, seed=3000 + 17 * i).points for i in range(class_runs)]))
                   for d in klass]
    median_student = klass[int(np.argsort(class_means)[len(class_means) // 2])]

    def play(settings: list[tuple[str, int]], label: str) -> dict:
        """Every agent on the same (layout, seed) list."""
        res = {"code": {}, "drivers": {}}
        for name, code in routines.items():
            say(f"{label}: code {name}, {len(settings)} runs")
            res["code"][name] = [simulate(game, code=code, code_name=name, layout=l, seed=s) for l, s in settings]
            for r in res["code"][name]:
                keep(r, f"{label}/code-{Path(name).stem}")
        for d in drivers:
            say(f"{label}: driver {d.name}")
            res["drivers"][d.id] = [simulate(game, driver=d, layout=l, seed=s) for l, s in settings[:driver_runs]]
            for r in res["drivers"][d.id]:
                keep(r, f"{label}/driver-{d.id}")
        res["median"] = [simulate(game, driver=median_student, layout=l, seed=s) for l, s in settings]
        return res

    pub_settings = [(lay, 1000 + s) for s in range(runs)]
    pub = play(pub_settings, "public")
    uns, uns_groups = None, None
    if use_unseen:
        uns_settings = [(lays_u[s % unseen], 6000 + s) for s in range(runs)]
        uns_groups = [s % unseen for s in range(runs)]
        uns = play(uns_settings, "unseen")

    def block(res, groups, name):
        a = res["code"][name]
        comp = {"typical student": _cmp(a, res["median"], groups, rng, margin)}
        for d in drivers:
            b = res["drivers"][d.id]
            k = min(len(a), len(b))
            comp[d.name] = _cmp(a[:k], b, groups[:k] if groups else None, rng, margin)
        return comp

    report = {
        "game": game.key, "title": game.raw["title"], "layout": lay, "max_points": game.max_points, "time_s": game.time_s,
        "simulated": True, "baseline": "parameterized simulated drivers (uncalibrated)",
        "design": {"runs": runs, "driver_runs": driver_runs, "paired": True, "margin_points": margin,
                   "bootstrap": "paired runs (public); layouts then runs (unseen)", "unseen_layouts": len(lays_u),
                   "unseen_seed_base": unseen_seed if use_unseen else None},
        "class": {"size": class_size, "runs_each": class_runs, "median_of_means": float(np.median(class_means)),
                  "top_quarter": float(np.percentile(class_means, 75)), "bottom_quarter": float(np.percentile(class_means, 25)),
                  "median_student": median_student.as_dict()},
        "public": {"code": {n: _stats(rs) for n, rs in pub["code"].items()},
                   "drivers": {d.id: {"name": d.name, "blurb": d.blurb, "traits": d.as_dict(), **_stats(pub["drivers"][d.id])}
                               for d in drivers},
                   "typical_student": _stats(pub["median"]),
                   "comparisons": {n: block(pub, None, n) for n in routines}},
        "comparisons": {},
    }
    if uns:
        report["unseen"] = {"code": {n: _stats(rs) for n, rs in uns["code"].items()},
                            "drivers": {d.id: {"name": d.name, **_stats(uns["drivers"][d.id])} for d in drivers},
                            "typical_student": _stats(uns["median"]),
                            "comparisons": {n: block(uns, uns_groups, n) for n in routines},
                            # within-routine change: does not involve the drivers at all
                            "change_from_public": {n: _change(uns["code"][n], pub["code"][n], rng, margin) for n in routines}}
    for n in routines:
        p = report["public"]["comparisons"][n]["typical student"]
        head = f"On the public layout it {_phrase(p, 'the typical simulated driver')}."
        if uns:
            u = report["unseen"]["comparisons"][n]["typical student"]
            ch = report["unseen"]["change_from_public"][n]
            ch = report["unseen"]["change_from_public"][n]
            word = {"better": "rises", "worse": "drops", "equivalent": "holds", "no difference found": "shows no detectable change"}[ch["verdict"]]
            head += (f" On unseen layouts it {_phrase(u, 'the typical simulated driver')}. Its own score {word} from public to unseen"
                     f" ({ch['diff_points']:+.2f}, 95% range {ch['ci95'][0]:+.2f} to {ch['ci95'][1]:+.2f}).")
        report["comparisons"][n] = {"headline": head}
    if outp:
        outp.mkdir(parents=True, exist_ok=True)
        (outp / "verdict.json").write_text(json.dumps(report, indent=1))
    return report


def print_verdict(r: dict) -> None:
    d = r["design"]
    print(f"{r['title']} ({r['game']}) - SIMULATED, {r['time_s']:.0f} s, max {r['max_points']:.0f} points")
    print(f"baseline: {r['baseline']}; {d['runs']} paired runs per agent; equivalence margin +/-{d['margin_points']} points")
    for setting in ("public", "unseen"):
        if setting not in r:
            continue
        S = r[setting]
        title = f"public layout {r['layout']}" if setting == "public" else f"{d['unseen_layouts']} unseen layouts"
        print(f"\n{title.upper()}")
        print(f"{'':<34}{'mean':>7}{'worst':>7}{'best':>6}{'all in':>8}")
        rows = [(f"code: {n}", s) for n, s in S["code"].items()] + [("typical student", S["typical_student"])] + \
               [(f"driver: {v['name']}", v) for v in S["drivers"].values()]
        for label, s in rows:
            print(f"{label[:33]:<34}{s['mean_points']:>7.2f}{s['worst_points']:>7.0f}{s['best_points']:>6.0f}{s['full_score_runs']:>5}/{s['runs']:<2}")
        for n, comp in S["comparisons"].items():
            print(f"  {n}:")
            for k, v in comp.items():
                print(f"     vs {k:<16} {v['diff_points']:+.2f}  [{v['ci95'][0]:+.2f}, {v['ci95'][1]:+.2f}]  {v['verdict']}")
        if setting == "unseen":
            for n, c in S["change_from_public"].items():
                print(f"  {n}: change from public {c['diff_points']:+.2f} [{c['ci95'][0]:+.2f}, {c['ci95'][1]:+.2f}] {c['verdict']}")
    for n, c in r["comparisons"].items():
        print(f"\nVERDICT {n}: {c['headline']}")
