"""Generate a tiny synthetic dataset, score it, and print a leaderboard.

    pip install motionbench
    python examples/quickstart.py
"""

import tempfile

import motionbench as mb
from motionbench.synth import generate

plan = {"MB-VEX-NAV@1.0": {"seeds": ["dev-01"], "trials": 3, "roster": "vex"}}

with tempfile.TemporaryDirectory() as d:
    generate(d, master_seed=2026, plan=plan)
    reports, errors = mb.score_dir(d)
    board = mb.aggregate(reports)

nav = board["tasks"][0]
print(f"\n{nav['task']}: every agent scored against the goal ({nav['max_score']:.0f} waypoints)")
if nav["people"]:
    p = nav["people"]
    print(f"Drivers and code use the same robot here, so we also compare with people: "
          f"median driver {p['median']:.1f} of {nav['max_score']:.0f} ({p['n_people']} drivers)")
print(f"\n{'rank':>4}  {'agent':<42} {'class':<12} {'goal':>5} {'worst':>5} {'index':>5} {'vs ppl':>6}")
for r in nav["rows"]:
    vs = f"{r['vs_people']:>6.2f}" if r["vs_people"] is not None else f"{'':>6}"
    print(f"{r['rank']:>4}  {r['agent_name']:<42} {r['class']:<12} {r['goal']:>5.0%} "
          f"{r['goal_worst']:>5.0%} {r['mbi']:>5.2f} {vs}")
print("\nSynthetic agents only - this illustrates the method, not real performance.")
