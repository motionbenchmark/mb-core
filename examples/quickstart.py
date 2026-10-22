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
ref = nav["human_reference"]
print(f"\n{nav['task']}: human median {ref['median']:.1f} of {nav['max_score']:.0f} waypoints ({ref['n_drivers']} drivers)\n")
print(f"{'rank':>4}  {'agent':<42} {'class':<12} {'mean':>5} {'worst':>5} {'HNS':>5} {'MBI':>5}")
for r in nav["rows"]:
    print(f"{r['rank']:>4}  {r['agent_name']:<42} {r['class']:<12} {r['success_mean']:>5.1f} "
          f"{r['success_worst']:>5.1f} {r['hns']:>5.2f} {r['mbi']:>5.2f}")
print("\nSynthetic agents only - this illustrates the method, not real performance.")
