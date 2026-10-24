# match-sim-v0.1 (synthetic)

Five simulated 2 v 2 VEX V5 matches (seeds 1–5) with full ground truth: every robot's
pose at 20 Hz, every block's state changes, and an events log (pickups, deliveries,
robot and wall contacts, parking). **Synthetic data**: for building and testing match
trackers, not a measurement of real teams.

Regenerate exactly with `motionbench match synth -o data/match-sim-v0.1 --seeds 1-5`.
Videos are not stored; render one with
`motionbench match render data/match-sim-v0.1/m0001 -o m0001.mp4 --fps 10 --seed 1`.
Format and benchmark: [docs/match.md](../../docs/match.md). License: CC BY 4.0.
