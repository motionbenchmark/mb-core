# VEX V5 onboard logging (PROS)

`motionbench_log.hpp` logs your odometry pose to the SD card as a MotionBench
`poses.csv` at 50 Hz. It works with any library that can report x, y (inches)
and heading (degrees).

1. Copy the header into your project's `include/`.
2. Create a `Logger` with a lambda returning your pose (see the header comment).
3. Call `start("/usd/run.csv")` at the start of autonomous or a skills run, and
   `stop()` at the end.
4. Wrap the file into a trace:

```python
import csv, motionbench as mb
poses = [tuple(float(r[k]) for k in ("t_s","x_m","y_m","theta_rad")) for r in csv.DictReader(open("run.csv"))]
mb.write_trace("runs/auton-t0", task="MB-VEX-NAV@1.0", seed="dev-01",
               agent_id="team-1234a-auton", agent_class="human_auton", poses=poses,
               tier="F", capture="onboard")
```

Onboard logs record where the robot *thinks* it is. Odometry drift is invisible
to it, so leaderboard runs also need overhead video; the scorer can then compare
the two (planned for v0.2).

The pose source runs inside a PROS task: keep it fast and thread-safe.
