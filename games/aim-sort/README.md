# AIM-SORT: VEX AIM barrel sorting (simulated game)

One VEX AIM robot sorts three orange and three blue barrels into the zone of
their colour, in 45 seconds, on a 6 ft x 4 ft table. Same layouts as MotionBench
task MB-AIM-SORT@0.1. Defined once in `game.json`; see [docs/games.md](../../docs/games.md).

```bash
motionbench game simulate games/aim-sort --code games/aim-sort/routines/planner_v4.py --layout dev-01 -o runs/v4 --video v4.mp4
motionbench game verdict games/aim-sort --layout dev-01 \
  --code games/aim-sort/routines/vision_v2.py --code games/aim-sort/routines/planner_v4.py
```

Sample routines (all written for layout dev-01; the robot starts mid near wall, facing the far wall):

| file | idea |
|---|---|
| `routines/deadreckon_v1.py` | fixed plan, no sensors |
| `routines/vision_v2.py` | AI Vision finds barrels; odometry back to the zone |
| `routines/vision_tags_v3.py` | v2 plus homing on the AprilTag behind each zone |
| `routines/planner_v4.py` | planned order, full speed, vision for the last few cm, spread drop spots |

Not an official VEX game. Everything produced from it is simulated.
