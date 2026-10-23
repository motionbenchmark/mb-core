# MotionBench field runs v0.1

**Real runs, unverified (tier F).** Each run keeps its own task spec in `tasks/`.

| Run | Task | Capture | Result |
| --- | --- | --- | --- |
| `robot-vacuum-hallway` | `MB-FIELD-HALLWAY@0.1` (1.28 m x 2.96 m home hallway, coverage, 30 cm tool) | One hand-held iPhone video, no tags, 137 s tracked | 85.0% of visible floor covered, 50.7% re-swept, 18.6 m driven |

How the hallway run was measured: frames aligned to the first one (ORB +
homography), robot found against a median empty-floor image, floor geometry from
the floorboards' vanishing point and the back-wall line, scale from the robot's own
size (35 cm diameter, 10 cm tall; focal length and camera height fitted to 405
silhouettes). Floor the camera never saw is marked `not-in-view` and excluded;
walls are `wall`. Heading comes from the direction of travel. No human baseline
exists for this hallway yet, so there is no human-normalized score.

This is one clip from one home: a test of the method, not a review of any product.

    motionbench --tasks-dir data/field-v0.1/tasks aggregate data/field-v0.1 -o results/leaderboard-field-v0.1.json

License: CC BY 4.0.
