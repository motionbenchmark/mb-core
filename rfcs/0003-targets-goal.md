# RFC-0003: Targets goal (knock-down tasks)

| Field | Value |
| --- | --- |
| RFC | 0003 |
| Status | **Draft — open for comment** (implemented in scorer 0.1.0, spec 0.2) |
| Type | Standards Track |
| Created | 2026-10-25 |
| Author | Shekhar (23v) |
| Discussion | GitHub Discussions, category "RFCs" |
| License | CC BY 4.0 (this document) |

## Summary

Add a fourth goal kind, `targets`: knock down each target in the layout. A run's success is
the number of targets that fell for an accepted reason. Four tasks use it or come with it:
MB-AIM-TAG1@0.1, MB-AIM-TAG2@0.1 and MB-AIM-TAG2B@0.1 (targets), and MB-AIM-SORT18@0.1
(delivery), all on the 18 in VEX AIM field.

## Motivation

Classroom AIM games often ask a robot to knock something over: "grab the ball and use it to
knock down the AprilTag", without ramming it. None of the existing goals fits. Waypoints
reward being somewhere; delivery rewards where objects end up, and a fallen sign barely moves.
What counts is *that* the target fell and *what* knocked it over.

## Detailed design

```json
"goal": { "kind": "targets", "valid_by": ["ball"] },
"layouts": { "photo": { "start": {...}, "objects": [ { "id": "T1", "x": 0.221, "y": 0.219 } ] } }
```

- The layout's `objects` are the targets. A task MUST list at least one per layout.
- A trace reports a fall as an event `target_down` with `object_id` = the target and
  `detail` = what knocked it down (`ball`, `rammed`, `carried`, an object kind...).
- A target counts once, at its first `target_down` event whose `detail` is in `valid_by`.
  An empty `valid_by` accepts any cause.
- `success.score` = targets counted, `max_score` = targets in the layout, `completed` when all
  are counted, `time_to_goal_s` = the last counted fall when completed.
- Path efficiency: the oracle is a nearest-first tour from the start through the counted
  targets. An agent that knocks targets over from a distance can beat it; efficiency is capped
  at 1, as for every goal.
- Forbidden ways of playing (ramming the sign, hitting a barrel) are reported as `rule_flag`
  events and count in Safety, unchanged. A game's own rule that a violation *fails* the run
  stays in the game's score; the MotionBench goal score does not zero itself.

## Capture

Simulated runs emit `target_down` with the cause. Real runs need a judge: the overhead video
shows whether a sign fell and whether a rolling ball or the robot hit it. Until a tracker
detects falls, event-only scoring is lower confidence than the world-state scoring delivery
uses, and verified-tier runs SHOULD come with video.

## Compatibility

New goal kind; existing tasks, traces and reports are unchanged. A scorer older than this RFC
rejects targets tasks as unknown.

## Unresolved questions

1. Should a fall caused by the robot count partially, rather than not at all?
2. Should the oracle start from the nearest point a ball could be kicked from, not the target?
3. How should a tracker decide that a sign fell (height change, visible tag face)?
