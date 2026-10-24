# RFC-0002: Goal-referenced scoring

| Field | Value |
| --- | --- |
| RFC | 0002 |
| Status | **Draft — open for comment** (implemented in scorer 0.1.0, spec 0.2) |
| Type | Standards Track |
| Created | 2026-10-23 |
| Author | Shekhar (23v) |
| Discussion | GitHub Discussions, category "RFCs" |
| Supersedes | RFC-0001 Part II §5 (aggregation) and the related rationale |
| License | CC BY 4.0 (this document) |

The key words MUST, MUST NOT, SHOULD and MAY are to be interpreted as described in RFC 2119.

## Summary

Every run is scored against the **task's goal**, not against people. The headline score
becomes the share of the goal achieved, from 0 to 100%, for every kind of agent: a person,
team-written code, AI-written code, a person with robot assistance, a commercial robot.
A comparison with people is kept only as an optional extra, on tasks that declare a person
can do exactly the same job with the same means. The MotionBench Index is redefined on
goal-based terms, and the cross-task overall ranking is removed.

## Motivation

Spec 0.1 normalized every score to the median person on the task (HNS = 1.0 for the median
person). That works when a person and a machine do the same job with the same means: a
student driving a VEX robot and the team's autonomous code driving the same robot. It does
not work in general:

- **Different means.** A robot vacuum and a person with a stick vacuum differ in tool width,
  reach, suction, speed and endurance. Normalizing the robot to the person turns a
  difference in equipment into a score that looks like skill.
- **Jobs no person does the same way.** Many robot tasks have no meaningful like-for-like
  human run, and some datasets contain only people or only robots. Spec 0.1 left those
  tasks without a headline score.
- **Saturation looks like superiority.** Where most people finish the job, the human median
  sits near the maximum, and HNS values above 1.0 mostly mean "finished every time".
- **It made people the reference rather than participants.** People, code and assisted
  drivers are all agents attempting the same goal; the benchmark should measure each the
  same way.

The goal is the one reference every task already has.

## Detailed design

### Goal score

For each run, `goal = success / max_score`, where `success` is computed exactly as in spec
0.1 (waypoints reached in order, objects delivered, percent of free floor covered). For
each agent on a task, the board MUST report:

- `goal`: the mean goal score over the agent's trials, in [0, 1];
- `goal_ci95`: the 95% percentile-bootstrap interval of that mean (the spec 0.1 resampler,
  unchanged, scaled by `max_score`);
- `goal_worst`: the worst trial's goal score.

Success in task units (`success_mean`, `success_ci95`, `success_worst`, …) remains in the
board for compatibility.

### Chance level

Each task board MUST report `chance`, the goal score a random walk achieves: the median of
per-agent mean goal scores across `baseline` agents when the data contains any, otherwise
`reference.floor / max_score`. `chance_source` is `baseline` or `task`. Chance is
reported beside the goal score and MUST NOT be subtracted from it.

### MotionBench Index

```
MBI = 0.50·goal + 0.20·clamp(path_efficiency, 0, 1) + 0.15·Consistency + 0.15·Safety
Consistency = clamp(1 − std/mean, 0, 1)     (of success, unchanged)
Safety      = 1 / (1 + mean violations)     (unchanged)
```

Every term lies in [0, 1], so MBI lies in [0, 1] and needs no reference group. MBI ranks
agents **within one task**. Implementations MUST NOT average MBI, or any score, across
different tasks; the `overall` section of the spec 0.1 leaderboard is removed.

### Ranks

Shared ranks are unchanged: rows sort by mean success, and a new rank starts only when an
agent's upper interval bound falls below the lower bound of the current group's leader.
Because goal is proportional to success within a task, ranks are identical to spec 0.1.

### People comparison (optional)

A task MAY set `reference.people_comparable: true` when a person can do exactly the same
job with the same means in the same space, for example driving the same robot on the same
field. When the flag is true **and** the data contains `human` agents, the board reports:

- `people`: `median` and `expert` (90th percentile) of per-person mean success, and
  `n_people`;
- per agent, `vs_people = (S_agent − S_chance) / (S_people_median − S_chance)` in task units,
  with `vs_people_ci95`. This is the spec 0.1 HNS, renamed. It is reported only when
  `S_people_median − S_chance` is at least 10% of the task's maximum score; a denominator
  near zero would make the ratio unstable, so otherwise `vs_people` is null.

When the flag is false or absent, `people` and `vs_people` MUST be null, and presentations
SHOULD NOT draw a robot-versus-person comparison. People may still be agents on such a
task; their runs are scored against the goal like any other agent's.

Task flags in this release:

| Task | `people_comparable` | Why |
| --- | --- | --- |
| MB-VEX-NAV@1.0, MB-VEX-PICK@1.0 | true | Drivers and code control the same robot |
| MB-AIM-NAV@0.1, MB-AIM-SORT@0.1 | true | Students drive the same AIM robot the code drives |
| MB-COVER@0.1 | false | A robot vacuum and a person with a stick vacuum use different means |
| MB-FIELD-HALLWAY@0.1 | false | Robot-only field run |

## Compatibility

- **Spec version** becomes 0.2. Reports and boards record `spec_version: "0.2"`. The trace
  and task formats are unchanged apart from the new optional `reference.people_comparable`
  field (default false), so every existing trace remains scorable.
- **Board fields renamed or removed:** `floor` → `chance` (now a goal score in [0, 1]),
  `floor_source` → `chance_source`, `human_reference` → `people` (`n_drivers` → `n_people`;
  `median_path_efficiency` dropped), row `hns` → `vs_people`, `hns_ci95` → `vs_people_ci95`;
  `hns_expert`, `weights.hns_clip` and `overall` removed. New: `goal`, `goal_ci95`,
  `goal_worst`, `people_comparable`.
- **Values that change:** MBI (now in [0, 1]). Ranks and per-run reports do not change.
- RFC-0001 froze the weights "for spec 0.x". This RFC changes the normalization inside the
  index, so it is proposed as a spec change through the public RFC process RFC-0001 sets out.
  Results produced under spec 0.1 stay valid as spec 0.1 results.

## Alternatives

- **Keep HNS everywhere and add caveats.** Rejected: the headline number would still
  compare unlike means on tasks such as coverage.
- **Goal score with chance subtracted** (`(S − chance) / (max − chance)`). Kept in reserve.
  It reads more fairly where luck earns points, but "share of the goal achieved" is easier
  to read, and showing chance beside it keeps the information.
- **Separate boards per agent class.** Rejected: agents attempting the same goal belong on
  the same board, labelled by class.

## Unresolved questions

1. Should tasks where chance is high (coverage) report a chance-adjusted goal score as a
   second column?
2. Should a task declare a target goal level (for example 90% coverage as "done"), and
   report the share of runs that reach it, alongside the mean?
3. What minimum number of trials should an agent need before it receives a shared rank?
