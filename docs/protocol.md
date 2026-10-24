# Evaluation protocol v0.1

How a MotionBench result is produced, reported and checked. Numbers here are the
defaults every published result follows; a result that departs from them says how.
Draft, open for comment (GitHub Discussions, category "Protocol").

## 1. Design principles

1. **Goal first.** Every task states what "done" means. Every attempt is scored as the
   share of that goal it achieved (RFC-0002), whoever or whatever made the attempt.
2. **Any agent.** People, team-written code, AI-written code, people with robot
   assistance and commercial robots are scored the same way on the same board,
   labelled by who was in control. People and robots are compared directly only when
   they use the same means (`reference.people_comparable`).
3. **Measured where robots work.** A run is a recorded path (a trace), captured from an
   ordinary video, printed tags or the robot's own log. The benchmark does not need the
   robot's code or a special rig.
4. **Re-scorable by anyone.** One open scorer turns a trace into a report, the same way
   every time. Every published number can be recomputed from published data.
5. **Honest about uncertainty.** Ranges on every mean, the worst run beside the
   average, chance level beside every score, and simulated results always labelled.

## 2. Tiers

| Tier | What it is | Shown as |
| --- | --- | --- |
| **S** simulated | produced by a simulator, for testing methods and tools | "Simulated", never mixed with real runs |
| **F** field | a real robot, captured and submitted by its owner, reviewed by a person before publishing | "Real, unverified" |
| **V** verified | a real robot, run under the checks in section 6 | "Verified" |

## 3. Runs

- An agent needs **at least 5 runs** on a task to be ranked; **10 are recommended**.
  Agents with fewer runs are shown, unranked.
- Runs are **consecutive attempts**: every attempt in a session is reported, including
  failures. Picking the best attempts is not allowed.
- Simulated verdicts on games use 20 runs per routine, 12 per driver character and a
  simulated class of 20 drivers with 3 runs each (`motionbench game verdict` defaults).

## 4. Layouts

- **Public layouts** (`dev-*` in each task) are published for practice and for the
  public boards.
- **Unseen layouts** are drawn from the task's layout rules with a seed that is not
  published. They show whether an agent learned the task or only the public layout.
  Simulated verdicts report public and unseen layouts side by side.
- **Verified runs** use a layout drawn by MotionBench just before the session.

## 5. What is reported

For every agent on a task:

- **goal**: mean share of the goal achieved, with a 95% percentile-bootstrap range;
- **worst run**: the lowest goal share among its runs;
- **completion rate**: the share of runs that achieved the whole goal;
- **time to goal**: mean time over runs that achieved the whole goal (on the boards, a time
  is shown only when every run achieved it);
- path efficiency, consistency, safety (contact and rule violations) and the
  MotionBench Index, which ranks agents within one task only;
- **chance**: what a robot moving at random achieves on the task, shown beside the score.

Ranks are shared while ranges overlap (RFC-0001). Scores are never averaged across tasks.

## 6. Verified runs (tier V) checklist

1. The layout is drawn by MotionBench and sent at the start of the session.
2. The video is one uncut take from a fixed camera, showing the whole arena, a code word
   sent at the start of the session, and every run of the session.
3. The trace comes from printed tags or the MotionBench converter run by a reviewer, not
   from the submitter's own processing.
4. Every attempt in the session is reported (section 3).
5. The agent is described: who or what was in control, the robot, and for code the
   version of the routine (shared privately with reviewers on request).
6. A second reviewer re-scores at least one run from the raw video.
7. Videos showing people are not published without their permission; for minors, a parent
   or guardian's. Published results use paths, not footage.

## 7. Repeatability

- **Scorer.** The same trace and task always give the same report (tested on every
  change).
- **Capture.** On rendered test video, the tag-free converter places the robot within
  about 2 cm with a tape-measured floor and about 6 cm from the robot's size alone; with
  printed tags, under 1 cm.
- **Simulation.** Every simulated run is reproducible from its seed.
- Real-world run-to-run spread is reported per agent (the range on its mean); a study of
  repeated sessions with the same robot is part of the first pilots.

## 8. Disputes

Open a GitHub issue (or use the contact page) within 30 days of publication. A reviewer
who did not score the original re-scores the run from the raw trace and video; the
outcome and any correction are logged in `results/CHANGELOG.md`.

## 9. Versions

Every board states the spec version, the scorer version and the date it was produced.
Changes to results are listed in `results/CHANGELOG.md`. Results under an older spec stay
valid as results under that spec.
