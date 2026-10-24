# The MotionBench Manifesto

*Why we measure how well agents complete goals in bounded spaces, and how we promise to do it.*

## The question

A student drives a robot across a 12-foot field. A team's autonomous code does the
same run. An AI writes a program that does it too. A driver tries again with an
auto-align assist. A robot vacuum cleans a living room.

**How much of the job did each one get done, how well, and how reliably?**

Today nobody can answer that with a number anyone else can check. Competitions
score matches, not methods. Product claims are marketing. Research benchmarks
mostly live in simulation. MotionBench exists to make that question answerable:
openly, cheaply, and against the goal the task sets.

## What we measure: goal-directed agents in bounded spaces

MotionBench is for **agents that move inside a bounded space towards a goal**: people,
robots running team-written or AI-written code, people assisted by robots, and
commercial machines. The space can be
a competition field, a room, a warehouse cell, a yard or a lab bench. A task belongs
here when it passes the *bounded-space test*:

1. **Bounded arena.** The space has known edges and known geometry.
2. **Explicit goal.** "Done" is a measurable function of the world state:
   waypoints reached, objects delivered, floor covered.
3. **Repeatable in minutes.** A run is short, the arena can be reset, and five
   trials are cheap.
4. **Contained stakes.** Speeds and energies are low, and the people near the
   robot are the ones running the test.

### What we are not

MotionBench is **not** a benchmark for self-driving cars, road vehicles, delivery
robots on public sidewalks, drones in open airspace, or any system that shares
uncontrolled space with the public. Those are open-world problems with life-safety
stakes, regulators, and their own mature evaluation communities. We do not score
them, we do not borrow their claims, and nothing on our leaderboard is a safety
certification of any kind.

The line is deliberate. Inside a bounded space the world is closed enough that a
result can be reproduced, success can be measured from the end state, and a
fourteen-year-old with a phone can take part. That is what makes the benchmark
honest, and it is what we refuse to dilute.

## What we believe

- **The goal is the reference.** Every run is scored as how much of the task's goal it
  achieved, whoever or whatever ran it. A comparison with people is reported only
  where a person can do exactly the same job with the same means, such as driving
  the same robot. A robot vacuum is not compared with a person holding a stick
  vacuum: the tools differ, so the comparison would measure equipment, not skill.
- **Luck is shown, not hidden.** Every board shows what a robot wandering at random
  achieves on the same task, so a score can be read against chance.
- **Runs, not robots.** We score traces of what happened, never source code or
  hardware. Any machine that leaves a trace can be measured, so no vendor,
  league or language is privileged.
- **Bad runs count.** Matches are lost on the worst run, not won on the best one.
  We publish worst-of-N, variance and confidence intervals next to every mean, and
  we let agents share a rank when the data cannot separate them.
- **Cheap enough for everyone.** A phone on a tripod and four printed tags must be
  enough to participate. If taking part needs a lab, we have failed.
- **Open by default.** Code under Apache-2.0, tasks and data under CC BY 4.0,
  methods and weights published before results. Anyone can rerun every number.
- **Hard to game.** Hidden seeds, repeated trials, verified tiers and frozen
  weights per major version. We would rather publish fewer results than
  unverifiable ones.
- **Synthetic is labelled synthetic.** Simulated data exercises the pipeline. It is
  never presented as evidence about real robots or real people.

## Our promises to the community

1. We will never sell or publish personal data. Runs involving minors are shared as
   traces only, with guardian consent, and faces are never required.
2. We will not imply endorsement by any competition, league or manufacturer that
   has not given it, and we will not reproduce anyone's game rules or field art.
3. We will treat people, team-written code, AI-written code and assisted agents on the
   same footing: same tasks, same seeds, same goals, same scorer, results always
   tagged by agent class.
4. We will change tasks and scoring only through public RFCs, and keep old
   versions scorable forever.
5. We will lead with what helps teams improve, not with "AI beats kids" headlines.

## Where it goes

We start with VEX robotics because thousands of teams already run the same robots
on the same 12-foot field and already split every match into autonomous and
human-driven periods, so drivers and their code can be measured on the same robot.
The same scorer then measures robot vacuums and mowers on floor and lawn covered,
warehouse robots on picks delivered, classroom arms on objects sorted, and simulated
AI agents on all of them.

One question, asked the same way everywhere an agent moves through a bounded space
towards a goal: **how much of the job got done, how well, and how reliably?**

---

*Adopted with RFC-0001; updated with RFC-0002 (goal-referenced scoring). Changes to this
document follow the RFC process.*
