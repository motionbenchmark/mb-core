# Paper: changes from v0.3 to v0.4

After a third review. No new data were collected.

| Review point | What changed in v0.4 |
|---|---|
| Research question not answered by the evidence | Fair, repeatable and cheap are stated as design objectives; the paper says it develops and demonstrates the framework and names the validation still needed |
| How routines were developed and selected | New paragraph "Routine development and selection": one version of each routine, debugged on the public layout (a development layout), no alternative planners, driver traits not changed after any comparison, unseen layouts generated after the routines were final |
| Simulator and scorer may share assumptions | Seventeen hand-worked scoring cases with derivations (`docs/handworked-cases.md`, `python/tests/test_handworked.py`), including tolerance boundaries, the time limit, missing samples, zone edges, a coverage pass and wall contact; all pass. Two of the first hand derivations were wrong, not the scorer |
| Chance baseline underspecified | Defined: the random walk's action space, sampling and observation, how chance is computed and shown, why it is 0 on delivery tasks, and that a task-level random policy is not implemented |
| Bootstrap wording | "Exact at the resolution of the data" replaced with the number of resamples, the interval method and the limited resolution |
| Vacuum 85% | Called a demonstration of the scoring output, not a validated estimate |
| Figure 2 too small | Split: the simulator image and a full-width chart with larger labels and values |
| Figure 3 | Now shows the tracked path top-down with swept and unseen floor (`make_vacuum_fig.py`); the drawing re-implements the scorer's coverage rule and checks it gives the scorer's 85.0% |
| Repetition and tone | Shorter qualifiers in the introduction and Section 6; "What the study shows, and what it does not" is now "Interpretation"; "three modest statements" reworded |
| Release | Tag `paper-v0.4`; reproduction checked from a fresh clone in a new virtual environment (all checksums match); the install line now names SciPy and Matplotlib, which the analyses need |

Still pending: real-world accuracy experiment; capture-error propagation to scores; more unseen
layouts; a task-level chance policy.

# Paper: changes from v0.2 to v0.3

v0.3 publishes the framework and its methodology. The data illustrate the method and are
not the claim; experiments the second review asked for are listed as pending work rather
than added.

| Review point (second review) | What changed in v0.3 |
|---|---|
| Author | Single author, Abhay Yadav; contributions, AI use and acknowledgements completed |
| Title and framing | "Agent-Independent Robot Evaluation through Goal-Referenced Outcome Scoring: An Open Framework and Preliminary Evaluation in Simulation"; new abstract along the reviewer's lines; "no empirical claim about real robots, code or people" stated in the introduction |
| "Trajectory alone" too broad (E) | Now "externally observed trajectories and task-state measurements"; Section 3 explains which goals depend on the path and which on object positions |
| Novelty unclear (A) | Related work says which ideas are not new; comparison table (Table 3); framework figure (Figure 1) |
| Predictable main result (B) | Presented as a demonstration that the protocol catches a public-layout advantage, with the selection effect stated; new experiments listed under future work |
| Index justification (C) | Index moved to Appendix B, explicitly optional and unused in conclusions; precise definition of the efficiency reference length; when not to use the index; consistency and safety problems named |
| Statistics (D) | Bootstrap replicates (4,000) and method (percentile) stated; equivalence defined as strictly inside the margin, related to two one-sided tests; margin's basis stated; paired simulated runs distinguished from independent replicates; winner-selection bias discussed; per-layout results (Appendix D, `analysis/per_layout.py`, no new runs) |
| Equivalence of the vision routine | The equivalence rule is now strict in the code as well; the vision routine's change is "no difference found" |
| Results did not reproduce from the repository | The published AIM verdict predated simulator changes; regenerated with the current code (`results/CHANGELOG.md`). Planner and dead-reckoning results are unchanged; vision and vision + tags change by 0.05 to 0.15 points |
| Simulator validity, 45 s budget | Stated as assumptions and as a property of the task in Sections 4 and 6; no completion in 45 s noted as partial-progress measurement |
| Table 1, Table 2 | Physical measurements separated from simulated and synthetic; tasks marked by whether they are used in this paper |
| Verified tier | Labelled a proposed procedure not yet exercised |
| Driver parameters | Appendix C lists every trait |
| Vacuum figure | Its own, larger figure |
| Missing-frame bias | Stated as untested |
| Multi-robot matches, roadmap | Matches moved to Appendix E; roadmap shortened to future work led by a small real-world accuracy experiment |
| Speculative sentences | Removed ("would plausibly carry over", "equipment we do not have", grant-like endings) |
| Introduction | Rewritten in the author's first person from the competition and RoboBench motivation |
| AI disclosure | Reviewer's wording; lists the checks actually made instead of "validated every result" |
| Fixed release | The paper names release tag `paper-v0.3` |

Pending (not in v0.3): real-world accuracy experiment; capture-error propagation to scores;
more unseen layouts at several difficulties and ranking stability; failure-case figure;
hand-worked scoring examples; simulator-to-reality comparison.

# Paper: changes from v0.1 to v0.2

How each point of the external review was addressed.

| Review point | What changed in v0.2 |
|---|---|
| Claims broader than the evidence | New title and subtitle (framework, implementation, preliminary experiments); abstract tells one story; a "built / shown / not yet shown" table (Table 1); matches moved to a short "extension" paragraph |
| Simulated human baseline | Called a "parameterized simulated-driver baseline" throughout; the main conclusion is stated as a within-routine comparison that does not depend on the drivers |
| Index justification and bounds | Goal score is the primary outcome; the index is "experimental and secondary"; consistency is clamped with edge cases defined (Appendix B); weight-sensitivity analysis added (Table 5, `analysis/index_sensitivity.py`) |
| Camera validation | Capture accuracy re-measured over three render seeds with median, 90th percentile and missing frames (Table 4, `analysis/capture_accuracy.py`); clearly limited to synthetic video; real validation is pilot 1. Earlier figures (about 2 cm / 6 cm) were corrected to about 3 cm |
| Statistics | Paired runs (same seeds for every agent); resampling unit stated; hierarchical bootstrap over layouts then runs; four outcomes with a 0.5-point equivalence margin; "no difference found" distinguished from "equivalent"; shared ranks described as a display convention |
| Too many directions | Centred on the scoring model, protocol and one experiment (public vs unseen layouts); capture, the vacuum, THÖR-MAGNI and matches are feasibility demonstrations or extensions |
| Version numbers | Paragraph explaining task versions, specification version and scorer version |
| People formula near-zero denominator | Reported only when people beat chance by at least 10% of the maximum (implemented in the scorer, RFC-0002 updated) |
| Verified-run reviewers | Who qualifies, conflicts of interest, third reviewer on disagreement, identities recorded |
| 4.0 vs 2.8 and +1.3 rounding | Exact values to two decimals; the comparison baseline (paired runs of the typical simulated driver, 2.90) is distinguished from the class median used to select it (2.83) |
| Speed-invalid synthetic AIM boards | Marked invalid on the site and in `results/CHANGELOG.md`; excluded from the paper |
| Reproduction | `reproduce.sh`, recorded environment, expected outputs, SHA-256 checksums of deterministic outputs (Appendix A) |
| Authors, contributions, contact | Placeholders for named authors, "Independent researchers", contact email; author contributions and AI-use sections to be completed by the authors |
| Ethics | Statement on data sources, the broadcast footage, and what pilots with minors require (consent, youth protection, IRB) |
