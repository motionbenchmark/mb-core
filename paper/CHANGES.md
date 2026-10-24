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
