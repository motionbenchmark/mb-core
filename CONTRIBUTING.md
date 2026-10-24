# Contributing to MotionBench

Thank you for helping. Every kind of contribution counts: code, tasks, data
collection, reviewing runs, docs and answering questions.

## Ground rules

* Sign off your commits under the [Developer Certificate of Origin](https://developercertificate.org/)
  (`git commit -s`). No CLA.
* Follow the [Code of Conduct](CODE_OF_CONDUCT.md).
* Scoring behaviour lives in `crates/motionbench-core` only. Bindings never
  reimplement scoring; they call the core.
* Any change that alters a score must come with a test and, if it changes
  published results, an RFC.
* Synthetic data must stay labelled synthetic everywhere it appears.

## Development

```bash
cargo fmt --all && cargo clippy --all-targets && cargo test
cd python && maturin develop --release && pytest tests
```

## Submitting runs

* Human runs involving minors need guardian consent before any video is shared.
  Public datasets carry traces only by default.
* Real-hardware runs for the main board need video; verified runs need a hidden
  seed issued by the leaderboard server.

## Proposing a task

Open an RFC using `rfcs/0000-template.md`. A task proposal must pass the
bounded-space test in [MANIFESTO.md](MANIFESTO.md), define public seeds, say how
the goal is measured and whether a person can do the same job with the same means
(`reference.people_comparable`), and include at least one reference trace.
