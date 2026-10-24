# Paper: MotionBench methodology (draft for arXiv)

`main.tex` + `refs.bib`; figures in `figs/` (`make_figs.py` rebuilds the verdict chart from
`results/aim-sort-verdict-v0.1.json`).

```bash
cd paper && latexmk -pdf main.tex
```

Status: draft v0.1, open for comment. Before submitting: fill in the author list, read
every number against `results/` and `docs/`, and decide the arXiv category (cs.RO, with
cs.LG as cross-list).
