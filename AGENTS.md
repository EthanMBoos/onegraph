# AGENTS.md

onegraph is a prototype: research and student code that changes quickly.
Nothing here is fixed.

## How to work here

- Write the simple, direct version. Add an abstraction, config layer or
  interface only when a present need requires it.
- Experiments are welcome. Replace what does not fit.
- The truth and the graph stay apart. An arm receives the window of fixes and
  controls and nothing else; only the scoring code reads the truth.
- Every arm in a comparison flies the same seeds, and a result is reported as a
  mean and standard error over seeds, never as one run.
- Do not gold-plate: no exhaustive tests, no handling of cases that cannot
  happen. Keep a test if it catches a real mistake.
- State what was run and what was not.

## How to write here

Write docs and comments as a methods section is written: full sentences, each
paragraph opening with its claim, then the reason. Write impersonally, without
"we". Use American spelling.

- State each fact once, in one place. Elsewhere, point to it.
- Give the measurement and what it was measured on, not an adjective.
- Mark what is measured, what is read from a paper, and what is untried.
- A comment says why the code is so. It does not restate the code.
- Wrap Markdown at 80 columns.

## Where things stand

- The point mass is linear, so the graph is a GTSAM `GaussianFactorGraph` solved
  exactly in one step. A nonlinear model needs a `NonlinearFactorGraph` and an
  optimizer.
- Costs are Gaussian factors: a cost of `w * e^2` is a factor on `e` with sigma
  `1 / sqrt(w)`.
- GTSAM is the released 4.3.0 wheel. The STAG code in GTSAM pull request 2936
  is unmerged and is not used.
- The numbers in the README come from `uv run onegraph` with its defaults.
  Rerun it and update the README when a default changes.

## Practical

- `uv run pytest`, then `uv run ruff check . && uv run ruff format .` before
  committing.
- RotorPy is an optional extra: `uv run --extra drone python -m onegraph.drone`.
- Open work is in `docs/TODO.md`.
