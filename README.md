# onegraph

onegraph puts a robot's state estimator and its controller in one factor graph
and compares that with the usual design, where the two are separate. It is a
riff on a
[post by Frank Dellaert](https://gtsam.org/2026/04/21/factor-graphs-and-world-models.html),
which proposes the joint graph and calls it STAG. The past few seconds of the
graph are a smoother and the next few are a model predictive controller, and
the two halves share the state at the current tick. The graph is built with
[GTSAM](https://gtsam.org).

So far the repo flies a planar point mass to a goal. The drone is the work
ahead; see [docs/TODO.md](docs/TODO.md).
[docs/BACKGROUND.md](docs/BACKGROUND.md) explains the concepts.

## Try it

With [uv](https://docs.astral.sh/uv/) installed:

```sh
git clone https://github.com/EthanMBoos/onegraph.git
cd onegraph
uv sync
uv run onegraph                       # about 10 seconds
uv run onegraph --plot out/paths.png  # also draws seed 0
```

This flies two arms on the same 20 seeds and scores each against the truth,
which the arms never see. `separate` solves the smoother alone and plans from
its estimate as if it were exact. `joint` solves both halves as one graph.

## What it shows so far

The truth here is the model the graph plans with: linear dynamics, Gaussian
noise, and a quadratic cost. In that case the separate design is known to be
optimal, so the joint graph can at best tie. It does not tie. Its estimate
leans toward the goal, and the lean grows with the cost weights.

| cost scale | arm | estimate RMSE (m) | lean toward goal (m) | flown cost |
|---|---|---|---|---|
| any | separate | 0.323 ± 0.009 | 0.006 ± 0.035 | 1151 ± 40 |
| 0.01 | joint | 0.324 ± 0.009 | 0.031 ± 0.035 | 1152 ± 40 |
| 0.1 | joint | 0.369 ± 0.010 | 0.243 ± 0.032 | 1168 ± 41 |
| 1 | joint | 1.05 ± 0.02 | 1.44 ± 0.04 | 1712 ± 70 |

Each entry is a mean and its standard error over 20 seeds of a 15 s flight with
a position fix of 0.5 m standard deviation every 0.1 s. The lean is the
estimate's position error along the direction from the estimate to the goal.
Both arms take 0.55 ms per tick on an Apple M-series CPU with GTSAM 4.3.0. At
a cost scale of 3 the joint estimate is off by 2.6 m RMS, and at 10 the flight
diverges.

**Why the estimate leans.** A cost is a factor like any other. Seen from the
current state, the future half sums to a quadratic cost-to-go that is lowest at
the goal, so it acts on the estimate as a position fix at the goal would. The
solver cannot tell a measurement from a wish.

**Why the cost scale matters.** A controller alone does not care about the scale
of its cost, since doubling every weight leaves the best plan unchanged. The
separate arm's row is the same at every scale for that reason. In the joint
graph the scale sets how much the costs weigh against the measurements, so a
number that used to be arbitrary now decides how wrong the estimate is.

Two further runs, each at a cost scale of 0.1, were made once. Giving the
future dynamics factors the process noise of the past ones (`--dynamics soft`),
as the post describes, changes no entry by more than 0.001. A steady wind of
1 m/s² on the truth that the model does not know (`--wind 1 0`) leaves both arms
about 1.8 m short of the goal, because neither estimates the wind.

## Where things are

- `src/onegraph/point_mass.py` is the model and the truth.
- `src/onegraph/graphs.py` builds the two halves of the graph.
- `src/onegraph/arms.py` is the two designs, a few lines each.
- `src/onegraph/experiment.py` is the closed loop, the scores, and the command.
- `src/onegraph/drone.py` checks that RotorPy can serve as the drone's truth.

## Prior work

Ta, Kobilarov, and Dellaert put estimation and model predictive control in one
factor graph on a simulated quadrotor in
[2014](https://dellaert.github.io/files/Ta14icuas.pdf).
[STEAP](https://arxiv.org/abs/1807.10425) (2019) solved estimation and planning
together over a fixed interval on a mobile manipulator.
[DynoJEPP](https://arxiv.org/abs/2605.12897) (2026) reports that its robot
collides in most runs unless some factors are made directed, so that planning
cannot move the estimate; that reads like the lean measured here, but the two
have not been compared. None of these papers has been reproduced in this repo.
