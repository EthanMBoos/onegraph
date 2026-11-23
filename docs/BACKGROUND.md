# Background

onegraph follows Frank Dellaert's post
[Factor Graphs and "World Models"](https://gtsam.org/2026/04/21/factor-graphs-and-world-models.html).
The post assumes the reader already knows factor graphs, smoothing, and model
predictive control. The sections below build each of those up in the order the
post uses them, and name the code in this repo that implements each one. The
measured results are in the [README](../README.md).

## State, measurements, and models

A robot is described at one instant by its state, which for the point mass in
this repo is a position and a velocity in the plane, `[px, py, vx, vy]`. The
robot cannot read its state directly. It reads measurements, which are noisy
and usually partial (e.g., a position fix with a standard deviation of 0.5 m
that says nothing about velocity), so the state has to be estimated from them.

Estimation relies on two models. The dynamics model predicts the next state
from the current state `x` and the control `u` that was applied, and is written
`f(x, u)`. The measurement model predicts what a sensor reads when the state is
`x`, and is written `h(x)`. Neither model is exact, so each comes with a noise
level that says how far the truth is expected to stray from the prediction.
Both models for the point mass are in `src/onegraph/point_mass.py`, where the
dynamics are `A x + B u` and a fix is `H x`.

## Factor graphs

A factor is a function that scores how well a small set of variables agrees
with one piece of knowledge. A measurement factor is attached to one state and
scores high when `h(x)` is close to what the sensor read. A dynamics factor is
attached to two consecutive states and scores high when the second is close to
`f` applied to the first. A factor graph is the collection of all the unknown
variables and all the factors on them, drawn with a circle for each variable
and a dot for each factor, joined to the variables it scores.

The score of one guess for all the states is the product of the factors, and
that product is proportional to the probability of the guess given the
measurements `z`:

$$p(x \mid z) \propto \prod_i \phi_i(x_i)$$

Here `x_i` is the small subset of variables that factor `i` is attached to.
Each factor involves one or two variables out of possibly hundreds, and
solvers such as GTSAM are fast because they exploit that sparsity.

## Energy

The post's first claim is that a factor graph is an energy-based model. The
negative log of the probability above turns the product into a sum, and the
post refers to that sum as the energy:

$$E(x) = -\log p(x \mid z) = \sum_i E_i(x_i) + \text{constant}$$

A probable guess has a low energy, so finding the most probable states is the
same as minimizing the energy. When the noise is Gaussian, each term is a
squared error divided by a variance (e.g., `(z - h(x))² / σ²` for a sensor with
standard deviation `σ`), so the minimization is a least-squares problem, and a
factor with a small `σ` pulls harder on the solution than one with a large `σ`.

An energy-based model, in the sense of LeCun's work that the post cites,
likewise defines an energy over its variables and finds an answer by minimizing
it. The difference is in where the energy comes from: in a factor graph each
term is written by hand from physics and sensor models, whereas in a learned
world model the terms are neural networks.

## The past half: a fixed-lag smoother

The first graph the post builds covers the last few seconds. Its variables are
the states over that window. A dynamics factor connects each pair of
consecutive states, and a measurement factor is attached to each state that
received a fix. The controls in the window are known, because the robot has
already applied them, so they enter the dynamics factors as constants.

Minimizing this graph estimates the whole recent path and not only the current
state. This is the difference between a smoother and a filter: a filter (e.g.,
a Kalman filter) updates the current state and never revises an earlier one,
whereas a smoother revises the earlier states each time a measurement arrives,
which also improves the current estimate. The window is kept at a fixed length
so that the cost of each solve stays bounded, and when a state leaves the
window, what was known about it is kept as a prior on the new oldest state.
`past_factors` in `src/onegraph/graphs.py` builds this half, and `Window`
carries the prior forward with a Kalman filter step. The post draws a window of
five seconds; the default here is 20 ticks of 0.1 s.

## The future half: a model predictive controller

The second graph covers the next few seconds, and its variables are the states
over that horizon. It uses the same dynamics factors as the past half, which
the post emphasizes: the robot predicts the future with the model it uses to
explain the past. The controls are different, because they have not been
chosen yet, and they therefore become variables that are solved for together
with the states.

The future has no measurements. In their place the graph has cost factors,
which express what is wanted (e.g., a state near the goal, a state away from an
obstacle, a small control). A cost of `w e²` is written as a Gaussian factor on
`e` with standard deviation `1 / sqrt(w)`, so that it has the same form as a
measurement factor and can sit in the same graph. Minimizing the graph gives a
planned path and the controls that follow it. The robot applies the first
control, advances one tick, and solves again, which is the standard model
predictive control loop. `future_factors` builds this half.

The post refers to this half as planning. This repo refers to it as control,
because it outputs a control at every tick and nothing in it chooses a route.
The word plan is kept for the path that the controller solves for.

## STAG: the two halves anchored at the present

The two halves have one variable in common, the state at the current tick.
Drawn together they form a single graph, with measurement factors on the past
side, cost factors on the future side, and the same dynamics throughout. The
post refers to this as STAG, for Sense-Think-Act with Graphs: the robot
collects measurements, optimizes over the factors, and applies the first
control. The earlier STEAP paper solved estimation and planning together over a
fixed interval from a start to a goal, and the post's step beyond it is to make
the graph a rolling window around the present.

The post is not consistent about whether this is one graph or two. Its figure
and its STAG section describe "a single rolling factor graph", while its
setup and its summary describe "two tightly coupled graphs" and say that the
robot optimizes "factor graphs over both the recent past and the near future".
The drawing is the same either way, since the halves share one variable. What
differs is how it is solved, and the post never says. Solving everything at
once lets every factor act on every variable, so the costs can move the
estimate. Solving the past first and then the future from its result lets
information pass in one direction only.

The two arms in `src/onegraph/arms.py` are these two readings. `joint` places
both halves in one `GaussianFactorGraph` and solves it once. `separate` solves
the past half alone and then fixes the current state of the future half to
that estimate, which is also the usual design of an estimator followed by a
controller.

## What the post leaves open

The post proposes a structure and reports no experiment. It closes with
questions for any robot and does not answer them: what the state is and whether
it can be learned, whether the dynamics model is analytic, learned, or a
hybrid, how each sensor connects to the state, and what the objectives for the
future are. It also doubts parts of its own structure, asking whether the graph
should be probabilistic at all, whether a factor could be a neural network that
outputs an energy, and whether a sampling method (e.g., MPPI) could replace the
least-squares solve.

One question is missing from that list, and it is the one the two readings
above turn on: which way information should flow through the current state.
Information that flows forward, from the past half to the future half, is a
fact about the world (e.g., where the robot is and how certain that estimate
is), and a plan should use it. Information that flows backward is a goal. A
goal says where the robot should be and is not evidence of where it is.

```
      measurements                                 goals
      z   z   z   z                            g   g   g   g
      │   │   │   │                            │   │   │   │
      x ─ x ─ x ─ x ────────── x_t ─────────── x ─ x ─ x ─ x
        past half              now              future half


                  ┌──────────┐                      ┌──────────┐
  usual design    │ estimate │ ──── one value ────> │   plan   │
                  └──────────┘                      └──────────┘

                  ┌──────────┐ ──── everything ───> ┌──────────┐
  joint solve     │ estimate │                      │   plan   │
                  └──────────┘ <─── everything ──── └──────────┘

                  ┌──────────┐      value and       ┌──────────┐
  in between      │ estimate │ ─── uncertainty ───> │   plan   │
                  └──────────┘                      └──────────┘
```

The usual design sends one value forward and nothing backward. A joint solve
sends everything both ways, so the plan sees the uncertainty of the estimate,
and the goal also moves the estimate. The README measures that second effect on
the point mass and refers to it as the lean. Designs in between are possible
(e.g., passing the uncertainty forward and nothing backward).

Which of these is right is not known. The post does not say, and the prior work
listed in the README does not settle it.
