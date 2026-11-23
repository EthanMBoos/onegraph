# TODO

The goal is a drone that is robust to conditions nobody tuned it for, using one
factor graph for estimation and control in place of a separately tuned
estimator and controller. The repo so far has the point mass, two arms, and one
measured result: the joint estimate leans toward the goal (see the README).
Nothing below has been started, and the items are in the order they should be
taken.

**Explain the lean.** Eliminating the future half of the graph should leave a
quadratic factor on the current state. Derive it, and check that the predicted
lean matches the measured one at each cost scale. Then find out why the flight
diverges at a cost scale of 10; this has been observed and not explained.

**Add a third arm that cannot lean.** The estimate should inform the plan
without the plan informing the estimate. One way is to solve the past first
and hand the future its estimate and covariance, not the estimate alone. For
the point mass this arm should match `separate` exactly, since a plan with a
quadratic cost ignores the covariance. It is the arm the later steps need.

**Fly both arms through conditions neither was tuned for.** Tune each arm once
on the nominal case and then change the truth without retuning: a fix that
degrades or drops out, a wind the model does not know (`--wind` already does
this), and a mass or drag that differs from the model. The score is how much
each arm loses. One standard case to include is the light-dark problem (Platt et
al., 2010), where fixes are accurate in one region and poor elsewhere, so the
best path detours to localize before approaching the goal; a controller that
accounts for that is the first piece that does not fit in a linear graph.

**Estimate the wind in the past and use it in the future.** `--wind` already
puts a steady wind on the truth. Add the wind as a variable shared by the
dynamics factors of both halves. A separate estimator that passes its wind
estimate to the controller is the baseline to beat, and it may simply tie.

**Move to a drone.** `src/onegraph/drone.py` shows RotorPy stepping a Crazyflie
under an acceleration command in wind, with an IMU reading and a range to a
beacon. Only that has been run. The state becomes a pose and velocity, the
past half needs IMU and range factors, and the graph becomes nonlinear.

**Measure the cost.** Report solve time per tick against the horizon and the
lag, and try an incremental solver (iSAM2) once the graph is nonlinear.

**Compare with sampling.** GTSAM pull request 2936 adds MPPI on a factor graph.
Once it merges, run it as the controller in each arm.

Before showing results to Dellaert's group, read the 2014 and STEAP papers
linked in the README and state how this differs from each.
