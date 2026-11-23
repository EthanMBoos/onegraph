"""The two halves of the graph, as GTSAM linear factors.

The past half is a fixed-lag smoother: a prior on the oldest state in the window, a dynamics
factor per step with the control that was applied, and a factor per position fix. The future
half is a model predictive controller: a dynamics factor per step with the control as a
variable, and cost factors that pull each state toward the goal and each control toward zero.
Both halves share the state at the current tick, which is the only place they touch.

A cost is written as a Gaussian factor so that it can sit in the same graph as a measurement:
a cost of w * e^2 is a factor on e with sigma 1 / sqrt(w). `cost_scale` multiplies every cost
weight. It does not change what a controller alone would do, but it sets how much the costs weigh
against the measurements once both are in one graph.
"""

from dataclasses import dataclass, field

import gtsam
import numpy as np
from gtsam import noiseModel

from onegraph.point_mass import PointMass


def X(k: int) -> int:
    return gtsam.symbol("x", k)


def U(k: int) -> int:
    return gtsam.symbol("u", k)


@dataclass
class Plan:
    """What the future half is asked to do."""

    goal: np.ndarray = field(default_factory=lambda: np.array([10.0, 5.0, 0.0, 0.0]))
    horizon: int = 20
    # Cost weights per step on position error, velocity error and control.
    position_weight: float = 1.0
    velocity_weight: float = 0.1
    control_weight: float = 0.1
    cost_scale: float = 1.0
    # "hard" plans as if the model were exact, as a standard MPC does. "soft" gives the future
    # dynamics factors the process noise of the past ones, as the STAG post describes.
    dynamics: str = "hard"


@dataclass
class Window:
    """The measurements and controls the past half is built from.

    `prior_mean` and `prior_cov` describe the state at tick `start` given every fix before
    `start`. A Kalman filter step moves them forward when a tick leaves the window, so the
    prior never contains anything the future half produced.
    """

    prior_mean: np.ndarray
    prior_cov: np.ndarray
    lag: int = 20
    start: int = 0
    now: int = 0
    controls: dict[int, np.ndarray] = field(default_factory=dict)
    fixes: dict[int, np.ndarray] = field(default_factory=dict)

    def add_fix(self, z: np.ndarray) -> None:
        self.fixes[self.now] = z

    def advance(self, model: PointMass, u: np.ndarray) -> None:
        """Record the control applied at `now`, move to the next tick, and trim the window."""
        self.controls[self.now] = u
        self.now += 1
        while self.now - self.start > self.lag:
            self._drop_oldest(model)

    def _drop_oldest(self, model: PointMass) -> None:
        m, P = self.prior_mean, self.prior_cov
        z = self.fixes.pop(self.start, None)
        if z is not None:
            H = model.H
            S = H @ P @ H.T + np.eye(2) * model.fix_sigma_m**2
            K = P @ H.T @ np.linalg.inv(S)
            m = m + K @ (z - H @ m)
            P = (np.eye(4) - K @ H) @ P
        u = self.controls.pop(self.start)
        self.prior_mean = model.A @ m + model.B @ u
        self.prior_cov = model.A @ P @ model.A.T + np.diag(model.process_sigmas**2)
        self.start += 1


def past_factors(model: PointMass, window: Window) -> gtsam.GaussianFactorGraph:
    graph = gtsam.GaussianFactorGraph()
    eye = np.eye(4)
    # A JacobianFactor takes a diagonal noise model, so the prior's full covariance is folded
    # into the factor: with R'R equal to the inverse covariance, R x = R mean has unit noise.
    R = np.linalg.cholesky(np.linalg.inv(window.prior_cov)).T
    graph.add(
        gtsam.JacobianFactor(X(window.start), R, R @ window.prior_mean, noiseModel.Unit.Create(4))
    )
    process = noiseModel.Diagonal.Sigmas(model.process_sigmas)
    for k in range(window.start, window.now):
        # x[k+1] - A x[k] = B u[k], with the control that was applied.
        graph.add(
            gtsam.JacobianFactor(
                X(k), -model.A, X(k + 1), eye, model.B @ window.controls[k], process
            )
        )
    fix_noise = noiseModel.Isotropic.Sigma(2, model.fix_sigma_m)
    for k, z in window.fixes.items():
        graph.add(gtsam.JacobianFactor(X(k), model.H, z, fix_noise))
    return graph


def future_factors(model: PointMass, plan: Plan, now: int) -> gtsam.GaussianFactorGraph:
    graph = gtsam.GaussianFactorGraph()
    eye = np.eye(4)
    if plan.dynamics == "hard":
        dynamics = noiseModel.Constrained.All(4)
    elif plan.dynamics == "soft":
        dynamics = noiseModel.Diagonal.Sigmas(model.process_sigmas)
    else:
        raise ValueError(f"Plan.dynamics must be 'hard' or 'soft', not {plan.dynamics!r}")
    state_weights = plan.cost_scale * np.array(
        [plan.position_weight] * 2 + [plan.velocity_weight] * 2
    )
    state_cost = noiseModel.Diagonal.Sigmas(1.0 / np.sqrt(state_weights))
    control_cost = noiseModel.Isotropic.Sigma(
        2, 1.0 / np.sqrt(plan.cost_scale * plan.control_weight)
    )
    for k in range(now, now + plan.horizon):
        # x[k+1] - A x[k] - B u[k] = 0, with the control as a variable.
        graph.add(
            gtsam.JacobianFactor(
                X(k), -model.A, U(k), -model.B, X(k + 1), eye, np.zeros(4), dynamics
            )
        )
        graph.add(gtsam.JacobianFactor(X(k + 1), eye, plan.goal, state_cost))
        graph.add(gtsam.JacobianFactor(U(k), np.eye(2), np.zeros(2), control_cost))
    return graph
