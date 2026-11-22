"""The designs being compared. Each takes the window and returns (estimate, control).

`separate` is the usual design: the smoother is solved alone, and the planner starts from its
estimate as if that estimate were exact. `joint` puts both halves in one graph and solves them
together, so the cost factors can move the estimate.
"""

import gtsam
import numpy as np
from gtsam import noiseModel

from onegraph.graphs import Plan, U, Window, X, future_factors, past_factors
from onegraph.point_mass import PointMass


def separate(model: PointMass, window: Window, plan: Plan) -> tuple[np.ndarray, np.ndarray]:
    now = window.now
    estimate = past_factors(model, window).optimize().at(X(now))
    future = future_factors(model, plan, now)
    future.add(gtsam.JacobianFactor(X(now), np.eye(4), estimate, noiseModel.Constrained.All(4)))
    return estimate, future.optimize().at(U(now))


def joint(model: PointMass, window: Window, plan: Plan) -> tuple[np.ndarray, np.ndarray]:
    now = window.now
    graph = past_factors(model, window)
    graph.push_back(future_factors(model, plan, now))
    solution = graph.optimize()
    return solution.at(X(now)), solution.at(U(now))


ARMS = {"separate": separate, "joint": joint}
