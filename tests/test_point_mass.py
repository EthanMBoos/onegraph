import numpy as np

from onegraph.arms import joint, separate
from onegraph.experiment import START_MEAN, START_SIGMAS, run
from onegraph.graphs import Plan, Window
from onegraph.point_mass import PointMass


def fly(arm, lag, plan, ticks=40, seed=3):
    """The estimates an arm produces on a fixed sequence of fixes and controls."""
    model = PointMass()
    rng = np.random.default_rng(seed)
    x = START_MEAN + rng.normal(0.0, START_SIGMAS)
    window = Window(prior_mean=START_MEAN.copy(), prior_cov=np.diag(START_SIGMAS**2), lag=lag)
    estimates = []
    for _ in range(ticks):
        window.add_fix(model.fix(x, rng))
        estimate, _ = arm(model, window, plan)
        estimates.append(estimate)
        u = rng.normal(0.0, 1.0, size=2)  # the same controls whatever the arm plans
        x = model.step_truth(x, u, rng)
        window.advance(model, u)
    return np.array(estimates)


def test_a_short_window_gives_the_estimate_of_a_long_one():
    # The model is linear and Gaussian, so folding old ticks into the prior loses nothing.
    short = fly(separate, lag=3, plan=Plan())
    full = fly(separate, lag=1000, plan=Plan())
    np.testing.assert_allclose(short, full, atol=1e-8)


def test_joint_approaches_separate_as_the_costs_vanish():
    # With weightless costs the future half tells the estimate nothing.
    plan = Plan(cost_scale=1e-6)
    np.testing.assert_allclose(fly(joint, 20, plan), fly(separate, 20, plan), atol=1e-3)


def test_separate_ignores_the_cost_scale():
    model = PointMass()
    small, _, _ = run("separate", 0, model, Plan(cost_scale=0.01), ticks=60)
    large, _, _ = run("separate", 0, model, Plan(cost_scale=10.0), ticks=60)
    assert abs(small.flown_cost - large.flown_cost) < 1e-6 * small.flown_cost


def test_joint_estimate_leans_toward_the_goal():
    # The finding the README reports. If this stops holding, the README is out of date.
    model = PointMass()
    lean = {
        arm: np.mean(
            [run(arm, seed, model, Plan(), ticks=60)[0].toward_goal_m for seed in range(5)]
        )
        for arm in ("separate", "joint")
    }
    assert abs(lean["separate"]) < 0.2
    assert lean["joint"] > 0.5
