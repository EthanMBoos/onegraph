"""Fly each arm on the same seeds and score it against the truth.

A run is a closed loop. At each tick the truth produces a position fix, the arm turns the
window into an estimate and a control, and the truth takes a step with that control. The arm
never sees the truth. Two arms given the same seed see the same initial state; their fixes and
process noise are drawn from the same generator in the same order, so the draws match tick for
tick even though the flown paths differ.
"""

import argparse
import time
from dataclasses import dataclass, replace

import numpy as np

from onegraph.arms import ARMS
from onegraph.graphs import Plan, Window
from onegraph.point_mass import PointMass

# The estimator's prior on the starting state, and what the truth is drawn from.
START_MEAN = np.zeros(4)
START_SIGMAS = np.array([1.0, 1.0, 0.5, 0.5])


@dataclass
class Score:
    """One run, scored against the truth."""

    # RMS position error of the estimate, in meters.
    estimate_rmse_m: float
    # Mean of the estimate's position error along the direction from the estimate to the goal,
    # in meters. Zero for an unbiased estimator; positive when the estimate sits closer to the
    # goal than the vehicle is.
    toward_goal_m: float
    # The cost the plan minimizes, summed along the path the truth flew, at cost_scale 1.
    flown_cost: float
    # RMS distance of the truth from the goal over the last second, in meters.
    final_miss_m: float
    solve_ms: float


def run(arm_name: str, seed: int, model: PointMass, plan: Plan, ticks: int = 150, lag: int = 20):
    """Returns the score and the truth and estimate at every tick."""
    arm = ARMS[arm_name]
    rng = np.random.default_rng(seed)
    x = START_MEAN + rng.normal(0.0, START_SIGMAS)
    window = Window(prior_mean=START_MEAN.copy(), prior_cov=np.diag(START_SIGMAS**2), lag=lag)
    weights = np.array([plan.position_weight] * 2 + [plan.velocity_weight] * 2)
    truth, estimates = [], []
    cost = 0.0
    solve_s = 0.0
    for _ in range(ticks):
        window.add_fix(model.fix(x, rng))
        started = time.perf_counter()
        estimate, u = arm(model, window, plan)
        solve_s += time.perf_counter() - started
        truth.append(x)
        estimates.append(estimate)
        x = model.step_truth(x, u, rng)
        cost += weights @ (x - plan.goal) ** 2 + plan.control_weight * u @ u
        window.advance(model, u)
    truth, estimates = np.array(truth), np.array(estimates)
    error = estimates[:, :2] - truth[:, :2]
    # The direction is taken from the estimate, not the truth: a correct estimator's error
    # averages to zero given its own estimate, but not given where the truth happens to be.
    to_goal = plan.goal[:2] - estimates[:, :2]
    distance = np.linalg.norm(to_goal, axis=1)
    far = distance > 1.0  # the direction to the goal means nothing once the estimate is on it
    toward = np.sum(error[far] * to_goal[far], axis=1) / distance[far]
    miss = np.linalg.norm(plan.goal[:2] - truth[:, :2], axis=1)
    last = int(round(1.0 / model.dt_s))
    score = Score(
        estimate_rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
        toward_goal_m=float(np.mean(toward)) if far.any() else 0.0,
        flown_cost=float(cost),
        final_miss_m=float(np.sqrt(np.mean(miss[-last:] ** 2))),
        solve_ms=1000.0 * solve_s / ticks,
    )
    return score, truth, estimates


def compare(model: PointMass, plan: Plan, seeds: int) -> dict[str, dict[str, tuple[float, float]]]:
    """Mean and standard error of every score, per arm, over `seeds` runs."""
    table = {}
    for arm_name in ARMS:
        scores = [run(arm_name, seed, model, plan)[0] for seed in range(seeds)]
        table[arm_name] = {}
        for name in Score.__dataclass_fields__:
            values = np.array([getattr(score, name) for score in scores])
            table[arm_name][name] = (values.mean(), values.std(ddof=1) / np.sqrt(seeds))
    return table


def print_table(title: str, table) -> None:
    columns = {
        "estimate_rmse_m": "est rmse m",
        "toward_goal_m": "toward goal m",
        "flown_cost": "flown cost",
        "final_miss_m": "final miss m",
    }
    print(f"\n{title}")
    print(
        f"{'arm':<10}" + "".join(f"{label:>20}" for label in columns.values()) + f"{'solve ms':>10}"
    )
    for arm_name, row in table.items():
        cells = "".join(f"{f'{row[name][0]:.3g} ± {row[name][1]:.2g}':>20}" for name in columns)
        print(f"{arm_name:<10}{cells}{row['solve_ms'][0]:>10.2f}")


def plot(model: PointMass, plan: Plan, path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(ARMS), figsize=(5 * len(ARMS), 4.5), sharex=True, sharey=True)
    for ax, arm_name in zip(axes, ARMS, strict=True):
        _, truth, estimates = run(arm_name, 0, model, plan)
        ax.plot(truth[:, 0], truth[:, 1], color="#1f2933", label="truth")
        ax.plot(estimates[:, 0], estimates[:, 1], color="#2f6fdb", label="estimate")
        ax.plot(*plan.goal[:2], marker="x", color="#1a9c8a", markersize=10, label="goal")
        ax.set_title(f"{arm_name}, cost scale {plan.cost_scale:g}, seed 0")
        ax.set_xlabel("x (m)")
        ax.set_aspect("equal")
        ax.legend()
    axes[0].set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nwrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--cost-scales", type=float, nargs="+", default=[0.01, 0.1, 1.0])
    parser.add_argument("--dynamics", choices=["hard", "soft"], default="hard")
    parser.add_argument(
        "--wind",
        type=float,
        nargs=2,
        default=[0.0, 0.0],
        metavar=("X", "Y"),
        help="constant acceleration on the truth that the model does not know, m/s^2",
    )
    parser.add_argument("--plot", metavar="PNG", help="save the flown paths of seed 0")
    args = parser.parse_args()

    model = PointMass(wind_mps2=np.array(args.wind))
    for cost_scale in args.cost_scales:
        plan = Plan(cost_scale=cost_scale, dynamics=args.dynamics)
        print_table(
            f"cost scale {cost_scale:g}, {args.dynamics} plan dynamics, {args.seeds} seeds",
            compare(model, plan, args.seeds),
        )
    if args.plot:
        plot(model, replace(plan, cost_scale=args.cost_scales[-1]), args.plot)
