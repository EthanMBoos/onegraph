"""A check that the drone step has what it needs: RotorPy as the truth, read through sensors.

This flies nothing useful. It steps RotorPy's Crazyflie under a constant acceleration command
in a steady wind and prints the truth beside what the graph would be given: an IMU reading and
a range to a beacon. Neither arm is connected to it yet; docs/TODO.md says what is left.

Run it with `uv run --extra drone python -m onegraph.drone`.
"""

import numpy as np
from rotorpy.sensors.imu import Imu
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor

BEACON_M = np.array([5.0, 0.0, 1.0])
RANGE_SIGMA_M = 0.1


def beacon_range(position_m: np.ndarray, rng: np.random.Generator) -> float:
    return float(np.linalg.norm(position_m - BEACON_M) + rng.normal(0.0, RANGE_SIGMA_M))


def main() -> None:
    rng = np.random.default_rng(0)
    dt_s = 0.01
    # `cmd_acc` is a thrust vector divided by mass, in the world frame with Z up, so a planner's
    # acceleration needs gravity added. RotorPy's own attitude loop turns it into rotor speeds.
    vehicle = Multirotor(quad_params, control_abstraction="cmd_acc")
    imu = Imu()
    state = dict(vehicle.initial_state)
    state["wind"] = np.array([2.0, 0.0, 0.0])
    command = {"cmd_acc": np.array([0.5, 0.0, 0.0]) + np.array([0.0, 0.0, vehicle.g])}
    print(f"{'t s':>5}{'truth position m':>24}{'imu accel m/s^2':>24}{'range m':>9}")
    for tick in range(1, 201):
        state = vehicle.step(state, command, dt_s)
        if tick % 50 == 0:
            reading = imu.measurement(state, vehicle.statedot(state, command, dt_s))
            position = "  ".join(f"{value:6.2f}" for value in state["x"])
            accel = "  ".join(f"{value:6.2f}" for value in reading["accel"])
            print(
                f"{tick * dt_s:>5.1f}{position:>24}{accel:>24}{beacon_range(state['x'], rng):>9.2f}"
            )


if __name__ == "__main__":
    main()
