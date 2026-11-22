"""A planar point mass: the model the graph plans with, and the truth it is scored against.

The state is [px, py, vx, vy] in meters and meters per second, and the control is an
acceleration [ax, ay]. The truth takes the same step as the model plus process noise and an
optional constant wind acceleration that the model does not know about.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PointMass:
    dt_s: float = 0.1
    # Standard deviation of the noise one step adds to position and to velocity.
    process_sigma_m: float = 0.02
    process_sigma_mps: float = 0.1
    # Standard deviation of a position fix on each axis.
    fix_sigma_m: float = 0.5
    # A constant acceleration on the truth only. Zero makes the truth equal the model.
    wind_mps2: np.ndarray = field(default_factory=lambda: np.zeros(2))

    @property
    def A(self) -> np.ndarray:
        A = np.eye(4)
        A[0, 2] = A[1, 3] = self.dt_s
        return A

    @property
    def B(self) -> np.ndarray:
        B = np.zeros((4, 2))
        B[0, 0] = B[1, 1] = 0.5 * self.dt_s**2
        B[2, 0] = B[3, 1] = self.dt_s
        return B

    @property
    def H(self) -> np.ndarray:
        """A fix measures position."""
        return np.hstack([np.eye(2), np.zeros((2, 2))])

    @property
    def process_sigmas(self) -> np.ndarray:
        p, v = self.process_sigma_m, self.process_sigma_mps
        return np.array([p, p, v, v])

    def step_truth(self, x: np.ndarray, u: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        noise = rng.normal(0.0, self.process_sigmas)
        return self.A @ x + self.B @ (u + self.wind_mps2) + noise

    def fix(self, x: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        return self.H @ x + rng.normal(0.0, self.fix_sigma_m, size=2)
