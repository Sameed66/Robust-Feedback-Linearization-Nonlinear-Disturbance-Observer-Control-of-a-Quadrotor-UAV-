# xy_disturbance_observer.py

import numpy as np
from .params_refs import QuadrotorParams


class XYHarmonicDisturbanceObserver:
    """
    Harmonic disturbance observer for X–Y translational dynamics.

    Estimates:
        [d_x, d_y]

    using the model (consistent with eq. (21) in the paper):

        x_ddot = -ξ_x/m * x_dot + (Fz/m) * Fx + d_x
        y_ddot = -ξ_y/m * y_dot + (Fz/m) * Fy + d_y

    So:

        d_x_meas = x_ddot - [ -ξ_x/m * x_dot + (Fz/m) * Fx ]
        d_y_meas = y_ddot - [ -ξ_y/m * y_dot + (Fz/m) * Fy ]

    Each axis uses a 2nd-order harmonic observer (same structure as for z,φ,θ,ψ):

        z1_dot = z2 + k1 * (d_meas - z1)
        z2_dot = -ω^2 * z1 + k2 * (d_meas - z1)

        d_hat = z1
    """

    def __init__(self,
                 params: QuadrotorParams,
                 omega: float | None = None,
                 k1: float = 40.0,
                 k2: float = 400.0):
        """
        Parameters
        ----------
        params : QuadrotorParams
            Quadrotor parameters (m, ξ_x, ξ_y, do_omega, etc.).
        omega : float, optional
            Disturbance frequency; if None, uses params.do_omega.
        k1, k2 : float
            Observer gains (same for X and Y here).
        """
        self.p = params
        self.omega = omega if omega is not None else params.do_omega
        self.k1 = k1
        self.k2 = k2

        # observer states for x, y:
        # row 0 -> x axis [z1_x, z2_x]
        # row 1 -> y axis [z1_y, z2_y]
        self.z = np.zeros((2, 2), dtype=float)

        # previous velocities for finite-difference acceleration:
        # [x_dot, y_dot]
        self.prev_vel = np.zeros(2, dtype=float)
        self.initialized = False

    def reset(self):
        """Reset observer states and previous velocities."""
        self.z[:] = 0.0
        self.prev_vel[:] = 0.0
        self.initialized = False

    def step(self,
             x9_12: np.ndarray,
             Fz: float,
             Fx: float,
             Fy: float,
             dt: float) -> np.ndarray:
        """
        Advance X–Y disturbance observer one time-step.

        Parameters
        ----------
        x9_12 : np.ndarray
            [x, x_dot, y, y_dot] = [x9, x10, x11, x12].
        Fz : float
            Vertical thrust from inner loop (used in translational model).
        Fx, Fy : float
            Virtual control inputs along x, y (output of XYPositionRFBLSTC).
        dt : float
            Simulation time step.

        Returns
        -------
        d_hat_xy : np.ndarray
            Estimated disturbances [d_x_hat, d_y_hat].
        """
        p = self.p

        # unpack x–y states
        x_pos, x_dot, y_pos, y_dot = x9_12

        # current velocities
        curr_vel = np.array([x_dot, y_dot], dtype=float)

        # initialize previous velocity on first call
        if not self.initialized:
            self.prev_vel = curr_vel.copy()
            self.initialized = True

        # finite-difference accelerations
        safe_dt = max(dt, 1e-6)
        acc = (curr_vel - self.prev_vel) / safe_dt
        self.prev_vel = curr_vel.copy()

        # numerical accelerations
        x_ddot_meas = acc[0]
        y_ddot_meas = acc[1]

        # model-based parts from dynamics:
        # x_ddot_model = -ξ_x/m * x_dot + (Fz/m) * Fx
        # y_ddot_model = -ξ_y/m * y_dot + (Fz/m) * Fy
        x_ddot_model = -p.xi_x / p.m * x_dot + (Fz / p.m) * Fx
        y_ddot_model = -p.xi_y / p.m * y_dot + (Fz / p.m) * Fy

        # disturbance measurements d_meas = x_ddot_meas - model
        dx_meas = x_ddot_meas - x_ddot_model
        dy_meas = y_ddot_meas - y_ddot_model

        d_meas_vec = np.array([dx_meas, dy_meas], dtype=float)

        # harmonic observer update for x and y
        w2 = self.omega ** 2

        for i in range(2):
            z1, z2 = self.z[i]
            e = d_meas_vec[i] - z1  # innovation

            z1_dot = z2 + self.k1 * e
            z2_dot = -w2 * z1 + self.k2 * e

            z1 += z1_dot * dt
            z2 += z2_dot * dt

            self.z[i, 0] = z1
            self.z[i, 1] = z2

        # disturbance estimates are z1_x, z1_y
        d_hat_xy = self.z[:, 0].copy()  # [d_x_hat, d_y_hat]
        return d_hat_xy
