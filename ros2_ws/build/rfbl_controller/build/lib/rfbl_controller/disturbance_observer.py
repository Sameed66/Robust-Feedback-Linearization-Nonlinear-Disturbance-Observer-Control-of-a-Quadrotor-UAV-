# disturbance_observer.py

import numpy as np
from .params_refs import QuadrotorParams
from typing import Optional


class HarmonicDisturbanceObserver:
    """
    Harmonic disturbance observer for the quadrotor altitude/attitude system.

    It estimates:
        [d_z, d_phi, d_theta, d_psi]

    using a known disturbance frequency (params.do_omega) and the model of
    the quadrotor dynamics (eq. (2) in the paper).

    Idea (per axis):
        • Approximate the "measured" disturbance d_meas from the dynamics:
              x_ddot = (known model terms) + d
              => d_meas ≈ x_ddot - (known model terms)
          where x_ddot is obtained via finite-difference on the velocity.

        • Run a 2nd-order harmonic observer:
              z1_dot = z2 + k1 * (d_meas - z1)
              z2_dot = -ω^2 * z1 + k2 * (d_meas - z1)

          Then:
              d_hat = z1
    """

    def __init__(self,
                 params: QuadrotorParams,
                 omega: Optional[float] = None,
                 k1: float = 40.0,
                 k2: float = 400.0):
        """
        Parameters
        ----------
        params : QuadrotorParams
            Quadrotor parameters (mass, inertias, damping, etc.)
        omega : float, optional
            Disturbance frequency; if None, uses params.do_omega.
        k1, k2 : float
            Observer gains (same for all four axes here; can be individualized).
        """
        self.p = params
        self.omega = omega if omega is not None else params.do_omega
        self.k1 = k1
        self.k2 = k2

        # observer states for 4 axes: rows = [z, phi, theta, psi], cols = [z1, z2]
        self.z = np.zeros((4, 2), dtype=float)

        # store previous velocities for numerical acceleration estimate
        # order: [z_dot, phi_dot, theta_dot, psi_dot]
        self.prev_vel = np.zeros(4, dtype=float)
        self.initialized = False

    def reset(self):
        """Reset observer states and previous velocity."""
        self.z[:] = 0.0
        self.prev_vel[:] = 0.0
        self.initialized = False

    def step(self,
             x: np.ndarray,
             u: tuple[float, float, float, float],
             dt: float) -> np.ndarray:
        """
        Advance the disturbance observer one time step.

        Parameters
        ----------
        x : np.ndarray
            State vector [x1..x8] = [z, z_dot, φ, φ_dot, θ, θ_dot, ψ, ψ_dot].
        u : tuple
            Control inputs (Fz, tau_phi, tau_theta, tau_psi).
        dt : float
            Simulation step.

        Returns
        -------
        d_hat : np.ndarray
            Estimated disturbances [d_z_hat, d_phi_hat, d_theta_hat, d_psi_hat].
        """
        p = self.p
        Fz, tau_phi, tau_theta, tau_psi = u

        # unpack states
        z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot = x

        # current velocities for the four axes
        curr_vel = np.array([z_dot, phi_dot, theta_dot, psi_dot], dtype=float)

        # initialize prev_vel on first call to avoid a spike
        if not self.initialized:
            self.prev_vel = curr_vel.copy()
            self.initialized = True

        # numerical accelerations (finite difference)
        safe_dt = max(dt, 1e-6)
        acc = (curr_vel - self.prev_vel) / safe_dt
        self.prev_vel = curr_vel.copy()

        # ------------------------------
        # Model-based disturbance "measurements" d_meas
        # using eq. (2) from the paper.
        # ------------------------------

        # Altitude (z) dynamics:
        #   z_ddot = (cos φ cos θ / m) * Fz - g - (ξ_z/m) * z_dot + d_z
        # => d_z = z_ddot - [ (cos φ cos θ / m) Fz - g - (ξ_z/m) z_dot ]
        phi_cos = np.cos(phi)
        theta_cos = np.cos(theta)
        model_z_ddot = phi_cos * theta_cos / p.m * Fz - p.g - (p.xi_z / p.m) * z_dot
        dz_meas = acc[0] - model_z_ddot

        # Roll (φ) dynamics:
        #   φ_ddot = x4_dot =
        #       (θ_dot ψ_dot (Iy - Iz) / Ix)
        #       - (Ir / Ix) * θ_dot * ω_bar
        #       - (ξ_φ / Ix) * φ_dot
        #       + (1/Ix) τ_φ
        #       + d_φ
        omega_bar = getattr(p, "omega_bar", 0.0)
        model_phi_ddot = (
            theta_dot * psi_dot * (p.Iy - p.Iz) / p.Ix
            - p.Ir * theta_dot * omega_bar / p.Ix
            - p.xi_phi * phi_dot / p.Ix
            + tau_phi / p.Ix
        )
        dphi_meas = acc[1] - model_phi_ddot

        # Pitch (θ) dynamics:
        #   θ_ddot = x6_dot =
        #       (φ_dot ψ_dot (Iz - Ix) / Iy)
        #       - (Ir / Iy) * φ_dot * ω_bar
        #       - (ξ_θ / Iy) * θ_dot
        #       + (1/Iy) τ_θ
        #       + d_θ
        model_theta_ddot = (
            phi_dot * psi_dot * (p.Iz - p.Ix) / p.Iy
            - p.Ir * phi_dot * omega_bar / p.Iy
            - p.xi_theta * theta_dot / p.Iy
            + tau_theta / p.Iy
        )
        dtheta_meas = acc[2] - model_theta_ddot

        # Yaw (ψ) dynamics:
        #   ψ_ddot = x8_dot =
        #       (φ_dot θ_dot (Ix - Iy) / Iz)
        #       - (ξ_ψ / Iz) * ψ_dot
        #       + (1/Iz) τ_ψ
        #       + d_ψ
        model_psi_ddot = (
            phi_dot * theta_dot * (p.Ix - p.Iy) / p.Iz
            - p.xi_psi * psi_dot / p.Iz
            + tau_psi / p.Iz
        )
        dpsi_meas = acc[3] - model_psi_ddot

        d_meas_vec = np.array([dz_meas, dphi_meas, dtheta_meas, dpsi_meas], dtype=float)

        # ------------------------------
        # Harmonic observers (same structure for each axis)
        # ------------------------------
        w2 = self.omega ** 2

        for i in range(4):
            z1, z2 = self.z[i]
            e = d_meas_vec[i] - z1  # innovation

            z1_dot = z2 + self.k1 * e
            z2_dot = -w2 * z1 + self.k2 * e

            z1 += z1_dot * dt
            z2 += z2_dot * dt

            self.z[i, 0] = z1
            self.z[i, 1] = z2

        # disturbance estimate is z1 for each axis
        d_hat = self.z[:, 0].copy()
        return d_hat  # [d_z_hat, d_phi_hat, d_theta_hat, d_psi_hat]
