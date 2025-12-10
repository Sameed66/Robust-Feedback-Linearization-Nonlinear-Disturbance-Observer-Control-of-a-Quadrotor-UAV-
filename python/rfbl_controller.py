# rfbl_controller.py

import numpy as np
from params_refs import QuadrotorParams


def _sign(x: float) -> float:
    """Sign function with sign(0) = 0 (scalar)."""
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


class RFBLAltitudeAttitudeController:
    """
    Robust Feedback Linearization + Supertwisting controller
    for altitude z and attitude (φ, θ, ψ).

    Implements eqs. (11)–(14) of the paper.

    State (first 8 states of the paper):
        x1 = z
        x2 = z_dot
        x3 = φ (roll)
        x4 = φ_dot
        x5 = θ (pitch)
        x6 = θ_dot
        x7 = ψ (yaw)
        x8 = ψ_dot

    step() arguments:
        x       : np.array shape (8,)
                  current [x1..x8]
        xd      : np.array shape (8,)
                  desired states [x1d..x8d]
        xd_dot  : np.array shape (8,)
                  desired state derivatives [x1d_dot..x8d_dot]
                  Only indices 1,3,5,7 are used: ˙x2d, ˙x4d, ˙x6d, ˙x8d
                  (i.e. desired accelerations of z, φ, θ, ψ).
        d_hat   : np.array shape (4,)
                  disturbance estimates [d_hat_z, d_hat_phi, d_hat_theta, d_hat_psi]
        dt      : float
                  integration step for supertwisting integrals.

    Returns:
        Fz, tau_phi, tau_theta, tau_psi
    """

    def __init__(self, params: QuadrotorParams):
        self.p = params

        # supertwisting integral terms ∫ sign(s) dt
        self.int_sign_sz = 0.0
        self.int_sign_sphi = 0.0
        self.int_sign_stheta = 0.0
        self.int_sign_spsi = 0.0

    def reset(self):
        """Reset internal supertwisting integral states."""
        self.int_sign_sz = 0.0
        self.int_sign_sphi = 0.0
        self.int_sign_stheta = 0.0
        self.int_sign_spsi = 0.0

    def step(self,
             x: np.ndarray,
             xd: np.ndarray,
             xd_dot: np.ndarray,
             d_hat: np.ndarray,
             dt: float):
        """
        Compute control inputs Fz, τφ, τθ, τψ for one time step.
        """

        p = self.p

        # ----------------------------------------------------------
        # 0) Unpack states, references, derivatives, disturbances
        # ----------------------------------------------------------
        # Current states
        z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot = x

        # Desired states
        z_d, z_dot_d, phi_d, phi_dot_d, theta_d, theta_dot_d, psi_d, psi_dot_d = xd

        # Desired second derivatives (˙x2d, ˙x4d, ˙x6d, ˙x8d)
        z_ddot_d   = xd_dot[1]
        phi_ddot_d = xd_dot[3]
        theta_ddot_d = xd_dot[5]
        psi_ddot_d = xd_dot[7]

        # Disturbance estimates
        dz_hat, dphi_hat, dtheta_hat, dpsi_hat = d_hat

        # ----------------------------------------------------------
        # 1) Tracking errors e1..e8  (ei = xi - xid_i)
        # ----------------------------------------------------------
        e1 = z         - z_d
        e2 = z_dot     - z_dot_d
        e3 = phi       - phi_d
        e4 = phi_dot   - phi_dot_d
        e5 = theta     - theta_d
        e6 = theta_dot - theta_dot_d
        e7 = psi       - psi_d
        e8 = psi_dot   - psi_dot_d

        # ----------------------------------------------------------
        # 2) Sliding surfaces sz, sφ, sθ, sψ (eq. (14))
        #     sz   = c1 e1 + e2
        #     sφ   = c2 e3 + e4
        #     sθ   = c3 e5 + e6
        #     sψ   = c4 e7 + e8
        # c1..c4 are in QuadrotorParams
        # ----------------------------------------------------------
        sz     = p.c1 * e1 + e2
        sphi   = p.c2 * e3 + e4
        stheta = p.c3 * e5 + e6
        spsi   = p.c4 * e7 + e8

        # ----------------------------------------------------------
        # 3) Update supertwisting integrals  ∫ sign(s) dt
        # ----------------------------------------------------------
        self.int_sign_sz     += _sign(sz)     * dt
        self.int_sign_sphi   += _sign(sphi)   * dt
        self.int_sign_stheta += _sign(stheta) * dt
        self.int_sign_spsi   += _sign(spsi)   * dt

        # ----------------------------------------------------------
        # 4) PD base control v1..v4 (eq. (12))
        #
        #   v1 = ˙x2d - Kpz     * e1 - Kdz     * e2
        #   v2 = ˙x4d - Kp_phi  * e3 - Kd_phi  * e4
        #   v3 = ˙x6d - Kp_theta* e5 - Kd_theta* e6
        #   v4 = ˙x8d - Kp_psi  * e7 - Kd_psi  * e8
        # ----------------------------------------------------------
        v1_pd = z_ddot_d      - p.Kpz      * e1 - p.Kdz      * e2
        v2_pd = phi_ddot_d    - p.Kp_phi   * e3 - p.Kd_phi   * e4
        v3_pd = theta_ddot_d  - p.Kp_theta * e5 - p.Kd_theta * e6
        v4_pd = psi_ddot_d    - p.Kp_psi   * e7 - p.Kd_psi   * e8

        # ----------------------------------------------------------
        # 5) Supertwisting modification (eq. (13))
        #
        #   v_new = v_pd
        #           - k1 * |s|^0.5 sign(s)
        #           - k2 * ∫ sign(s) dt
        #
        # Gains k1_*, k2_* are in QuadrotorParams.
        # ----------------------------------------------------------
        v1 = (
            v1_pd
            - p.k1_z    * np.sqrt(abs(sz))     * _sign(sz)
            - p.k2_z    * self.int_sign_sz
        )

        v2 = (
            v2_pd
            - p.k1_phi  * np.sqrt(abs(sphi))   * _sign(sphi)
            - p.k2_phi  * self.int_sign_sphi
        )

        v3 = (
            v3_pd
            - p.k1_theta * np.sqrt(abs(stheta)) * _sign(stheta)
            - p.k2_theta * self.int_sign_stheta
        )

        v4 = (
            v4_pd
            - p.k1_psi  * np.sqrt(abs(spsi))   * _sign(spsi)
            - p.k2_psi  * self.int_sign_spsi
        )

        # ----------------------------------------------------------
        # 6) Feedback linearization to real inputs (eq. (11))
        #
        #   Fz   = m / (cos φ cos θ) * ( g + ξz/m * x2 + v1 - d̂z )
        #
        #   τφ   = Ix * [ - x6 x8 (Iy - Iz)/Ix + Ir/Ix * x6 * ω̄
        #                 + ξφ/Ix * x4 + v2 - d̂φ ]
        #
        #   τθ   = Iy * [ - x4 x8 (Iz - Ix)/Iy + Ir/Iy * x4 * ω̄
        #                 + ξθ/Iy * x6 + v3 - d̂θ ]
        #
        #   τψ   = Iz * [ - x4 x6 (Ix - Iy)/Iz + ξψ/Iz * x8 + v4 - d̂ψ ]
        #
        # If params does not define omega_bar, gyroscopic term is ignored.
        # ----------------------------------------------------------
        phi_cos = np.cos(phi)
        theta_cos = np.cos(theta)
        denom = phi_cos * theta_cos

        # avoid singularity when cosφ cosθ → 0
        eps = 1e-6
        if abs(denom) < eps:
            denom = eps * np.sign(denom) if denom != 0 else eps

        omega_bar = getattr(p, "omega_bar", 0.0)

        Fz = p.m / denom * (
            p.g + (p.xi_z / p.m) * z_dot + v1 - dz_hat
        )

        tau_phi = p.Ix * (
            - theta_dot * psi_dot * (p.Iy - p.Iz) / p.Ix
            + p.Ir * theta_dot * omega_bar / p.Ix
            + p.xi_phi * phi_dot / p.Ix
            + v2 - dphi_hat
        )

        tau_theta = p.Iy * (
            - phi_dot * psi_dot * (p.Iz - p.Ix) / p.Iy
            + p.Ir * phi_dot * omega_bar / p.Iy
            + p.xi_theta * theta_dot / p.Iy
            + v3 - dtheta_hat
        )

        tau_psi = p.Iz * (
            - phi_dot * theta_dot * (p.Ix - p.Iy) / p.Iz
            + p.xi_psi * psi_dot / p.Iz
            + v4 - dpsi_hat
        )

        return float(Fz), float(tau_phi), float(tau_theta), float(tau_psi)
