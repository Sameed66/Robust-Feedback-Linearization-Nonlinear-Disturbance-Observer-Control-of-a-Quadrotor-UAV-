# xy_position_controller.py

import numpy as np
from .params_refs import QuadrotorParams


def _sign(x: float) -> float:
    """Sign function with sign(0) = 0 (scalar)."""
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


class XYPositionRFBLSTC:
    """
    X–Y position RFBL + Supertwisting controller (outer loop).

    Based on eqs. (21)–(23) in the paper:

        x9  = x
        x10 = x_dot
        x11 = y
        x12 = y_dot

        ˙x10 = -ξ_x/m * x10 + (Fz/m) * Fx + d_x
        ˙x12 = -ξ_y/m * x12 + (Fz/m) * Fy + d_y

    Virtual control (with PD + STC on sliding surfaces):

        s_x = c5 * e9  + e10
        s_y = c6 * e11 + e12

        v_x_PD = x_ddot_d - Kp_x * e9 - Kd_x * e10
        v_y_PD = y_ddot_d - Kp_y * e11 - Kd_y * e12

        v_x = v_x_PD
              - k1_x * |s_x|^0.5 * sign(s_x)
              - k2_x * ∫ sign(s_x) dt

        v_y = v_y_PD
              - k1_y * |s_y|^0.5 * sign(s_y)
              - k2_y * ∫ sign(s_y) dt

    Feedback linearization to actual inputs Fx, Fy (similar structure to eq. (11)):

        Fx = m / Fz * ( v_x + (ξ_x/m) * x10 - d_hat_x )
        Fy = m / Fz * ( v_y + (ξ_y/m) * x12 - d_hat_y )

    Desired roll and pitch angles (eq. (3)):

        φ_des = 1/Fz * ( Fx * sin(ψ_d) - Fy * cos(ψ_d) )
        θ_des = 1/Fz * ( Fx * cos(ψ_d) - Fy * sin(ψ_d) )
    """

    def __init__(self, params: QuadrotorParams):
        self.p = params

        # supertwisting integrals ∫ sign(s) dt
        self.int_sign_sx = 0.0
        self.int_sign_sy = 0.0

    def reset(self):
        """Reset internal supertwisting integrals."""
        self.int_sign_sx = 0.0
        self.int_sign_sy = 0.0

    def step(self,
             x9_12: np.ndarray,
             x_ref: float,
             x_ref_dot: float,
             x_ref_ddot: float,
             y_ref: float,
             y_ref_dot: float,
             y_ref_ddot: float,
             Fz: float,
             d_hat_xy: np.ndarray,
             psi_d: float,
             dt: float):
        """
        Compute Fx, Fy and desired roll/pitch from X–Y position errors.

        Parameters
        ----------
        x9_12 : np.ndarray
            [x, x_dot, y, y_dot] = [x9, x10, x11, x12].
        x_ref, x_ref_dot, x_ref_ddot : float
            Desired x position, velocity, acceleration.
        y_ref, y_ref_dot, y_ref_ddot : float
            Desired y position, velocity, acceleration.
        Fz : float
            Current thrust from the inner (altitude) loop.
        d_hat_xy : np.ndarray
            Disturbance estimates [d_hat_x, d_hat_y] (for now can be zeros).
        psi_d : float
            Desired yaw angle ψ_d used in the mapping to φ_des, θ_des.
        dt : float
            Timestep.

        Returns
        -------
        Fx, Fy, phi_des, theta_des : tuple of floats
        """

        p = self.p

        # unpack states
        x_pos, x_dot, y_pos, y_dot = x9_12
        d_hat_x, d_hat_y = d_hat_xy

        # ----------------------------------------------------------
        # 1) Errors and sliding surfaces (eq. (23))
        #    e9 = x - x_d, e10 = x_dot - x_dot_d
        #    e11 = y - y_d, e12 = y_dot - y_dot_d
        #    s_x = c5 e9 + e10
        #    s_y = c6 e11 + e12
        # ----------------------------------------------------------
        e9  = x_pos - x_ref
        e10 = x_dot - x_ref_dot
        e11 = y_pos - y_ref
        e12 = y_dot - y_ref_dot

        s_x = p.c5 * e9  + e10
        s_y = p.c6 * e11 + e12

        # ----------------------------------------------------------
        # 2) Update supertwisting integrals ∫ sign(s) dt
        # ----------------------------------------------------------
        self.int_sign_sx += _sign(s_x) * dt
        self.int_sign_sy += _sign(s_y) * dt

        # ----------------------------------------------------------
        # 3) PD virtual control (like eq. (22) before STC)
        #     v_x_PD = x_ddot_d - Kp_x * e9 - Kd_x * e10
        #     v_y_PD = y_ddot_d - Kp_y * e11 - Kd_y * e12
        # ----------------------------------------------------------
        v_x_pd = x_ref_ddot - p.Kp_x * e9  - p.Kd_x * e10
        v_y_pd = y_ref_ddot - p.Kp_y * e11 - p.Kd_y * e12

        # ----------------------------------------------------------
        # 4) Supertwisting modification (per axis, like eq. (13))
        #     v_x = v_x_PD
        #           - k1_x |s_x|^0.5 sign(s_x)
        #           - k2_x ∫ sign(s_x) dt
        # ----------------------------------------------------------
        v_x = (
            v_x_pd
            - p.k1_x * np.sqrt(abs(s_x)) * _sign(s_x)
            - p.k2_x * self.int_sign_sx
        )

        v_y = (
            v_y_pd
            - p.k1_y * np.sqrt(abs(s_y)) * _sign(s_y)
            - p.k2_y * self.int_sign_sy
        )

        # ----------------------------------------------------------
        # 5) Feedback linearization to Fx, Fy (similar to eq. (11))
        #
        #    ˙x10 = -ξ_x/m x10 + (Fz/m) Fx + d_x
        #    => Fx = m/Fz * ( v_x + ξ_x/m x10 - d_hat_x )
        #
        #    ˙x12 = -ξ_y/m x12 + (Fz/m) Fy + d_y
        #    => Fy = m/Fz * ( v_y + ξ_y/m x12 - d_hat_y )
        # ----------------------------------------------------------
        # protect against very small Fz
        # eps = 1e-6
        # Fz_safe = Fz if abs(Fz) > eps else eps * _sign(Fz) if Fz != 0 else eps

        # Fx = p.m / Fz_safe * (v_x + (p.xi_x / p.m) * x_dot - d_hat_x)
        # Fy = p.m / Fz_safe * (v_y + (p.xi_y / p.m) * y_dot - d_hat_y)

        eps = 1e-6
        # protect against very small Fz
        if abs(Fz) > eps:
            Fz_safe = Fz
        elif Fz != 0.0:
            Fz_safe = eps * _sign(Fz)
        else:
            Fz_safe = eps

        Fx = p.m / Fz_safe * (v_x + (p.xi_x / p.m) * x_dot - d_hat_x)
        Fy = p.m / Fz_safe * (v_y + (p.xi_y / p.m) * y_dot - d_hat_y)


        # ----------------------------------------------------------
        # 6) Map Fx, Fy to desired roll & pitch (eq. (3))
        #
        #    φ_des = 1/Fz * ( Fx sin ψ_d - Fy cos ψ_d )
        #    θ_des = 1/Fz * ( Fx cos ψ_d - Fy sin ψ_d )
        #
        # again, guard against small Fz.
        # ----------------------------------------------------------
        sin_psi = np.sin(psi_d)
        cos_psi = np.cos(psi_d)

        phi_des   = (Fx * sin_psi - Fy * cos_psi) / Fz_safe
        theta_des = (Fx * cos_psi - Fy * sin_psi) / Fz_safe

        return float(Fx), float(Fy), float(phi_des), float(theta_des)
