# xy_controller.py
import numpy as np
from params_refs import QuadrotorParams


def _sign(x):
    if x > 0: return 1.0
    if x < 0: return -1.0
    return 0.0


class XY_RFBL_Controller:
    """
    Robust Feedback Linearization + Supertwisting
    for X and Y position loops.

    Implements the outer-loop of the paper:
        X–Y position → desired roll φ_d, pitch θ_d
    """

    def __init__(self, params: QuadrotorParams):
        self.p = params

        # super-twisting integrals for x and y
        self.int_sign_sx = 0.0
        self.int_sign_sy = 0.0

    def reset(self):
        self.int_sign_sx = 0.0
        self.int_sign_sy = 0.0

    def step(self,
             x_pos: float, x_dot: float,
             y_pos: float, y_dot: float,
             xd: float, xd_dot: float, xd_ddot: float,
             yd: float, yd_dot: float, yd_ddot: float,
             d_hat_xy: np.ndarray,
             psi: float,
             Fz: float,
             dt: float):
        """
        Compute desired roll φ_d and pitch θ_d.

        Inputs:
            (x_pos, x_dot)
            (y_pos, y_dot)
            desired trajectories xd, yd, xd_dot, yd_dot, xd_ddot, yd_ddot
            disturbance estimates d_hat_xy = [dx_hat, dy_hat]
            current yaw angle psi (needed for transformation)
            current thrust Fz
            dt: integration step

        Outputs:
            φ_d, θ_d
        """

        p = self.p

        # disturbances
        dx_hat, dy_hat = d_hat_xy

        # --- Tracking errors ---
        ex  = x_pos - xd
        exd = x_dot - xd_dot

        ey  = y_pos - yd
        eyd = y_dot - yd_dot

        # --- Sliding surfaces (eq. 23) ---
        sx = p.c5 * ex + exd
        sy = p.c6 * ey + eyd

        # --- Update supertwisting integrals ---
        self.int_sign_sx += _sign(sx) * dt
        self.int_sign_sy += _sign(sy) * dt

        # --- Base PD control (eq. 24) ---
        v1x = xd_ddot - p.Kp_x * ex - p.Kd_x * exd
        v1y = yd_ddot - p.Kp_y * ey - p.Kd_y * eyd

        # --- Supertwisting modification (eq. 25) ---
        vx = (v1x
              - p.k1_x * np.sqrt(abs(sx)) * _sign(sx)
              - p.k2_x * self.int_sign_sx)

        vy = (v1y
              - p.k1_y * np.sqrt(abs(sy)) * _sign(sy)
              - p.k2_y * self.int_sign_sy)

        # --- Feedback linearization to get desired φ_d, θ_d (eq. 27) ---
        #   [vx]
        #   [vy] = (Fz/m) * R(psi) * [θ_d, φ_d]^T - disturbances
        #
        # where R(psi) = [ cosψ, -sinψ
        #                 sinψ,  cosψ ]
        #
        # Solve for [θ_d, φ_d]:

        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        denom = (Fz / p.m)

        # Invert rotation
        theta_d = ( cpsi * (vx + dx_hat) + spsi * (vy + dy_hat) ) / denom
        phi_d   = ( -spsi * (vx + dx_hat) + cpsi * (vy + dy_hat) ) / denom

        # small angle protection (optional clutch for simulation)
        theta_d = np.clip(theta_d, -0.5, 0.5)
        phi_d   = np.clip(phi_d,   -0.5, 0.5)

        return phi_d, theta_d

