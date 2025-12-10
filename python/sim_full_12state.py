# sim_full_12state.py

import numpy as np

from params_refs import (
    QuadrotorParams,
    sinusoidal_disturbance,
    multi_step_reference,
    circular_reference,
)
from rfbl_controller import RFBLAltitudeAttitudeController
from disturbance_observer import HarmonicDisturbanceObserver
from xy_position_controller import XYPositionRFBLSTC
from xy_disturbance_observer import XYHarmonicDisturbanceObserver


def quadrotor_dynamics_12state(
    x: np.ndarray,
    u: tuple[float, float, float, float],
    Fx: float,
    Fy: float,
    d_all: np.ndarray,
    p: QuadrotorParams,
) -> np.ndarray:
    """
    Full 12-state dynamics (MODEL A, as in the paper):

      x = [ z, z_dot,
            phi, phi_dot,
            theta, theta_dot,
            psi, psi_dot,
            x_pos, x_dot,
            y_pos, y_dot ]

    Inputs
    ------
    u   : (Fz, tau_phi, tau_theta, tau_psi)  [N, N·m]
    Fx, Fy : translational virtual inputs (from XYPositionRFBLSTC), unitless
    d_all  : [d_z, d_phi, d_theta, d_psi, d_x, d_y]
    """

    Fz, tau_phi, tau_theta, tau_psi = u
    dz, dphi, dtheta, dpsi, dx, dy = d_all

    (
        z, z_dot,
        phi, phi_dot,
        theta, theta_dot,
        psi, psi_dot,
        x_pos, x_dot,
        y_pos, y_dot,
    ) = x

    omega_bar = getattr(p, "omega_bar", 0.0)

    # --- altitude & attitude dynamics (matches RFBL model) ---
    z_ddot = (
        np.cos(phi) * np.cos(theta) / p.m * Fz
        - p.g
        - (p.xi_z / p.m) * z_dot
        + dz
    )

    phi_ddot = (
        theta_dot * psi_dot * (p.Iy - p.Iz) / p.Ix
        - p.Ir * theta_dot * omega_bar / p.Ix
        - p.xi_phi * phi_dot / p.Ix
        + tau_phi / p.Ix
        + dphi
    )

    theta_ddot = (
        phi_dot * psi_dot * (p.Iz - p.Ix) / p.Iy
        - p.Ir * phi_dot * omega_bar / p.Iy
        - p.xi_theta * theta_dot / p.Iy
        + tau_theta / p.Iy
        + dtheta
    )

    psi_ddot = (
        phi_dot * theta_dot * (p.Ix - p.Iy) / p.Iz
        - p.xi_psi * psi_dot / p.Iz
        + tau_psi / p.Iz
        + dpsi
    )

    # --- simplified translational dynamics (eq. 21-like) ---
    # x_ddot = -ξ_x/m * x_dot + (Fz/m) * Fx + d_x
    # y_ddot = -ξ_y/m * y_dot + (Fz/m) * Fy + d_y
    x_ddot = -p.xi_x / p.m * x_dot + (Fz / p.m) * Fx + dx
    y_ddot = -p.xi_y / p.m * y_dot + (Fz / p.m) * Fy + dy

    x_dot_vec = np.array(
        [
            z_dot,
            z_ddot,
            phi_dot,
            phi_ddot,
            theta_dot,
            theta_ddot,
            psi_dot,
            psi_ddot,
            x_dot,
            x_ddot,
            y_dot,
            y_ddot,
        ],
        dtype=float,
    )

    return x_dot_vec


def simulate_full_12state(
    T: float = 20.0,
    dt: float = 1e-3,
    ref_type: str = "multi_step",
):
    """
    Full 3D simulation with:

      • Inner loop: RFBL + STC for z, phi, theta, psi  (with 4-axis DO)
      • Outer loop: RFBL + STC for x, y               (with 2-axis DO)
      • 12-state quadrotor dynamics (MODEL A)

    ref_type:
        "multi_step" -> uses multi_step_reference
        "circular"   -> uses circular_reference
    """

    p = QuadrotorParams()

    # controllers
    inner_ctrl = RFBLAltitudeAttitudeController(p)
    outer_ctrl = XYPositionRFBLSTC(p)

    # observers
    att_observer = HarmonicDisturbanceObserver(p)        # for z,phi,theta,psi
    xy_observer = XYHarmonicDisturbanceObserver(p)       # for x,y

    N = int(T / dt) + 1
    t = np.linspace(0.0, T, N)

    # logs
    X = np.zeros((N, 12))   # states
    U = np.zeros((N, 4))    # [Fz, tau_phi, tau_theta, tau_psi]
    FXFY = np.zeros((N, 2)) # [Fx, Fy]
    D_true = np.zeros((N, 6))   # [d_z,d_phi,d_theta,d_psi,d_x,d_y]
    D_hat = np.zeros((N, 6))    # estimates in same order

    # === Initial conditions: start ON the reference ===
    if ref_type == "multi_step":
        x0_ref, y0_ref, z0_ref, psi0_ref = multi_step_reference(0.0)
    else:  # "circular"
        x0_ref, y0_ref, z0_ref, psi0_ref = circular_reference(0.0)

    x = np.zeros(12, dtype=float)

    # z, z_dot
    x[0] = z0_ref
    x[1] = 0.0

    # phi, phi_dot, theta, theta_dot
    x[2] = 0.0
    x[3] = 0.0
    x[4] = 0.0
    x[5] = 0.0

    # psi, psi_dot
    x[6] = psi0_ref
    x[7] = 0.0

    # x, x_dot, y, y_dot
    x[8]  = x0_ref
    x[9]  = 0.0
    x[10] = y0_ref
    x[11] = 0.0

    X[0, :] = x

    # previous control (for observers)
    u_prev = np.zeros(4, dtype=float)
    Fx_prev = 0.0
    Fy_prev = 0.0

    for k in range(N - 1):
        tk = t[k]

        # -------------------------------------------------
        # 1) Reference trajectory (x_d, y_d, z_d, psi_d)
        # -------------------------------------------------
        if ref_type == "multi_step":
            x_d, y_d, z_d, psi_d = multi_step_reference(tk)
            x_dot_d  = 0.0
            x_ddot_d = 0.0
            y_dot_d  = 0.0
            y_ddot_d = 0.0
        else:  # "circular"
            x_d, y_d, z_d, psi_d = circular_reference(tk)
            # x_d = cos(0.5 t); y_d = sin(0.5 t)
            x_dot_d  = -0.5 * np.sin(0.5 * tk)
            x_ddot_d = -0.25 * np.cos(0.5 * tk)
            y_dot_d  =  0.5 * np.cos(0.5 * tk)
            y_ddot_d = -0.25 * np.sin(0.5 * tk)

        # altitude & yaw references derivatives: z, psi constant => 0
        z_dot_d = 0.0
        z_ddot_d = 0.0
        psi_dot_d = 0.0
        psi_ddot_d = 0.0

        # -------------------------------------------------
        # 2) True disturbances (sinusoidal on all 6 axes)
        # -------------------------------------------------
        dz_true      = sinusoidal_disturbance(tk)
        dphi_true    = sinusoidal_disturbance(tk)
        dtheta_true  = sinusoidal_disturbance(tk)
        dpsi_true    = sinusoidal_disturbance(tk)
        dx_true      = sinusoidal_disturbance(tk)
        dy_true      = sinusoidal_disturbance(tk)

        d_true_vec = np.array(
            [dz_true, dphi_true, dtheta_true, dpsi_true, dx_true, dy_true],
            dtype=float,
        )

        # -------------------------------------------------
        # 3) Disturbance observers (use previous controls)
        # -------------------------------------------------
        x_8 = x[0:8]
        x_xy = x[8:12]

        d_hat_att = att_observer.step(x_8, tuple(u_prev), dt)  # (4,)
        d_hat_xy = xy_observer.step(
            x_xy, Fz=u_prev[0], Fx=Fx_prev, Fy=Fy_prev, dt=dt
        )  # (2,)

        d_hat_all = np.concatenate([d_hat_att, d_hat_xy])  # length 6

        # -------------------------------------------------
        # 4) Outer loop X–Y position control (RFBL + STC)
        # -------------------------------------------------
        Fx, Fy, phi_des, theta_des = outer_ctrl.step(
            x9_12=x_xy,
            x_ref=x_d,
            x_ref_dot=x_dot_d,
            x_ref_ddot=x_ddot_d,
            y_ref=y_d,
            y_ref_dot=y_dot_d,
            y_ref_ddot=y_ddot_d,
            Fz=u_prev[0] if u_prev[0] != 0.0 else p.m * p.g,
            d_hat_xy=d_hat_xy,
            psi_d=psi_d,
            dt=dt,
        )

        # -------------------------------------------------
        # 5) Inner loop altitude/attitude control
        # -------------------------------------------------
        phi_d = phi_des
        theta_d = theta_des
        phi_dot_d = 0.0
        theta_dot_d = 0.0

        xd = np.array(
            [
                z_d,      z_dot_d,
                phi_d,    phi_dot_d,
                theta_d,  theta_dot_d,
                psi_d,    psi_dot_d,
            ],
            dtype=float,
        )

        xd_dot = np.zeros(8, dtype=float)
        xd_dot[1] = z_ddot_d
        xd_dot[3] = 0.0
        xd_dot[5] = 0.0
        xd_dot[7] = psi_ddot_d

        Fz, tau_phi, tau_theta, tau_psi = inner_ctrl.step(
            x=x_8,
            xd=xd,
            xd_dot=xd_dot,
            d_hat=d_hat_att,
            dt=dt,
        )

        u = np.array([Fz, tau_phi, tau_theta, tau_psi], dtype=float)

        # -------------------------------------------------
        # 6) Integrate full 12-state dynamics (Euler)
        # -------------------------------------------------
        x_dot_vec = quadrotor_dynamics_12state(
            x=x,
            u=tuple(u),
            Fx=Fx,
            Fy=Fy,
            d_all=d_true_vec,
            p=p,
        )

        x = x + dt * x_dot_vec

        # -------------------------------------------------
        # 7) Log
        # -------------------------------------------------
        X[k + 1, :] = x
        U[k, :] = u
        FXFY[k, :] = np.array([Fx, Fy], dtype=float)
        D_true[k, :] = d_true_vec
        D_hat[k, :] = d_hat_all

        u_prev = u
        Fx_prev = Fx
        Fy_prev = Fy

    D_true[-1, :] = D_true[-2, :]
    D_hat[-1, :] = D_hat[-2, :]
    FXFY[-1, :] = FXFY[-2, :]

    return t, X, U, FXFY, D_true, D_hat


if __name__ == "__main__":
    t, X, U, FXFY, D_true, D_hat = simulate_full_12state(
        T=10.0,
        dt=1e-3,
        ref_type="circular",
    )
    print("Full 12-state simulation finished. Final state:")
    print(X[-1])
