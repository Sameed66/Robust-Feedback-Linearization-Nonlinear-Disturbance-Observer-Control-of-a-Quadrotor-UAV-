# simulate_full_12state.py

import numpy as np
import matplotlib.pyplot as plt

from params_refs import (
    QuadrotorParams,
    multi_step_reference,
    circular_reference,
    sinusoidal_disturbance,
)
from rfbl_controller import RFBLAltitudeAttitudeController
from disturbance_observer import HarmonicDisturbanceObserver
from xy_controller import XY_RFBL_Controller
from xy_disturbance_observer import XYHarmonicDisturbanceObserver
from dynamics import quadrotor_dynamics


def simulate_full(
    t_final: float = 20.0,
    dt: float = 0.001,
    use_circular_ref: bool = False,
):
    """
    Full 12-state quadrotor simulation with:
    - XY RFBL + STC (outer loop)
    - Altitude & attitude RFBL + STC (inner loop)
    - Harmonic disturbance observers for z,phi,theta,psi and x,y
    """

    params = QuadrotorParams()

    # controllers & observers
    inner_ctrl = RFBLAltitudeAttitudeController(params)
    xy_ctrl = XY_RFBL_Controller(params)

    obs_rot = HarmonicDisturbanceObserver(params)   # z, phi, theta, psi
    obs_xy = XYHarmonicDisturbanceObserver(params)  # x, y

    # initial 12D state
    # [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot, x, x_dot, y, y_dot]
    x = np.zeros(12)

    # start near ground, at origin
    x[0] = 0.0  # z
    x[8] = 0.0  # x
    x[10] = 0.0 # y

    # initial control (hover)
    Fz = params.m * params.g
    tau_phi = 0.0
    tau_theta = 0.0
    tau_psi = 0.0
    u = (Fz, tau_phi, tau_theta, tau_psi)

    n_steps = int(t_final / dt) + 1
    t_log = np.zeros(n_steps)
    x_log = np.zeros((n_steps, 12))
    u_log = np.zeros((n_steps, 4))

    d_true_rot_log = np.zeros((n_steps, 4))  # dz, dphi, dtheta, dpsi
    d_hat_rot_log = np.zeros((n_steps, 4))
    d_true_xy_log = np.zeros((n_steps, 2))   # dx, dy
    d_hat_xy_log = np.zeros((n_steps, 2))

    for k in range(n_steps):
        t = k * dt
        t_log[k] = t
        x_log[k, :] = x
        u_log[k, :] = u

        # -------------------------------------------------
        # True disturbances (for logging / comparison)
        # (dynamics.py uses same sinusoidal_disturbance(t))
        # -------------------------------------------------
        dz_true = sinusoidal_disturbance(t)
        dphi_true = sinusoidal_disturbance(t)
        dtheta_true = sinusoidal_disturbance(t)
        dpsi_true = sinusoidal_disturbance(t)
        d_true_rot = np.array([dz_true, dphi_true, dtheta_true, dpsi_true])
        d_true_rot_log[k, :] = d_true_rot

        dx_true = sinusoidal_disturbance(t)
        dy_true = sinusoidal_disturbance(t)
        d_true_xy = np.array([dx_true, dy_true])
        d_true_xy_log[k, :] = d_true_xy

        # -------------------------------------------------
        # Disturbance observers
        # -------------------------------------------------
        d_hat_rot = obs_rot.step(x[:8], u, dt)     # [dz_hat, dphi_hat, dtheta_hat, dpsi_hat]
        d_hat_xy = obs_xy.step(x, u, dt)           # [dx_hat, dy_hat]

        d_hat_rot_log[k, :] = d_hat_rot
        d_hat_xy_log[k, :] = d_hat_xy

        # -------------------------------------------------
        # Reference generation for (x,y,z,psi)
        # -------------------------------------------------
        if use_circular_ref:
            x_d, y_d, z_d, psi_d = circular_reference(t)
        else:
            x_d, y_d, z_d, psi_d = multi_step_reference(t)

        # for now, desired velocities & accelerations = 0
        x_dot_d = 0.0
        y_dot_d = 0.0
        x_ddot_d = 0.0
        y_ddot_d = 0.0

        # -------------------------------------------------
        # XY outer-loop RFBL + STC → desired φ_d, θ_d
        # -------------------------------------------------
        x_pos, x_dot = x[8], x[9]
        y_pos, y_dot = x[10], x[11]
        psi = x[6]

        Fz_current = u[0]

        phi_d, theta_d = xy_ctrl.step(
            x_pos=x_pos, x_dot=x_dot,
            y_pos=y_pos, y_dot=y_dot,
            xd=x_d, xd_dot=x_dot_d, xd_ddot=x_ddot_d,
            yd=y_d, yd_dot=y_dot_d, yd_ddot=y_ddot_d,
            d_hat_xy=d_hat_xy,
            psi=psi,
            Fz=Fz_current,
            dt=dt,
        )

        # For inner loop, desired velocities and accelerations = 0
        z_dot_d = 0.0
        phi_dot_d = 0.0
        theta_dot_d = 0.0
        psi_dot_d = 0.0

        z_ddot_d = 0.0
        phi_ddot_d = 0.0
        theta_ddot_d = 0.0
        psi_ddot_d = 0.0

        # -------------------------------------------------
        # Build xd & xd_dot for inner altitude+attitude controller
        # order: [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]
        # -------------------------------------------------
        xd_inner = np.array([
            z_d, z_dot_d,
            phi_d, phi_dot_d,
            theta_d, theta_dot_d,
            psi_d, psi_dot_d
        ])

        xd_dot_inner = np.zeros(8)
        xd_dot_inner[1] = z_ddot_d
        xd_dot_inner[3] = phi_ddot_d
        xd_dot_inner[5] = theta_ddot_d
        xd_dot_inner[7] = psi_ddot_d

        # -------------------------------------------------
        # Inner RFBL altitude+attitude controller
        # -------------------------------------------------
        Fz, tau_phi, tau_theta, tau_psi = inner_ctrl.step(
            x=x[:8],
            xd=xd_inner,
            xd_dot=xd_dot_inner,
            d_hat=d_hat_rot,
            dt=dt,
        )
        u = (Fz, tau_phi, tau_theta, tau_psi)

        # -------------------------------------------------
        # Dynamics integration (Euler)
        # -------------------------------------------------
        xdot = quadrotor_dynamics(t, x, u, params)
        x = x + xdot * dt

    # -----------------------------
    # Simple plots
    # -----------------------------
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    # Positions
    axs[0].plot(t_log, x_log[:, 8], label="x")
    axs[0].plot(t_log, x_log[:, 10], label="y")
    axs[0].set_ylabel("Position [m]")
    axs[0].grid(True)
    axs[0].legend()

    # Altitude
    axs[1].plot(t_log, x_log[:, 0], label="z")
    axs[1].set_ylabel("z [m]")
    axs[1].grid(True)
    axs[1].legend()

    # Attitude
    axs[2].plot(t_log, x_log[:, 2], label="phi")
    axs[2].plot(t_log, x_log[:, 4], label="theta")
    axs[2].plot(t_log, x_log[:, 6], label="psi")
    axs[2].set_ylabel("Angles [rad]")
    axs[2].grid(True)
    axs[2].legend()

    # Disturbance x: true vs estimate
    axs[3].plot(t_log, d_true_xy_log[:, 0], label="d_x true")
    axs[3].plot(t_log, d_hat_xy_log[:, 0], "--", label="d_x hat")
    axs[3].set_ylabel("d_x")
    axs[3].set_xlabel("Time [s]")
    axs[3].grid(True)
    axs[3].legend()

    plt.tight_layout()
    plt.show()

    return {
        "t": t_log,
        "x": x_log,
        "u": u_log,
        "d_true_rot": d_true_rot_log,
        "d_hat_rot": d_hat_rot_log,
        "d_true_xy": d_true_xy_log,
        "d_hat_xy": d_hat_xy_log,
    }


if __name__ == "__main__":
    # multi-step trajectory by default
    simulate_full(use_circular_ref=False)
