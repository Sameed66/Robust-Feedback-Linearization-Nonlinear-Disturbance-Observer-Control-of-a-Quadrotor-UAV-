# simulate_altitude_attitude.py
"""
Altitude & attitude simulation:
- Uses QuadrotorParams (params_refs.py)
- Uses RFBLAltitudeAttitudeController (rfbl_controller.py)
- Uses HarmonicDisturbanceObserver (disturbance_observer.py)
- Uses quadrotor_dynamics (dynamics.py)

State in dynamics.py is 12D:
    [z, z_dot,
     phi, phi_dot,
     theta, theta_dot,
     psi, psi_dot,
     x, x_dot,
     y, y_dot]

Controller/observer only use the first 8 states.
"""

import numpy as np
import matplotlib.pyplot as plt

from params_refs import QuadrotorParams, multi_step_reference, sinusoidal_disturbance
from rfbl_controller import RFBLAltitudeAttitudeController
from disturbance_observer import HarmonicDisturbanceObserver
from dynamics import quadrotor_dynamics  # defined in dynamics.py


def simulate_altitude_attitude(
    t_final: float = 20.0,
    dt: float = 0.001,
    use_multi_step_ref: bool = True,
):
    """
    Run a simulation for altitude + attitude only.

    Parameters
    ----------
    t_final : float
        Simulation end time [s]
    dt : float
        Time step [s]
    use_multi_step_ref : bool
        If True: use multi_step_reference (z_d = 2, psi_d = pi/6, etc.)
        If False: use a simple constant reference (z_d=1, psi_d=0).
    """

    # ---------------------------------------------------------------------
    # 1. Setup parameters, controller, observer
    # ---------------------------------------------------------------------
    params = QuadrotorParams()

    ctrl = RFBLAltitudeAttitudeController(params)
    obs = HarmonicDisturbanceObserver(params)

    # initial state: [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot, x, x_dot, y, y_dot]
    x = np.zeros(12)
    # (optionally start at some offset)
    x[0] = 0.0   # z
    x[6] = 0.0   # psi
    # x[8], x[10] (x_pos, y_pos) already 0

    # initial control (hover approx)
    Fz = params.m * params.g
    tau_phi = 0.0
    tau_theta = 0.0
    tau_psi = 0.0
    u = (Fz, tau_phi, tau_theta, tau_psi)

    # for logging
    n_steps = int(t_final / dt) + 1
    t_log = np.zeros(n_steps)
    x_log = np.zeros((n_steps, 12))   # 12 states now
    u_log = np.zeros((n_steps, 4))
    d_true_log = np.zeros((n_steps, 4))  # [dz, dphi, dtheta, dpsi]
    d_hat_log = np.zeros((n_steps, 4))

    # ---------------------------------------------------------------------
    # 2. Simulation loop
    # ---------------------------------------------------------------------
    for k in range(n_steps):
        t = k * dt
        t_log[k] = t
        x_log[k, :] = x
        u_log[k, :] = u

        # -------------------------------------------------------------
        # 2.1 True disturbances (for z, phi, theta, psi)
        #     same sinusoidal form on each axis, as in the paper
        #     dynamics.py also uses sinusoidal_disturbance(t) internally,
        #     so this matches the actual disturbances.
        # -------------------------------------------------------------
        dz_true = sinusoidal_disturbance(t)
        dphi_true = sinusoidal_disturbance(t)
        dtheta_true = sinusoidal_disturbance(t)
        dpsi_true = sinusoidal_disturbance(t)
        d_true = np.array([dz_true, dphi_true, dtheta_true, dpsi_true])
        d_true_log[k, :] = d_true

        # -------------------------------------------------------------
        # 2.2 Disturbance observer update (using x[:8] and previous input u)
        #      returns d_hat = [dz_hat, dphi_hat, dtheta_hat, dpsi_hat]
        # -------------------------------------------------------------
        d_hat = obs.step(x[:8], u, dt)   # use only the first 8 states
        d_hat_log[k, :] = d_hat

        # -------------------------------------------------------------
        # 2.3 Build reference for z, phi, theta, psi
        #
        # For this step:
        #   - Use multi_step_reference for z_d and psi_d if requested
        #   - Keep phi_d = 0, theta_d = 0
        #   - All desired derivatives and second derivatives are 0
        # -------------------------------------------------------------
        if use_multi_step_ref:
            _, _, z_d, psi_d = multi_step_reference(t)
        else:
            z_d = 1.0
            psi_d = 0.0

        phi_d = 0.0
        theta_d = 0.0

        # desired velocities and accelerations = 0 for this step
        z_dot_d = 0.0
        phi_dot_d = 0.0
        theta_dot_d = 0.0
        psi_dot_d = 0.0

        z_ddot_d = 0.0
        phi_ddot_d = 0.0
        theta_ddot_d = 0.0
        psi_ddot_d = 0.0

        # build xd and xd_dot arrays compatible with RFBL controller
        xd = np.array([
            z_d, z_dot_d,
            phi_d, phi_dot_d,
            theta_d, theta_dot_d,
            psi_d, psi_dot_d
        ])

        # xd_dot: 2nd derivatives live at indices 1,3,5,7
        xd_dot = np.zeros(8)
        xd_dot[1] = z_ddot_d
        xd_dot[3] = phi_ddot_d
        xd_dot[5] = theta_ddot_d
        xd_dot[7] = psi_ddot_d

        # -------------------------------------------------------------
        # 2.4 RFBL + STC controller to get new control inputs
        # -------------------------------------------------------------
        Fz, tau_phi, tau_theta, tau_psi = ctrl.step(
            x=x[:8],       # only first 8 states
            xd=xd,
            xd_dot=xd_dot,
            d_hat=d_hat,
            dt=dt
        )
        u = (Fz, tau_phi, tau_theta, tau_psi)

        # -------------------------------------------------------------
        # 2.5 System dynamics integration (Euler)
        #     quadrotor_dynamics DOES NOT take 'disturbances=' kwarg,
        #     and already injects sinusoidal disturbances internally.
        # -------------------------------------------------------------
        xdot = quadrotor_dynamics(t, x, u, params)
        x = x + xdot * dt

    # ---------------------------------------------------------------------
    # 3. Plot some results (basic)
    # ---------------------------------------------------------------------
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Altitude
    axs[0].plot(t_log, x_log[:, 0], label="z")
    axs[0].set_ylabel("z [m]")
    axs[0].grid(True)
    axs[0].legend()

    # Attitude
    axs[1].plot(t_log, x_log[:, 2], label="phi (roll)")
    axs[1].plot(t_log, x_log[:, 4], label="theta (pitch)")
    axs[1].plot(t_log, x_log[:, 6], label="psi (yaw)")
    axs[1].set_ylabel("Angles [rad]")
    axs[1].grid(True)
    axs[1].legend()

    # Disturbance vs estimate (z-axis as example)
    axs[2].plot(t_log, d_true_log[:, 0], label="d_z true")
    axs[2].plot(t_log, d_hat_log[:, 0], "--", label="d_z hat")
    axs[2].set_ylabel("Disturbance z")
    axs[2].set_xlabel("Time [s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

    return t_log, x_log, u_log, d_true_log, d_hat_log


if __name__ == "__main__":
    simulate_altitude_attitude()
