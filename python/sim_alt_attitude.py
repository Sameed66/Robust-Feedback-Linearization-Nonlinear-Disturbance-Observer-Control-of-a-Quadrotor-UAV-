# sim_alt_attitude.py

import numpy as np
from params_refs import QuadrotorParams, sinusoidal_disturbance, multi_step_reference
from rfbl_controller import RFBLAltitudeAttitudeController
from disturbance_observer import HarmonicDisturbanceObserver


def quadrotor_dynamics_8state(x: np.ndarray,
                              u: tuple[float, float, float, float],
                              d: np.ndarray,
                              p: QuadrotorParams) -> np.ndarray:
    """
    8-state dynamics: [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]^T

    Used here only for altitude/attitude simulation.
    You can replace this with a call to your dynamics.py if you already
    have a more detailed 12-state model.

    Inputs
    ------
    x : np.ndarray (8,)
        state
    u : (Fz, tau_phi, tau_theta, tau_psi)
    d : np.ndarray (4,)
        true disturbances [d_z, d_phi, d_theta, d_psi]
    p : QuadrotorParams
    """
    Fz, tau_phi, tau_theta, tau_psi = u
    dz, dphi, dtheta, dpsi = d
    z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot = x

    omega_bar = getattr(p, "omega_bar", 0.0)

    # kinematics
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

    x_dot = np.array([
        z_dot,
        z_ddot,
        phi_dot,
        phi_ddot,
        theta_dot,
        theta_ddot,
        psi_dot,
        psi_ddot
    ], dtype=float)

    return x_dot


def simulate_altitude_attitude(T: float = 15.0,
                               dt: float = 1e-3,
                               use_multistep_ref: bool = True):
    """
    Simulate altitude (z) + attitude (phi, theta, psi) with:

      • RFBL + supertwisting controller (inner loop)
      • Harmonic disturbance observer
      • Sinusoidal disturbances on all four axes

    For now:
      - outer position controller (x, y) is NOT used yet (next step)
      - phi_d = theta_d = 0
      - z_d, psi_d come from either:
           - multi_step_reference(t)  (if use_multistep_ref=True)
           - simple constants otherwise

    Returns
    -------
    t   : (N,)
    X   : (N, 8)   states
    U   : (N, 4)   control inputs
    D   : (N, 4)   true disturbances
    D_hat : (N, 4) disturbance estimates
    """
    p = QuadrotorParams()

    controller = RFBLAltitudeAttitudeController(p)
    observer = HarmonicDisturbanceObserver(p)

    N = int(T / dt) + 1
    t = np.linspace(0.0, T, N)

    # allocate logs
    X = np.zeros((N, 8))    # [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]
    U = np.zeros((N, 4))    # [Fz, tau_phi, tau_theta, tau_psi]
    D = np.zeros((N, 4))    # true disturbances
    D_hat = np.zeros((N, 4))  # estimated disturbances

    # initial state: hovering at z=0, all angles 0
    x = np.zeros(8)
    X[0, :] = x

    # initial control & disturbance estimates
    u_prev = np.zeros(4)
    d_hat_prev = np.zeros(4)

    for k in range(N - 1):
        tk = t[k]

        # -----------------------------
        # 1) Generate references for z_d and psi_d
        # -----------------------------
        if use_multistep_ref:
            # multi_step_reference gives x_d, y_d, z_d, psi_d
            _, _, z_d, psi_d = multi_step_reference(tk)
        else:
            z_d = 1.0
            psi_d = 0.0

        # For now, outer loop is not implemented:
        #   phi_d = theta_d = 0, and all derivatives / 2nd derivatives are 0.
        phi_d = 0.0
        theta_d = 0.0

        # desired velocities
        z_dot_d = 0.0
        phi_dot_d = 0.0
        theta_dot_d = 0.0
        psi_dot_d = 0.0

        # desired accelerations (˙x2d, ˙x4d, ˙x6d, ˙x8d)
        z_ddot_d = 0.0
        phi_ddot_d = 0.0
        theta_ddot_d = 0.0
        psi_ddot_d = 0.0

        xd = np.array([
            z_d, z_dot_d,
            phi_d, phi_dot_d,
            theta_d, theta_dot_d,
            psi_d, psi_dot_d
        ], dtype=float)

        xd_dot = np.zeros(8)
        xd_dot[1] = z_ddot_d
        xd_dot[3] = phi_ddot_d
        xd_dot[5] = theta_ddot_d
        xd_dot[7] = psi_ddot_d

        # -----------------------------
        # 2) True disturbances (sinusoidal on each axis)
        # -----------------------------
        dz_true = sinusoidal_disturbance(tk)
        dphi_true = sinusoidal_disturbance(tk)
        dtheta_true = sinusoidal_disturbance(tk)
        dpsi_true = sinusoidal_disturbance(tk)

        d_true_vec = np.array([dz_true, dphi_true, dtheta_true, dpsi_true], dtype=float)

        # -----------------------------
        # 3) Disturbance observer update
        #    (use previous control input u_prev)
        # -----------------------------
        d_hat_vec = observer.step(x, tuple(u_prev), dt)
        D_hat[k, :] = d_hat_vec

        # -----------------------------
        # 4) RFBL + STC controller (uses d_hat)
        # -----------------------------
        Fz, tau_phi, tau_theta, tau_psi = controller.step(
            x=x,
            xd=xd,
            xd_dot=xd_dot,
            d_hat=d_hat_vec,
            dt=dt
        )

        u = np.array([Fz, tau_phi, tau_theta, tau_psi], dtype=float)

        # -----------------------------
        # 5) Integrate dynamics (Euler)
        # -----------------------------
        x_dot = quadrotor_dynamics_8state(x, tuple(u), d_true_vec, p)
        x = x + dt * x_dot

        # -----------------------------
        # 6) Log everything
        # -----------------------------
        X[k + 1, :] = x
        U[k, :] = u
        D[k, :] = d_true_vec

        # update for next step
        u_prev = u
        d_hat_prev = d_hat_vec  # not used explicitly now, but kept for debugging

    # final D_hat row (for consistency)
    D_hat[-1, :] = D_hat[-2, :]

    return t, X, U, D, D_hat


if __name__ == "__main__":
    # simple test run (no plotting here; you can plot in a separate script)
    t, X, U, D, D_hat = simulate_altitude_attitude()
    print("Simulation finished. Final state:", X[-1])
