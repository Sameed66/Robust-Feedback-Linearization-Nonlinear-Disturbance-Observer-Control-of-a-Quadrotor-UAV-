# dynamics.py
import numpy as np
from params_refs import QuadrotorParams, sinusoidal_disturbance

def quadrotor_dynamics(t: float,
                       x: np.ndarray,
                       u: np.ndarray,
                       params: QuadrotorParams) -> np.ndarray:
    """
    Continuous-time quadrotor dynamics with disturbances.

    State x: [ z, z_dot,
               phi, phi_dot,
               theta, theta_dot,
               psi, psi_dot,
               x, x_dot,
               y, y_dot ]

    Control u: [Fz, tau_phi, tau_theta, tau_psi]
    """
    # Unpack parameters
    m = params.m
    g = params.g
    Ix, Iy, Iz = params.Ix, params.Iy, params.Iz
    Ir = params.Ir
    xi_z = params.xi_z
    xi_phi, xi_theta, xi_psi = params.xi_phi, params.xi_theta, params.xi_psi
    xi_x, xi_y = params.xi_x, params.xi_y

    # Unpack state
    z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot, x_pos, x_dot, y_pos, y_dot = x

    # Unpack inputs
    Fz, tau_phi, tau_theta, tau_psi = u

    # Disturbances (same shape as in paper: 0.1 sin(2t) on each axis)
    dz = sinusoidal_disturbance(t)
    dphi = sinusoidal_disturbance(t)
    dtheta = sinusoidal_disturbance(t)
    dpsi = sinusoidal_disturbance(t)
    dx = sinusoidal_disturbance(t)
    dy = sinusoidal_disturbance(t)

    # Residual rotor angular disturbance term ω̄ (for now approximate as 0)
    w_bar = 0.0

    # Precompute trig
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta = np.cos(theta), np.sin(theta)
    c_psi, s_psi = np.cos(psi), np.sin(psi)

    # Allocate derivative
    xdot = np.zeros_like(x)

    # z dynamics
    xdot[0] = z_dot
    xdot[1] = (c_phi * c_theta) * Fz / m - g - xi_z * z_dot / m + dz

    # phi dynamics
    xdot[2] = phi_dot
    xdot[3] = (
        theta_dot * psi_dot * (Iy - Iz) / Ix
        - Ir / Ix * theta_dot * w_bar
        - xi_phi / Ix * phi_dot
        + tau_phi / Ix
        + dphi
    )

    # theta dynamics
    xdot[4] = theta_dot
    xdot[5] = (
        phi_dot * psi_dot * (Iz - Ix) / Iy
        - Ir / Iy * psi_dot * w_bar
        - xi_theta / Iy * theta_dot
        + tau_theta / Iy
        + dtheta
    )

    # psi dynamics
    xdot[6] = psi_dot
    xdot[7] = (
        phi_dot * theta_dot * (Ix - Iy) / Iz
        - xi_psi / Iz * psi_dot
        + tau_psi / Iz
        + dpsi
    )

    # x (position) dynamics
    xdot[8] = x_dot
    xdot[9] = (
        (c_phi * s_theta * c_psi + s_phi * s_psi) * Fz / m
        - xi_x * x_dot / m
        + dx
    )

    # y (position) dynamics
    xdot[10] = y_dot
    xdot[11] = (
        (c_phi * s_theta * s_psi - s_phi * c_psi) * Fz / m
        - xi_y * y_dot / m
        + dy
    )

    return xdot
