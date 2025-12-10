# params_and_refs.py
from dataclasses import dataclass
import numpy as np

@dataclass
class QuadrotorParams:
    # === Table 17: Parrot Mambo quadrotor parameters ===
    m: float  = 0.063          # [kg]
    g: float  = 9.81           # [m/s^2]

    # Moments of inertia [kg m^2]
    Ix: float = 5.8286e-5
    Iy: float = 7.1691e-5
    Iz: float = 1.0e-4

    # Rotor inertia (not given; small positive value)
    Ir: float = 0.1021e-6      # [kg m^2]

    # Aerodynamic damping coefficients
    # ξz, ξφ, ξθ, ξψ, ξx, ξy
    xi_z: float = 0.075
    xi_phi: float = 0.075
    xi_theta: float = 0.075
    xi_psi: float = 0.075
    xi_x: float = 0.075
    xi_y: float = 0.075

    # Geometry
    l: float   = 0.062         # [m] arm length

    # Thrust / moment coefficients (used for motor mixing, NOT dynamics)
    kF: float  = 0.01          # [N / (PWM/ω)^2] – used only in motor_mixer
    kM: float  = 7.8263e-4     # [Nm / (PWM/ω)^2]

    # Disturbance observer frequency (d(t) = 0.1 sin(2 t))
    do_omega: float = 2.0      # [rad/s]

    # === Table 18: RFBL controller gains ===

    # Kp for all axes: Kpz,Kpφ,Kpθ,Kpψ,Kpx,Kpy = 3
    Kpz: float    = 3.0
    Kp_phi: float = 3.0
    Kp_theta: float = 3.0
    Kp_psi: float = 3.0
    Kp_x: float   = 3.0
    Kp_y: float   = 3.0

    # Kd for all axes: Kdz,Kdφ,Kdθ,Kdψ,Kdx,Kdy = 6
    Kdz: float    = 6.0
    Kd_phi: float = 6.0
    Kd_theta: float = 6.0
    Kd_psi: float = 6.0
    Kd_x: float   = 6.0
    Kd_y: float   = 6.0

    # Super-twisting gains (Table 18)
    # k1z = 7
    k1_z: float = 7.0
    # k1φ = 4
    k1_phi: float = 4.0
    # k1θ,k1ψ,k1x,k1y = 6
    k1_theta: float = 6.0
    k1_psi: float   = 6.0
    k1_x: float     = 6.0
    k1_y: float     = 6.0

    # k2φ = 1.1
    k2_phi: float = 1.1
    # k2z,k2θ,k2ψ = 1
    k2_z: float    = 1.0
    k2_theta: float = 1.0
    k2_psi: float   = 1.0
    # paper typo: "k2x 0.5, k2x 0.6" => interpret as k2x=0.5, k2y=0.6
    k2_x: float    = 0.5
    k2_y: float    = 0.6

    # Sliding surface coefficients: c1..c6 = 10
    c1: float = 10.0
    c2: float = 10.0
    c3: float = 10.0
    c4: float = 10.0
    c5: float = 10.0
    c6: float = 10.0


# ===== Disturbance and reference trajectories =====

def sinusoidal_disturbance(t: float, amplitude: float = 0.1, omega: float = 2.0) -> float:
    """d(t) = 0.1 sin(2 t) as in the paper."""
    return amplitude * np.sin(omega * t)


def multi_step_reference(t: float):
    """
    Multi-step trajectory (matches your earlier definition):
        t ∈ [0,5]:   x=2, y=1
        t ∈ (5,10]:  x=3, y=2
        t > 10:      x=5, y=4
        z = 2, ψ = π/6
    """
    if t <= 5.0:
        x_d = 2.0
        y_d = 1.0
    elif t <= 10.0:
        x_d = 3.0
        y_d = 2.0
    else:
        x_d = 5.0
        y_d = 4.0

    z_d = 2.0
    psi_d = np.pi / 6.0
    return x_d, y_d, z_d, psi_d


def circular_reference(t: float):
    """
    Circular trajectory (classic circle):
        x_d = cos(0.5 t)
        y_d = sin(0.5 t)
        z_d = 1
        ψ_d = 0
    """
    x_d = np.cos(0.5 * t)
    y_d = np.sin(0.5 * t)
    z_d = 1.0
    psi_d = 0.0
    return x_d, y_d, z_d, psi_d
