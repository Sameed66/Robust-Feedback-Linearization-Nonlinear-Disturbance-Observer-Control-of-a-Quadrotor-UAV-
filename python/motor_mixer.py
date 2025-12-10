# motor_mixer.py

import numpy as np
from params_refs import QuadrotorParams

def motor_mixing(Fz, tau_phi, tau_theta, tau_psi, p: QuadrotorParams):
    """
    Map total thrust + body torques to motor speeds (ω1..ω4).

    Model:
        [ Fz      ]   [ kF   kF    kF    kF   ] [ ω1^2 ]
        [ τφ      ] = [ 0   kF l   0   -kF l ] [ ω2^2 ]
        [ τθ      ]   [ -kF l  0  kF l  0    ] [ ω3^2 ]
        [ τψ      ]   [ kM  -kM   kM  -kM   ] [ ω4^2 ]

    Returns ω in rad/s (non-negative).
    """
    kF = p.kF
    kM = p.kM
    l  = p.l

    M = np.array([
        [ kF,      kF,      kF,      kF     ],
        [ 0.0,     kF*l,    0.0,    -kF*l  ],
        [-kF*l,    0.0,     kF*l,    0.0    ],
        [ kM,     -kM,      kM,     -kM     ],
    ])

    U = np.array([Fz, tau_phi, tau_theta, tau_psi], dtype=float)

    # Solve M * w_sq = U
    try:
        w_sq = np.linalg.solve(M, U)
    except np.linalg.LinAlgError:
        # Fallback: pseudo-inverse
        w_sq = np.linalg.pinv(M).dot(U)

    # Motor speeds must be non-negative
    w_sq = np.clip(w_sq, 0.0, None)
    w = np.sqrt(w_sq)

    return w  # [ω1, ω2, ω3, ω4]
