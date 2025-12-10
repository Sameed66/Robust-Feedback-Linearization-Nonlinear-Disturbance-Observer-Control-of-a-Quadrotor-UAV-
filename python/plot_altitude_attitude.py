# plot_altitude_attitude.py

import numpy as np
import matplotlib.pyplot as plt
from sim_alt_attitude import simulate_altitude_attitude
from params_refs import multi_step_reference


def plot_altitude_attitude():
    # run simulation
    t, X, U, D, D_hat = simulate_altitude_attitude(T=15.0, dt=1e-3)

    # unpack states
    z     = X[:, 0]
    z_dot = X[:, 1]
    phi   = X[:, 2]
    theta = X[:, 4]
    psi   = X[:, 6]

    # generate reference signals for plotting
    z_ref   = np.zeros_like(t)
    psi_ref = np.zeros_like(t)

    for i, ti in enumerate(t):
        _, _, z_ref[i], psi_ref[i] = multi_step_reference(ti)

    # ----- Plotting -----

    fig, axs = plt.subplots(4, 1, figsize=(10, 14), sharex=True)
    fig.suptitle("Altitude + Attitude RFBL-STC with Disturbance Observer", fontsize=15)

    # 1) Altitude tracking
    axs[0].plot(t, z, label="z (m)")
    axs[0].plot(t, z_ref, '--', label="z_ref", linewidth=2)
    axs[0].set_ylabel("Altitude z (m)")
    axs[0].legend()
    axs[0].grid(True)

    # 2) Attitude: phi, theta, psi
    axs[1].plot(t, phi, label="phi (rad)")
    axs[1].plot(t, theta, label="theta (rad)")
    axs[1].set_ylabel("Roll φ, Pitch θ (rad)")
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(t, psi, label="psi (rad)")
    axs[2].plot(t, psi_ref, '--', label="psi_ref", linewidth=2)
    axs[2].set_ylabel("Yaw ψ (rad)")
    axs[2].legend()
    axs[2].grid(True)

    # 3) Disturbances vs estimates
    axs[3].plot(t, D[:, 0], label="d_z true")
    axs[3].plot(t, D_hat[:, 0], '--', label="d_z_hat")
    axs[3].plot(t, D[:, 1], label="d_phi true")
    axs[3].plot(t, D_hat[:, 1], '--', label="d_phi_hat")
    axs[3].plot(t, D[:, 2], label="d_theta true")
    axs[3].plot(t, D_hat[:, 2], '--', label="d_theta_hat")
    axs[3].plot(t, D[:, 3], label="d_psi true")
    axs[3].plot(t, D_hat[:, 3], '--', label="d_psi_hat")
    axs[3].set_ylabel("Disturbances")
    axs[3].set_xlabel("Time (s)")
    axs[3].legend(loc="upper right")
    axs[3].grid(True)

    plt.tight_layout()
    plt.show()

    # Additional optional plots --------------------------
    # Uncomment to visualize control inputs
    #
    plot_controls(t, U)
    # -----------------------------------------------------


def plot_controls(t, U):
    Fz = U[:, 0]
    tau_phi = U[:, 1]
    tau_theta = U[:, 2]
    tau_psi = U[:, 3]

    plt.figure(figsize=(10, 8))
    plt.subplot(4, 1, 1)
    plt.plot(t, Fz); plt.grid(True)
    plt.ylabel("Fz (N)")

    plt.subplot(4, 1, 2)
    plt.plot(t, tau_phi); plt.grid(True)
    plt.ylabel("tau_phi")

    plt.subplot(4, 1, 3)
    plt.plot(t, tau_theta); plt.grid(True)
    plt.ylabel("tau_theta")

    plt.subplot(4, 1, 4)
    plt.plot(t, tau_psi); plt.grid(True)
    plt.ylabel("tau_psi")
    plt.xlabel("Time (s)")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    plot_altitude_attitude()
