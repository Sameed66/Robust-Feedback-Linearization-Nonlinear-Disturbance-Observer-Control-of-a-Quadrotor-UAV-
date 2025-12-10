# plot_full_12state.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D)
from sim_full_12state import simulate_full_12state
from params_refs import multi_step_reference, circular_reference


def build_references(t: np.ndarray, ref_type: str = "multi_step"):
    x_ref = np.zeros_like(t)
    y_ref = np.zeros_like(t)
    z_ref = np.zeros_like(t)
    psi_ref = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ref_type == "multi_step":
            x_ref[i], y_ref[i], z_ref[i], psi_ref[i] = multi_step_reference(ti)
        else:  # "circular"
            x_ref[i], y_ref[i], z_ref[i], psi_ref[i] = circular_reference(ti)

    return x_ref, y_ref, z_ref, psi_ref


def plot_full_12state(ref_type: str = "multi_step"):
    # --- run simulation ---
    t, X, U, FXFY, D_true, D_hat = simulate_full_12state(
        T=20.0,
        dt=1e-3,
        ref_type=ref_type,
    )

    # states
    z     = X[:, 0]
    z_dot = X[:, 1]
    phi   = X[:, 2]
    theta = X[:, 4]
    psi   = X[:, 6]
    x_pos = X[:, 8]
    x_dot = X[:, 9]
    y_pos = X[:, 10]
    y_dot = X[:, 11]

    # controls
    Fz       = U[:, 0]
    tau_phi  = U[:, 1]
    tau_theta = U[:, 2]
    tau_psi  = U[:, 3]
    Fx       = FXFY[:, 0]
    Fy       = FXFY[:, 1]

    # references
    x_ref, y_ref, z_ref, psi_ref = build_references(t, ref_type=ref_type)

    # --- 3D trajectory plot ---
    fig3d = plt.figure(figsize=(8, 6))
    ax3d = fig3d.add_subplot(111, projection="3d")
    ax3d.plot(x_pos, y_pos, z, label="trajectory")
    ax3d.plot(x_ref, y_ref, z_ref, "--", label="reference")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title(f"3D Trajectory ({ref_type})")
    ax3d.legend()
    plt.tight_layout()

    # --- position tracking (x,y,z) ---
    fig1, axs1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle("Position Tracking", fontsize=14)

    axs1[0].plot(t, x_pos, label="x")
    axs1[0].plot(t, x_ref, "--", label="x_ref")
    axs1[0].set_ylabel("x (m)")
    axs1[0].grid(True)
    axs1[0].legend()

    axs1[1].plot(t, y_pos, label="y")
    axs1[1].plot(t, y_ref, "--", label="y_ref")
    axs1[1].set_ylabel("y (m)")
    axs1[1].grid(True)
    axs1[1].legend()

    axs1[2].plot(t, z, label="z")
    axs1[2].plot(t, z_ref, "--", label="z_ref")
    axs1[2].set_ylabel("z (m)")
    axs1[2].set_xlabel("Time (s)")
    axs1[2].grid(True)
    axs1[2].legend()

    plt.tight_layout()

    # --- attitude tracking (phi, theta, psi) ---
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle("Attitude Tracking", fontsize=14)

    axs2[0].plot(t, phi, label="phi")
    axs2[0].set_ylabel("phi (rad)")
    axs2[0].grid(True)
    axs2[0].legend()

    axs2[1].plot(t, theta, label="theta")
    axs2[1].set_ylabel("theta (rad)")
    axs2[1].grid(True)
    axs2[1].legend()

    axs2[2].plot(t, psi, label="psi")
    axs2[2].plot(t, psi_ref, "--", label="psi_ref")
    axs2[2].set_ylabel("psi (rad)")
    axs2[2].set_xlabel("Time (s)")
    axs2[2].grid(True)
    axs2[2].legend()

    plt.tight_layout()

    # --- control inputs Fz, torques, Fx, Fy ---
    fig3, axs3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig3.suptitle("Control Inputs", fontsize=14)

    axs3[0].plot(t, Fz, label="Fz")
    axs3[0].set_ylabel("Fz (N)")
    axs3[0].grid(True)
    axs3[0].legend()

    axs3[1].plot(t, tau_phi, label="tau_phi")
    axs3[1].plot(t, tau_theta, label="tau_theta")
    axs3[1].plot(t, tau_psi, label="tau_psi")
    axs3[1].set_ylabel("Torques (Nm)")
    axs3[1].grid(True)
    axs3[1].legend()

    axs3[2].plot(t, Fx, label="Fx")
    axs3[2].plot(t, Fy, label="Fy")
    axs3[2].set_ylabel("Fx, Fy")
    axs3[2].set_xlabel("Time (s)")
    axs3[2].grid(True)
    axs3[2].legend()

    plt.tight_layout()

    # --- disturbances vs estimates (all 6 axes) ---
    fig4, axs4 = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig4.suptitle("Disturbances: True vs Estimated", fontsize=14)

    # z, phi
    axs4[0].plot(t, D_true[:, 0], label="d_z true")
    axs4[0].plot(t, D_hat[:, 0], "--", label="d_z_hat")
    axs4[0].plot(t, D_true[:, 1], label="d_phi true")
    axs4[0].plot(t, D_hat[:, 1], "--", label="d_phi_hat")
    axs4[0].set_ylabel("z, phi")
    axs4[0].grid(True)
    axs4[0].legend()

    # theta, psi
    axs4[1].plot(t, D_true[:, 2], label="d_theta true")
    axs4[1].plot(t, D_hat[:, 2], "--", label="d_theta_hat")
    axs4[1].plot(t, D_true[:, 3], label="d_psi true")
    axs4[1].plot(t, D_hat[:, 3], "--", label="d_psi_hat")
    axs4[1].set_ylabel("theta, psi")
    axs4[1].grid(True)
    axs4[1].legend()

    # x, y
    axs4[2].plot(t, D_true[:, 4], label="d_x true")
    axs4[2].plot(t, D_hat[:, 4], "--", label="d_x_hat")
    axs4[2].plot(t, D_true[:, 5], label="d_y true")
    axs4[2].plot(t, D_hat[:, 5], "--", label="d_y_hat")
    axs4[2].set_ylabel("x, y")
    axs4[2].set_xlabel("Time (s)")
    axs4[2].grid(True)
    axs4[2].legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # choose "multi_step" or "circular"
    plot_full_12state(ref_type="circular")
