
# Robust Feedback Linearization & Nonlinear Harmonic Disturbance Observer Control for a Quadrotor UAV  


---

# Table of Contents
1. [Introduction](#introduction)
2. [Quadrotor Dynamics (Partial Derivation)](#quadrotor-dynamics-partial-derivation)
3. [Controller Architecture](#controller-architecture)
4. [Robust Feedback Linearization (RFBL)](#robust-feedback-linearization-rfbl)
5. [Nonlinear Harmonic Disturbance Observer (NHDO)](#nonlinear-harmonic-disturbance-observer-nhdo)
6. [System Architecture Diagrams](#system-architecture-diagrams)
7. [Python 12-State Simulation](#python-12-state-simulation)
8. [Simulation Results](#simulation-results)
9. [ROS2 + PX4 SITL Implementation](#ros2--px4-sitl-implementation)
10. [Conclusions](#conclusions)
11. [Future Work](#future-work)
12. [License](#license)

---

# Introduction

Quadrotor UAVs are nonlinear, underactuated vehicles whose performance is heavily influenced by external disturbances and modeling uncertainties.  
Traditional controllers (e.g., PID) handle small deviations but do not guarantee robustness under nonlinear coupling and unknown disturbances.

This work implements a **Robust Feedback Linearization-Based Controller (RFBL)** combined with a **Nonlinear Harmonic Disturbance Observer (NHDO)** to achieve:

- Robust trajectory tracking  
- Real-time disturbance rejection  
- Exact cancellation of nonlinear dynamics  
- Smooth control allocation compatible with PX4 flight stack  
- Simulation both in Python and **PX4 SITL (ROS2 Offboard)**  

---

# Quadrotor Dynamics 

A quadrotor in 3D motion is expressed by 12 states:

\[
x = [p_x, p_y, p_z, v_x, v_y, v_z, \phi, 	heta, \psi, p, q, r]^T
\]

## **Rotation Matrix**

\[
R = R_z(\psi) R_y(	heta) R_x(\phi)
\]

\[
R = 
egin{bmatrix}
c	heta c\psi & c	heta s\psi & -s	heta \
s\phi s	heta c\psi - c\phi s\psi & s\phi s	heta s\psi + c\phi c\psi & s\phi c	heta \
c\phi s	heta c\psi + s\phi s\psi & c\phi s	heta s\psi - s\phi c\psi & c\phi c	heta
\end{bmatrix}
\]

## **Translational Dynamics**

\[
m\ddot{p} = Regin{bmatrix}0\0\F\end{bmatrix} + 
egin{bmatrix}0 \ 0 \ -mg \end{bmatrix} + d_t
\]

Expanded:

\[
\ddot{x} = rac{F}{m}(c\phi s	heta c\psi + s\phi s\psi) + d_x
\]

\[
\ddot{y} = rac{F}{m}(c\phi s	heta s\psi - s\phi c\psi) + d_y
\]

\[
\ddot{z} = -g + rac{F}{m}c\phi c	heta + d_z
\]

## **Rotational Dynamics**

\[
I\dot{\omega} + \omega 	imes (I\omega) = 	au + d_r
\]

Kinematic relation:

\[
\dot{\eta} = E(\phi, 	heta)\omega
\]

Where:

- \( 	au = [	au_\phi,	au_	heta,	au_\psi]^T \)  
- \( d_r = [d_\phi,d_	heta,d_\psi]^T \)

---

# 🧠 Controller Architecture

```
          +--------------------------+
          |   Reference Trajectory   |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          |  RFBL Controller         |
          |  (Dynamic Inversion)     |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          | NHDO Disturbance Obs.    |
          +------------+-------------+
                       |
                       v
          +--------------------------+
          | PX4 Offboard Commands    |
          +--------------------------+
```

The RFBL ensures nonlinearities are canceled, and NHDO handles unknown disturbances.

---

# Robust Feedback Linearization (RFBL)

The control objective is:

\[
\ddot{e} + k_d\dot{e} + k_p e = 0
\]

Where \( e = y_{ref}-y \).

---

## **Altitude Control**

Dynamics:

\[
\ddot{z} = -g + rac{F}{m}c\phi c	heta + d_z
\]

Define linearizing input:

\[
u_z = \ddot{z}_{ref} + k_{dz}(\dot{z}_{ref}-\dot{z}) + k_{pz}(z_{ref}-z)
\]

Solve for thrust:

\[
F = rac{m(u_z + g - \hat{d}_z)}{c\phi c	heta}
\]

---

## **Lateral Control (X/Y)**

Desired accelerations:

\[
u_x = \ddot{x}_{ref} + k_{dx}\dot{e}_x + k_{px}e_x
\]
\[
u_y = \ddot{y}_{ref} + k_{dy}\dot{e}_y + k_{py}e_y
\]

Inversion yields desired pitch & roll:

\[
	heta_{des} = rac{mu_x}{F}
\]
\[
\phi_{des} = -rac{mu_y}{F}
\]

---

## **Attitude Control**

Euler-angle PD tracking:

\[
	au_\phi = k_{p\phi}(\phi_{des}-\phi) + k_{d\phi}(p_{des}-p)
\]

Similar equations for \( 	heta, \psi \).

---

# Nonlinear Harmonic Disturbance Observer (NHDO)

Unknown disturbances are represented as:

\[
d(t) = \sum_{k=1}^{N} a_k \sin(\omega_k t) + b_k \cos(\omega_k t)
\]

The observer estimates:

- Translational disturbances \( d_x, d_y, d_z \)  
- Rotational disturbances \( d_\phi, d_	heta, d_\psi \)

The estimates \( \hat{d} \) are fed back to RFBL:

\[
u = u_{RFBL} - \hat{d}
\]

Thus achieving **robustness and disturbance rejection**.

---

# System Architecture Diagrams

## **High-Level Control Flow**

```
+------------------+      +----------------------+      +---------------+
| Trajectory Gen.  | ---> | RFBL Linearization   | ---> | NHDO Observer |
+------------------+      +----------------------+      +---------------+
                                    |
                                    v
                           +----------------------+
                           | PX4 Offboard Control |
                           +----------------------+
                                    |
                                    v
                               +---------+
                               |  UAV    |
                               +---------+
```

---

# Python 12-State Simulation

The Python simulation validates:

- Controller correctness  
- NHDO convergence  
- Trajectory tracking  
- Smooth control effort  

Multiple trajectories were tested, including:

- Circular  
- Hover  
- Step response  

---

# Simulation Results

## **3D Trajectory Tracking**
![3D Trajectory](f_results/Figure_1_3D_Trajectory.png)

## **Position Tracking**
![Position Tracking](f_results/Figure_2_Position_Tracking.png)

## **Attitude Tracking**
![Attitude Tracking](f_results/Figure_3_Attitude_Tracking.png)

## **Control Inputs**
![Control Inputs](f_results/Figure_4_Control_Inputs.png)

## **Disturbance Estimation**
![Disturbance](f_results/Figure_5_Disturbance.png)

---

# ROS2 + PX4 SITL Implementation

The ROS2 Offboard node:

- Subscribes to PX4 odometry  
- Publishes:
  - Thrust  
  - Attitude setpoints  
- Computes RFBL + NHDO in real time  
- Logs simulation data  

Run:

```bash
ros2 launch rfbl_controller rfbl_offboard.launch.py
```

---

# Conclusions

This study demonstrates:

- Successful RFBL implementation  
- Accurate disturbance rejection via NHDO  
- Excellent trajectory tracking  
- Smooth, stable control inputs  
- Consistency between Python and ROS2 simulations  

---

# Future Work

- Real UAV flight tests  
- Adaptive disturbance frequency estimation  
- Wind tunnel validation  
- Formation control extensions  

---

# License

MIT License.

# References
1. Kamal, A. Arif, and M. Rafique, “Robust Feedback Linearization Based Disturbance Observer Control of Quadrotor UAV,” International Journal of Control, Automation and Systems, 2020
2. Quadrotor Dynamics Foundations
R. M. Murray, Z. Li, and S. S. Sastry, A Mathematical Introduction to Robotic Manipulation, CRC Press, 1994.
3. Nonlinear Systems & Feedback Linearization
A. Isidori, Nonlinear Control Systems, Springer, 1995.
4. Disturbance Observer Theory
K. Ohnishi, “A new servo method in mechatronics,” Trans. Japanese Soc. Electrical Engineers, vol. 107-D, pp. 83–86, 1987.
5. Quadrotor Modeling & Classical Control Reference
T. Lee, M. Leok, and N. H. McClamroch, “Geometric tracking control of a quadrotor UAV on SE(3),” Proceedings of the IEEE Conference on Decision and Control (CDC), 2010.
6. PX4 Documentation
PX4 Autopilot User Guide, https://docs.px4.io/
7. ROS2 Documentation
ROS 2 Documentation, https://docs.ros.org/
8. MAVLink / Offboard Control Reference
MAVLink Developer Guide, https://mavlink.io/en/
