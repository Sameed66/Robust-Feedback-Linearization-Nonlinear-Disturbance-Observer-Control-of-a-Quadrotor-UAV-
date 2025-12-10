# hardware_loop_example.py

import time
import numpy as np

from params_and_refs import QuadrotorParams, circular_reference
from rfbl_controller import RFBLAltitudeAttitudeController
from disturbance_observer import HarmonicDisturbanceObserver
from xy_position_controller import XYPositionRFBLSTC
from xy_disturbance_observer import XYHarmonicDisturbanceObserver
from motor_mixer import motor_mixing

def get_state_from_sensors():
    """
    Replace this with your actual sensor fusion:
    return np.array([z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot,
                     x, x_dot, y, y_dot])
    """
    raise NotImplementedError

def send_motor_speeds(omega):
    """
    Replace with your hardware interface (PWM, ESC, etc.)
    omega: np.array([ω1, ω2, ω3, ω4])
    """
    raise NotImplementedError

def main_control_loop(dt=0.005):
    p = QuadrotorParams()

    inner_ctrl = RFBLAltitudeAttitudeController(p)
    outer_ctrl = XYPositionRFBLSTC(p)
    att_observer = HarmonicDisturbanceObserver(p)
    xy_observer = XYHarmonicDisturbanceObserver(p)

    u_prev = np.array([p.m * p.g, 0.0, 0.0, 0.0])  # hover guess
    Fx_prev = 0.0
    Fy_prev = 0.0

    t0 = time.time()

    while True:
        t_now = time.time() - t0

        # 1) Read state from sensors
        x = get_state_from_sensors()
        x_8 = x[0:8]
        x_xy = x[8:12]

        # 2) References (for example: circular)
        x_d, y_d, z_d, psi_d = circular_reference(t_now)
        x_dot_d  = -0.5 * np.sin(0.5 * t_now)
        x_ddot_d = -0.25 * np.cos(0.5 * t_now)
        y_dot_d  =  0.5 * np.cos(0.5 * t_now)
        y_ddot_d = -0.25 * np.sin(0.5 * t_now)

        z_dot_d = 0.0
        z_ddot_d = 0.0
        psi_dot_d = 0.0
        psi_ddot_d = 0.0

        # 3) Disturbance observers
        d_hat_att = att_observer.step(x_8, tuple(u_prev), dt)
        d_hat_xy = xy_observer.step(x_xy, Fz=u_prev[0], Fx=Fx_prev, Fy=Fy_prev, dt=dt)

        # 4) Outer loop XY
        Fx, Fy, phi_des, theta_des = outer_ctrl.step(
            x9_12=x_xy,
            x_ref=x_d,
            x_ref_dot=x_dot_d,
            x_ref_ddot=x_ddot_d,
            y_ref=y_d,
            y_ref_dot=y_dot_d,
            y_ref_ddot=y_ddot_d,
            Fz=u_prev[0],
            d_hat_xy=d_hat_xy,
            psi_d=psi_d,
            dt=dt,
        )

        # 5) Inner loop z,phi,theta,psi
        phi_dot_d = 0.0
        theta_dot_d = 0.0

        xd = np.array([
            z_d,      z_dot_d,
            phi_des,  phi_dot_d,
            theta_des,theta_dot_d,
            psi_d,    psi_dot_d
        ])

        xd_dot = np.zeros(8)
        xd_dot[1] = z_ddot_d
        xd_dot[7] = psi_ddot_d

        Fz, tau_phi, tau_theta, tau_psi = inner_ctrl.step(
            x=x_8,
            xd=xd,
            xd_dot=xd_dot,
            d_hat=d_hat_att,
            dt=dt,
        )

        u = np.array([Fz, tau_phi, tau_theta, tau_psi])

        # 6) Motor mixing
        omega = motor_mixing(Fz, tau_phi, tau_theta, tau_psi, p)

        # 7) Send to motors
        send_motor_speeds(omega)

        # 8) Store for next iteration
        u_prev = u
        Fx_prev = Fx
        Fy_prev = Fy

        # 9) Wait until next cycle
        time.sleep(dt)
