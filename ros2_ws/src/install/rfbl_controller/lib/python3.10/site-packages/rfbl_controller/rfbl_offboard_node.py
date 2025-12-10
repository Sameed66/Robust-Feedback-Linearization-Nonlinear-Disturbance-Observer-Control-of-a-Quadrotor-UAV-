# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node

# from mavros_msgs.msg import State, AttitudeTarget
# from geometry_msgs.msg import PoseStamped, TwistStamped
# from tf_transformations import euler_from_quaternion, quaternion_from_euler


# class RfblOffboardNode(Node):
#     def __init__(self):
#         super().__init__("rfbl_offboard_node")

#         # --- Subscribers ---
#         self.state_sub = self.create_subscription(
#             State,
#             "/mavros/state",
#             self.state_callback,
#             10,
#         )

#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             "/mavros/local_position/pose",
#             self.pose_callback,
#             10,
#         )

#         self.vel_sub = self.create_subscription(
#             TwistStamped,
#             "/mavros/local_position/velocity_local",
#             self.vel_callback,
#             10,
#         )

#         # --- Publisher: Attitude target ---
#         self.att_sp_pub = self.create_publisher(
#             AttitudeTarget,
#             "/mavros/setpoint_raw/attitude",
#             10,
#         )

#         # internal state storage
#         self.current_state = None
#         self.current_pose = None
#         self.current_vel = None

#         # control loop timer (e.g. 50 Hz)
#         self.control_rate = 50.0  # Hz
#         self.dt = 1.0 / self.control_rate
#         self.timer = self.create_timer(self.dt, self.control_loop)

#         self.get_logger().info("RFBL Offboard node initialized")

#         # TODO: here we will later initialize your RFBL controller objects
#         # self.params = QuadrotorParams()
#         # self.inner_ctrl = RFBLAltitudeAttitudeController(self.params)
#         # self.outer_ctrl = XYPositionRFBLSTC(self.params)
#         # self.do_altatt = HarmonicDisturbanceObserver(self.params)
#         # self.do_xy = HarmonicDisturbanceObserverXY(self.params)

#     # --------------------
#     # Callbacks
#     # --------------------
#     def state_callback(self, msg: State):
#         self.current_state = msg

#     def pose_callback(self, msg: PoseStamped):
#         self.current_pose = msg

#     def vel_callback(self, msg: TwistStamped):
#         self.current_vel = msg

#     # --------------------
#     # Main control loop
#     # --------------------
#     def control_loop(self):
#         if self.current_state is None or self.current_pose is None or self.current_vel is None:
#             # Not enough data yet
#             return

#         # Here you will:
#         # 1) extract x, y, z, yaw, velocities from current_pose/current_vel
#         # 2) build your state vector X (12-state)
#         # 3) run RFBL + observers to compute desired (phi_d, theta_d, psi_d, Fz)

#         # For Step 3, we just send a simple "hover" command:
#         #   - level attitude (0,0,yaw)
#         #   - constant thrust ~ hover

#         # Example: get current yaw from pose
#         q = self.current_pose.pose.orientation
#         quat = [q.x, q.y, q.z, q.w]
#         _, _, yaw = euler_from_quaternion(quat)

#         # Desired attitude: keep yaw, zero roll/pitch
#         phi_des = 0.0
#         theta_des = 0.0
#         psi_des = yaw  # hold current yaw

#         # Convert desired euler -> quaternion
#         qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_des)

#         # Rough hover thrust guess: 0.5 (we'll tune later using params.m * g)
#         thrust_norm = 0.5

#         att_msg = AttitudeTarget()
#         att_msg.header.stamp = self.get_clock().now().to_msg()
#         att_msg.type_mask = 0  # use all fields

#         att_msg.orientation.x = qx
#         att_msg.orientation.y = qy
#         att_msg.orientation.z = qz
#         att_msg.orientation.w = qw

#         # We are not using body rates in this simple example
#         att_msg.body_rate.x = 0.0
#         att_msg.body_rate.y = 0.0
#         att_msg.body_rate.z = 0.0

#         att_msg.thrust = float(thrust_norm)

#         self.att_sp_pub.publish(att_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RfblOffboardNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

############################# Running coder - hover ########################################################################## 

# import math
# import numpy as np

# import rclpy
# from rclpy.node import Node

# from mavros_msgs.msg import State, AttitudeTarget
# from geometry_msgs.msg import PoseStamped, TwistStamped
# from tf_transformations import euler_from_quaternion, quaternion_from_euler

# # import your control modules
# from .params_refs import QuadrotorParams
# from .rfbl_controller import RFBLAltitudeAttitudeController
# from .xy_position_controller import XYPositionRFBLSTC
# # observers can be added later if desired
# # from disturbance_observer import HarmonicDisturbanceObserver
# # from xy_disturbance_observer import HarmonicDisturbanceObserverXY


# class RfblOffboardNode(Node):
#     def __init__(self):
#         super().__init__("rfbl_offboard_node")

#         # --------- Parameters & Controllers ----------
#         self.params = QuadrotorParams()

#         # inner loop: z, phi, theta, psi
#         self.inner_ctrl = RFBLAltitudeAttitudeController(self.params)
#         # outer loop: x, y -> Fx, Fy, phi_d, theta_d
#         self.outer_ctrl = XYPositionRFBLSTC(self.params)

#         # disturbance observers (optional – not used yet in live ROS)
#         # self.do_altatt = HarmonicDisturbanceObserver(self.params)
#         # self.do_xy = HarmonicDisturbanceObserverXY(self.params)

#         # last thrust for outer loop (start at hover)
#         self.last_Fz = self.params.m * self.params.g

#         # reference storage (initialized when first pose arrives)
#         self.ref_initialized = False
#         self.x0_ref = 0.0
#         self.y0_ref = 0.0
#         self.z0_ref = 0.0
#         self.psi0_ref = 0.0

#         # --------- Subscribers ----------
#         self.state_sub = self.create_subscription(
#             State,
#             "/mavros/state",
#             self.state_callback,
#             10,
#         )

#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             "/mavros/local_position/pose",
#             self.pose_callback,
#             10,
#         )

#         self.vel_sub = self.create_subscription(
#             TwistStamped,
#             "/mavros/local_position/velocity_local",
#             self.vel_callback,
#             10,
#         )

#         # --------- Publisher: Attitude target ----------
#         self.att_sp_pub = self.create_publisher(
#             AttitudeTarget,
#             "/mavros/setpoint_raw/attitude",
#             10,
#         )

#         # internal state storage
#         self.current_state: State | None = None
#         self.current_pose: PoseStamped | None = None
#         self.current_vel: TwistStamped | None = None

#         # control loop timer (e.g. 50 Hz)
#         self.control_rate = 50.0  # Hz
#         self.dt = 1.0 / self.control_rate
#         self.timer = self.create_timer(self.dt, self.control_loop)

#         self.get_logger().info("RFBL Offboard node initialized (with controllers).")

#     # --------------------
#     # Callbacks
#     # --------------------
#     def state_callback(self, msg: State):
#         self.current_state = msg

#     def pose_callback(self, msg: PoseStamped):
#         self.current_pose = msg

#     def vel_callback(self, msg: TwistStamped):
#         self.current_vel = msg

    
#     def control_loop(self):
#         # --- ALWAYS publish a simple hover command (for debugging) ---

#         yaw = 0.0  # default

#         if self.current_pose is not None:
#             # If we have pose, keep current yaw
#             ori = self.current_pose.pose.orientation
#             quat = [ori.x, ori.y, ori.z, ori.w]
#             try:
#                 _, _, yaw = euler_from_quaternion(quat)
#             except Exception:
#                 yaw = 0.0

#         # Desired attitude: zero roll/pitch, current (or zero) yaw
#         phi_des = 0.0
#         theta_des = 0.0
#         psi_des = yaw

#         qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_des)

#         # Fixed hover-ish thrust
#         thrust_norm = 0.75  # in [0, 1]

#         att_msg = AttitudeTarget()
#         att_msg.header.stamp = self.get_clock().now().to_msg()
#         att_msg.type_mask = 0  # use all fields

#         att_msg.orientation.x = qx
#         att_msg.orientation.y = qy
#         att_msg.orientation.z = qz
#         att_msg.orientation.w = qw

#         att_msg.body_rate.x = 0.0
#         att_msg.body_rate.y = 0.0
#         att_msg.body_rate.z = 0.0

#         att_msg.thrust = float(thrust_norm)

#         self.att_sp_pub.publish(att_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RfblOffboardNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#####################################################################################################################

########################## runnning before xy ###################################################################

#!/usr/bin/env python3

# import numpy as np

# import rclpy
# from rclpy.node import Node

# from mavros_msgs.msg import State, AttitudeTarget
# from geometry_msgs.msg import PoseStamped, TwistStamped
# from tf_transformations import euler_from_quaternion, quaternion_from_euler

# from .params_refs import QuadrotorParams
# from .rfbl_controller import RFBLAltitudeAttitudeController
# # (we will add XY controller later)
# # from .xy_position_controller import XYPositionRFBLSTC


# class RfblOffboardNode(Node):
#     def __init__(self):
#         super().__init__("rfbl_offboard_node")

#         # --------- Parameters & Controllers ----------
#         self.params = QuadrotorParams()

#         # inner loop: z, phi, theta, psi
#         self.inner_ctrl = RFBLAltitudeAttitudeController(self.params)

#         # Approximate hover thrust in PX4 normalized units (from your tests)
#         self.hover_thrust_norm = 0.75
#         self.hover_Fz = self.params.m * self.params.g
#         self.last_Fz = self.hover_Fz

#         # reference storage
#         self.ref_initialized = False
#         self.z0_ref = 0.0
#         self.psi0_ref = 0.0

#         # --------- Subscribers ----------
#         self.state_sub = self.create_subscription(
#             State,
#             "/mavros/state",
#             self.state_callback,
#             10,
#         )

#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             "/mavros/local_position/pose",
#             self.pose_callback,
#             10,
#         )

#         self.vel_sub = self.create_subscription(
#             TwistStamped,
#             "/mavros/local_position/velocity_local",
#             self.vel_callback,
#             10,
#         )

#         # --------- Publisher: Attitude target ----------
#         self.att_sp_pub = self.create_publisher(
#             AttitudeTarget,
#             "/mavros/setpoint_raw/attitude",
#             10,
#         )

#         # internal state storage
#         self.current_state = None
#         self.current_pose = None
#         self.current_vel = None

#         # control loop timer (e.g. 50 Hz)
#         self.control_rate = 50.0  # Hz
#         self.dt = 1.0 / self.control_rate
#         self.timer = self.create_timer(self.dt, self.control_loop)

#         self.get_logger().info("RFBL Offboard node initialized (altitude+attitude RFBL).")

#     # --------------------
#     # Callbacks
#     # --------------------
#     def state_callback(self, msg: State):
#         self.current_state = msg

#     def pose_callback(self, msg: PoseStamped):
#         self.current_pose = msg

#     def vel_callback(self, msg: TwistStamped):
#         self.current_vel = msg

#     # --------------------
#     # Main control loop
#     # --------------------
#     def control_loop(self):
#         # If pose or velocity is missing, fall back to simple hover command
#         if self.current_pose is None or self.current_vel is None:
#             self.simple_hover_fallback()
#             return

#         # 1) Extract pose & velocity (MAVROS uses ENU: x=East, y=North, z=Up)
#         pos = self.current_pose.pose.position
#         ori = self.current_pose.pose.orientation
#         vel_lin = self.current_vel.twist.linear
#         vel_ang = self.current_vel.twist.angular  # roll/pitch/yaw rates

#         # orientation -> roll, pitch, yaw
#         quat = [ori.x, ori.y, ori.z, ori.w]
#         roll, pitch, yaw = euler_from_quaternion(quat)

#         # positions and velocities
#         x_pos = pos.x
#         y_pos = pos.y
#         z_pos = pos.z

#         x_dot = vel_lin.x
#         y_dot = vel_lin.y
#         z_dot = vel_lin.z

#         phi = roll
#         theta = pitch
#         psi = yaw

#         phi_dot = vel_ang.x
#         theta_dot = vel_ang.y
#         psi_dot = vel_ang.z

#         # 2) Initialize altitude/yaw reference
#         if not self.ref_initialized:
#             self.z0_ref = z_pos
#             self.psi0_ref = psi
#             self.ref_initialized = True
#             self.get_logger().info(
#                 f"Reference initialized at z={self.z0_ref:.2f}, yaw={self.psi0_ref:.2f}"
#             )

#         # Altitude target: climb 1 m above initial
#         z_ref = self.z0_ref + 1.0
#         z_dot_ref = 0.0
#         z_ddot_ref = 0.0

#         # Attitude target: keep roll/pitch = 0, yaw = initial yaw
#         phi_des = 0.0
#         theta_des = 0.0
#         psi_ref = self.psi0_ref
#         psi_dot_ref = 0.0
#         psi_ddot_ref = 0.0

#         # 3) Build inner state vector [z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]
#         x_inner = np.array([
#             z_pos,
#             z_dot,
#             phi,
#             phi_dot,
#             theta,
#             theta_dot,
#             psi,
#             psi_dot
#         ], dtype=float)

#         # Desired inner state
#         xd_inner = np.array([
#             z_ref,
#             z_dot_ref,
#             phi_des,
#             0.0,        # phi_dot_des
#             theta_des,
#             0.0,        # theta_dot_des
#             psi_ref,
#             psi_dot_ref
#         ], dtype=float)

#         xd_dot_inner = np.zeros(8)
#         xd_dot_inner[1] = z_ddot_ref
#         xd_dot_inner[7] = psi_ddot_ref

#         # Disturbance estimates (not used yet – set to zero)
#         d_hat_altatt = np.zeros(4)

#         # 4) Call RFBL inner controller
#         Fz, tau_phi, tau_theta, tau_psi = self.inner_ctrl.step(
#             x=x_inner,
#             xd=xd_inner,
#             xd_dot=xd_dot_inner,
#             d_hat=d_hat_altatt,
#             dt=self.dt,
#         )

#         self.last_Fz = Fz

#         # 5) Map Fz (Newtons) -> PX4 thrust [0, 1]
#         #    At hover: Fz ≈ m*g -> thrust_norm ≈ self.hover_thrust_norm
#         thrust_ratio = Fz / self.hover_Fz  # 1.0 at ideal hover
#         thrust_norm = self.hover_thrust_norm * thrust_ratio
#         thrust_norm = float(np.clip(thrust_norm, 0.0, 1.0))

#         # 6) Build AttitudeTarget message
#         qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_ref)

#         att_msg = AttitudeTarget()
#         att_msg.header.stamp = self.get_clock().now().to_msg()
#         att_msg.type_mask = 0  # use all fields

#         att_msg.orientation.x = qx
#         att_msg.orientation.y = qy
#         att_msg.orientation.z = qz
#         att_msg.orientation.w = qw

#         # For now we don't command body rates, only attitude + thrust
#         att_msg.body_rate.x = 0.0
#         att_msg.body_rate.y = 0.0
#         att_msg.body_rate.z = 0.0

#         att_msg.thrust = thrust_norm

#         self.att_sp_pub.publish(att_msg)

#     # --------------------
#     # Fallback simple hover if no pose/vel
#     # --------------------
#     def simple_hover_fallback(self):
#         yaw = 0.0
#         if self.current_pose is not None:
#             ori = self.current_pose.pose.orientation
#             quat = [ori.x, ori.y, ori.z, ori.w]
#             try:
#                 _, _, yaw = euler_from_quaternion(quat)
#             except Exception:
#                 yaw = 0.0

#         phi_des = 0.0
#         theta_des = 0.0
#         psi_des = yaw

#         qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_des)

#         att_msg = AttitudeTarget()
#         att_msg.header.stamp = self.get_clock().now().to_msg()
#         att_msg.type_mask = 0

#         att_msg.orientation.x = qx
#         att_msg.orientation.y = qy
#         att_msg.orientation.z = qz
#         att_msg.orientation.w = qw

#         att_msg.body_rate.x = 0.0
#         att_msg.body_rate.y = 0.0
#         att_msg.body_rate.z = 0.0

#         att_msg.thrust = float(self.hover_thrust_norm)
#         self.att_sp_pub.publish(att_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = RfblOffboardNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from .params_refs import QuadrotorParams
from .rfbl_controller import RFBLAltitudeAttitudeController
from .xy_position_controller import XYPositionRFBLSTC


class RfblOffboardNode(Node):
    def __init__(self):
        super().__init__("rfbl_offboard_node")

        self.xy_active = False
        self.traj_t0 = None
        self.z_cmd = None

        # --------- Parameters & Controllers ----------
        self.params = QuadrotorParams()

        # inner loop: z, phi, theta, psi
        self.inner_ctrl = RFBLAltitudeAttitudeController(self.params)
        # outer loop: x, y -> Fx, Fy, phi_des, theta_des
        self.outer_ctrl = XYPositionRFBLSTC(self.params)

        # Approximate hover thrust in PX4 normalized units (from your tests) --------------------------------------
        self.hover_thrust_norm = 0.70
        self.hover_Fz = self.params.m * self.params.g
        self.last_Fz = self.hover_Fz

        # reference storage
        self.ref_initialized = False
        self.x0_ref = 0.0
        self.y0_ref = 0.0
        self.z0_ref = 0.0
        self.psi0_ref = 0.0

        # trajectory selection: "hover", "step_xy", "circle" ------------------------------------
        self.traj_mode = "circle"  # change to "circle" or "step_xy" later
        self.start_time = None


        # --------- Subscribers ----------
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_profile_sensor_data,   # ⬅️ changed
        )

        self.vel_sub = self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.vel_callback,
            qos_profile_sensor_data,   # ⬅️ changed
        )

        

        # --------- Publisher: Attitude target ----------
        self.att_sp_pub = self.create_publisher(
            AttitudeTarget,
            "/mavros/setpoint_raw/attitude",
            10,
        )

        # internal state storage
        self.current_state = None
        self.current_pose = None
        self.current_vel = None

        # control loop timer (e.g. 50 Hz)
        self.control_rate = 50.0  # Hz
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("RFBL Offboard node initialized (full inner + XY outer RFBL).")

        self.debug_counter = 0

        # ---------------------------
        # Data logging buffers
        # ---------------------------
        self.log_t = []
        self.log_x = []
        self.log_y = []
        self.log_z = []
        self.log_xref = []
        self.log_yref = []
        self.log_zref = []
        self.log_phi = []
        self.log_theta = []
        self.log_thrust = []

    # --------------------
    # Callbacks
    # --------------------
    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def vel_callback(self, msg: TwistStamped):
        self.current_vel = msg

    # --------------------
    # Main control loop
    # --------------------

    def control_loop(self):
        # If pose or velocity is missing, fall back to simple hover
        if self.current_pose is None or self.current_vel is None:
            self.simple_hover_fallback()
            return

        # 1) Extract pose & velocity (MAVROS uses ENU: x=East, y=North, z=Up)
        pos = self.current_pose.pose.position
        ori = self.current_pose.pose.orientation
        vel_lin = self.current_vel.twist.linear
        vel_ang = self.current_vel.twist.angular  # roll/pitch/yaw rates

        # orientation -> roll, pitch, yaw
        quat = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        x_pos = pos.x
        y_pos = pos.y
        z_pos = pos.z

        x_dot = vel_lin.x
        y_dot = vel_lin.y
        z_dot = vel_lin.z

        phi = roll
        theta = pitch
        psi = yaw

        phi_dot = vel_ang.x
        theta_dot = vel_ang.y
        psi_dot = vel_ang.z

        # 2) Initialize references on first valid data
        if not self.ref_initialized:
            self.x0_ref = x_pos
            self.y0_ref = y_pos
            self.z0_ref = z_pos
            self.psi0_ref = psi
            self.ref_initialized = True
            # record start time
            self.start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f"Ref init at x={self.x0_ref:.2f}, y={self.y0_ref:.2f}, "
                f"z={self.z0_ref:.2f}, yaw={self.psi0_ref:.2f}"
            )

        # Time since reference init
        if self.start_time is None:
            t = 0.0
        else:
            now = self.get_clock().now().nanoseconds * 1e-9
            t = now - self.start_time

        # -------- Trajectory definitions --------
        psi_ref = self.psi0_ref

        offboard_armed = (
            self.current_state is not None
            and self.current_state.mode == "OFFBOARD"
            and self.current_state.armed
        )

        # --- Smooth altitude command (ramp to z0+1.0 m) ---
        if not offboard_armed:
            # Before OFFBOARD+ARM: hold current altitude, reset ramp
            z_ref = z_pos
            self.z_cmd = z_pos
        else:
            if self.z_cmd is None:
                self.z_cmd = z_pos  # initialize ramp

            climb_target = self.z0_ref + 1.0    # desired hover altitude
            ramp_rate = 0.3                     # [m/s] approximate climb rate

            delta = climb_target - self.z_cmd
            max_step = ramp_rate * self.dt      # max change per control step

            # clamp ramp step
            step = np.clip(delta, -max_step, max_step)
            self.z_cmd += step

            z_ref = self.z_cmd

        # defaults
        x_ref = self.x0_ref
        y_ref = self.y0_ref

        # Enable XY control only once we are in OFFBOARD, armed, and above some height
        # ----- XY activation logic with latch -----
        if not offboard_armed:
            # if we are not in OFFBOARD+armed, never do XY
            self.xy_active = False
            self.traj_t0 = None
        else:
            # condition to turn XY ON (once)
            # condition to turn XY ON (once)
            if (not self.xy_active) \
            and (z_pos > self.z0_ref + 0.7) \
            and (abs(z_pos - z_ref) < 0.10):
                self.xy_active = True
                self.traj_t0 = t
                self.get_logger().info("XY control ACTIVATED")

            # optional safety: turn XY OFF if altitude goes way off
            if self.xy_active and abs(z_pos - z_ref) > 0.7:
                self.xy_active = False
                self.traj_t0 = None
                self.get_logger().warn("XY control DEACTIVATED due to large altitude error")

        xy_enabled = self.xy_active

        # default derivatives
        x_dot_ref = 0.0
        y_dot_ref = 0.0
        x_ddot_ref = 0.0
        y_ddot_ref = 0.0

        z_dot_ref = 0.0
        z_ddot_ref = 0.0
        psi_dot_ref = 0.0
        psi_ddot_ref = 0.0

        # Time-varying XY reference (only used when XY is enabled and not pure hover)
        # Time-varying XY reference (only used when XY is enabled and not pure hover)
        if xy_enabled and self.traj_mode != "hover":

            # local trajectory time since XY turned on
            if self.traj_t0 is None:
                tau = 0.0
            else:
                tau = t - self.traj_t0

            if self.traj_mode == "step_xy":
                if tau <= 10.0:
                    x_ref = self.x0_ref
                    y_ref = self.y0_ref
                elif tau <= 25.0:
                    x_ref = self.x0_ref + 0.5
                    y_ref = self.y0_ref + 0.5
                else:
                    x_ref = self.x0_ref
                    y_ref = self.y0_ref

            elif self.traj_mode == "circle": #--------------------------------------------------------
                R = 0.5
                w = 0.12
                x_ref = self.x0_ref + R * np.cos(w * tau)
                y_ref = self.y0_ref + R * np.sin(w * tau)

            # derivatives (for both step_xy and circle if you like)
            if self.traj_mode == "circle":
                x_dot_ref = -R * w * np.sin(w * tau)
                y_dot_ref =  R * w * np.cos(w * tau)
                x_ddot_ref = -R * (w**2) * np.cos(w * tau)
                y_ddot_ref = -R * (w**2) * np.sin(w * tau)
            else:
                x_dot_ref = 0.0
                y_dot_ref = 0.0
                x_ddot_ref = 0.0
                y_ddot_ref = 0.0

         #____________________________________________________________________________________________________________   

        # # -------- Trajectory definitions --------
        # # Default altitude and yaw
        # # Default yaw
        # psi_ref = self.psi0_ref

        # offboard_armed = (
        #     self.current_state is not None
        #     and self.current_state.mode == "OFFBOARD"
        #     and self.current_state.armed
        # )

        # if not offboard_armed:
        #     # Before OFFBOARD+ARM: DO NOT climb yet, just hold current altitude
        #     z_ref = z_pos
        # else:
        #     # Once we're in OFFBOARD+ARM, aim for z0+1m
        #     z_ref = self.z0_ref + 1.0

        # # Default: hover at (x0, y0)
        # x_ref = self.x0_ref
        # y_ref = self.y0_ref

        # # Time-varying XY reference (only used in non-hover modes)
        # if self.traj_mode == "step_xy":
        #     if t <= 5.0:
        #         x_ref = self.x0_ref
        #         y_ref = self.y0_ref
        #     elif t <= 10.0:
        #         x_ref = self.x0_ref + 1.0
        #         y_ref = self.y0_ref + 1.0
        #     elif t <= 15.0:
        #         x_ref = self.x0_ref - 1.0
        #         y_ref = self.y0_ref
        #     else:
        #         x_ref = self.x0_ref + 2.0
        #         y_ref = self.y0_ref - 1.0

        # elif self.traj_mode == "circle": #-------------------------------------------------------------------------------------------------------------------------
        #     R = 0.8       # circle radius [m]
        #     w = 0.12       # angular speed [rad/s]
        #     x_ref = self.x0_ref + R * np.cos(w * t)
        #     y_ref = self.y0_ref + R * np.sin(w * t)

        # # desired derivatives (zero for now)
        # x_dot_ref = 0.0
        # y_dot_ref = 0.0
        # x_ddot_ref = 0.0
        # y_ddot_ref = 0.0
        # z_dot_ref = 0.0
        # z_ddot_ref = 0.0
        # psi_dot_ref = 0.0
        # psi_ddot_ref = 0.0

        # 3) Compute desired attitude
        if (not xy_enabled) or (self.traj_mode == "hover"):
            # --- SAFE MODE: no XY control, just level attitude ---
            phi_des = 0.0
            theta_des = 0.0

        else:
            # --- Use outer RFBL controller for XY ---
            x9_12 = np.array([x_pos, x_dot, y_pos, y_dot], dtype=float)
            d_hat_xy = np.zeros(2)

            Fx, Fy, phi_des, theta_des = self.outer_ctrl.step(
                x9_12=x9_12,
                x_ref=x_ref,
                x_ref_dot=x_dot_ref,
                x_ref_ddot=x_ddot_ref,
                y_ref=y_ref,
                y_ref_dot=y_dot_ref,
                y_ref_ddot=y_ddot_ref,
                Fz=self.last_Fz,
                d_hat_xy=d_hat_xy,
                psi_d=psi_ref,
                dt=self.dt,
            )

            # Even softer XY action -------------------------------------------------------------------
            xy_scale = 0.12   # was 0.2

            phi_des *= xy_scale
            theta_des *= xy_scale

            max_tilt = np.deg2rad(5.0)  # was 10 deg
            phi_des = float(np.clip(phi_des, -max_tilt, max_tilt))
            theta_des = float(np.clip(theta_des, -max_tilt, max_tilt))
        # 4) RFBL INNER LOOP FOR ALTITUDE (Fz only, ignore torques)
        x_inner = np.array([
            z_pos,
            z_dot,
            phi,
            phi_dot,
            theta,
            theta_dot,
            psi,
            psi_dot
        ], dtype=float)

        xd_inner = np.array([
            z_ref,
            z_dot_ref,
            phi_des,
            0.0,        # phi_dot_des
            theta_des,
            0.0,        # theta_dot_des
            psi_ref,
            psi_dot_ref
        ], dtype=float)

        xd_dot_inner = np.zeros(8)
        xd_dot_inner[1] = z_ddot_ref
        xd_dot_inner[7] = psi_ddot_ref

        d_hat_altatt = np.zeros(4)

        Fz_rfbl, tau_phi, tau_theta, tau_psi = self.inner_ctrl.step(
            x=x_inner,
            xd=xd_inner,
            xd_dot=xd_dot_inner,
            d_hat=d_hat_altatt,
            dt=self.dt,
        )

        # --- SOFTEN RFBL ALTITUDE ACTION ---------------------------------------------------------------------
        # Blend RFBL output around nominal hover force
        alpha = 0.3  # 0 = ignore RFBL, 1 = full RFBL; start with 0.3
        Fz = self.hover_Fz + alpha * (Fz_rfbl - self.hover_Fz)

        # Optional: safety clamp so it can't go crazy
        Fz = float(np.clip(Fz, 0.85 * self.hover_Fz, 1.15 * self.hover_Fz))

        # store for XY outer loop
        self.last_Fz = Fz

        # Map Fz (Newtons) -> PX4 thrust [0, 1]
        thrust_ratio = Fz / self.hover_Fz          # around 1.0 at hover
        thrust_norm = self.hover_thrust_norm * thrust_ratio
        thrust_norm = float(np.clip(thrust_norm, 0.0, 1.0))

        # 5) Build AttitudeTarget message using phi_des, theta_des, psi_ref
        qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_ref)

        att_msg = AttitudeTarget()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.type_mask = 0  # use all fields

        att_msg.orientation.x = qx
        att_msg.orientation.y = qy
        att_msg.orientation.z = qz
        att_msg.orientation.w = qw

        att_msg.body_rate.x = 0.0
        att_msg.body_rate.y = 0.0
        att_msg.body_rate.z = 0.0

        # Safety: if not in OFFBOARD or not armed, don't send crazy thrust
        if self.current_state is not None:
            if self.current_state.mode != "OFFBOARD" or not self.current_state.armed:
                # While waiting for OFFBOARD+ARM, keep near hover or slightly less
                thrust_norm = min(thrust_norm, 0.6)
        att_msg.thrust = thrust_norm

        # --- Debug print at ~1 Hz ---
        self.debug_counter += 1
        if self.debug_counter >= int(self.control_rate):
            self.debug_counter = 0
            self.get_logger().info(
                f"t={t:.1f}s "
                f"x={x_pos:.2f}, y={y_pos:.2f}, z={z_pos:.2f} | "
                f"x_ref={x_ref:.2f}, y_ref={y_ref:.2f}, z_ref={z_ref:.2f} | "
                f"phi_des={np.rad2deg(phi_des):.1f}deg, "
                f"theta_des={np.rad2deg(theta_des):.1f}deg "
                f"thrust={thrust_norm:.2f} "
            )

        # ------------------------------
        # Collect data for plotting
        # ------------------------------
        self.log_t.append(t)
        self.log_x.append(x_pos)
        self.log_y.append(y_pos)
        self.log_z.append(z_pos)
        self.log_xref.append(x_ref)
        self.log_yref.append(y_ref)
        self.log_zref.append(z_ref)
        self.log_phi.append(np.rad2deg(phi_des))
        self.log_theta.append(np.rad2deg(theta_des))
        self.log_thrust.append(thrust_norm)
        self.att_sp_pub.publish(att_msg)

    
    def simple_hover_fallback(self):
        yaw = 0.0
        if self.current_pose is not None:
            ori = self.current_pose.pose.orientation
            quat = [ori.x, ori.y, ori.z, ori.w]
            try:
                _, _, yaw = euler_from_quaternion(quat)
            except Exception:
                yaw = 0.0

        phi_des = 0.0
        theta_des = 0.0
        psi_des = yaw

        qx, qy, qz, qw = quaternion_from_euler(phi_des, theta_des, psi_des)

        att_msg = AttitudeTarget()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.type_mask = 0

        att_msg.orientation.x = qx
        att_msg.orientation.y = qy
        att_msg.orientation.z = qz
        att_msg.orientation.w = qw

        att_msg.body_rate.x = 0.0
        att_msg.body_rate.y = 0.0
        att_msg.body_rate.z = 0.0

        att_msg.thrust = float(self.hover_thrust_norm)
        self.att_sp_pub.publish(att_msg)



    def save_plots(self):
        import matplotlib.pyplot as plt
        import os

        # print("f1")
        # os.makedirs("rfbl_plots", exist_ok=True)
        # print("f2")

       

        # Absolute directory under your home
        base_dir = os.path.join(os.path.expanduser("~"), "rfbl_plots")
        os.makedirs(base_dir, exist_ok=True)

        # Debug info to prove what code runs
        # -------------------------------
        cwd = os.getcwd()
        self.get_logger().info(f"[save_plots] Current working directory: {cwd}")
        self.get_logger().info(f"[save_plots] Saving plots to: {base_dir}")
        self.get_logger().info(f"[save_plots] Logged samples: {len(self.log_t)}")

        self.get_logger().info(f"Saving plots to: {base_dir}")
        # ---- X tracking ----
        plt.figure()
        plt.plot(self.log_t, self.log_x, label="x")
        plt.plot(self.log_t, self.log_xref, "--", label="x_ref")
        plt.xlabel("Time [s]")
        plt.ylabel("X [m]")
        plt.legend()
        plt.grid()
        plt.title("X Tracking")
        plt.savefig("rfbl_plots/x_tracking.png")
        plt.close()

        # ---- Y tracking ----
        plt.figure()
        plt.plot(self.log_t, self.log_y, label="y")
        plt.plot(self.log_t, self.log_yref, "--", label="y_ref")
        plt.xlabel("Time [s]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid()
        plt.title("Y Tracking")
        plt.savefig("rfbl_plots/y_tracking.png")
        plt.close()

        # ---- Z tracking ----
        plt.figure()
        plt.plot(self.log_t, self.log_z, label="z")
        plt.plot(self.log_t, self.log_zref, "--", label="z_ref")
        plt.xlabel("Time [s]")
        plt.ylabel("Z [m]")
        plt.legend()
        plt.grid()
        plt.title("Z Tracking")
        plt.savefig("rfbl_plots/z_tracking.png")
        plt.close()

        # ---- XY Trajectory ----
        plt.figure()
        plt.plot(self.log_x, self.log_y, label="Trajectory")
        plt.plot(self.log_xref, self.log_yref, "--", label="Reference")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid()
        plt.title("XY Trajectory")
        plt.axis("equal")
        plt.savefig("rfbl_plots/xy_trajectory.png")
        plt.close()

        # ---- Roll / Pitch ----
        plt.figure()
        plt.plot(self.log_t, self.log_phi, label="phi_des [deg]")
        plt.plot(self.log_t, self.log_theta, label="theta_des [deg]")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [deg]")
        plt.legend()
        plt.grid()
        plt.title("Desired Roll / Pitch")
        plt.savefig("rfbl_plots/angles.png")
        plt.close()

        # ---- Thrust ----
        plt.figure()
        plt.plot(self.log_t, self.log_thrust, label="thrust")
        plt.xlabel("Time [s]")
        plt.ylabel("Thrust")
        plt.grid()
        plt.title("Thrust Command")
        plt.savefig("rfbl_plots/thrust.png")
        plt.close()

        self.get_logger().info("Plots saved to folder: rfbl_plots/")


# def main(args=None):
#     rclpy.init(args=args)
#     node = RfblOffboardNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nSaving plots ...")
#         node.save_plots()
        
#     node.destroy_node()
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RfblOffboardNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("CTRL+C detected — saving plots...")

    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")

    finally:
        # Save plots even if shutdown happens unexpectedly
        try:
            node.save_plots()
        except Exception as e:
            print(f"[ERROR] Failed to save plots: {e}")

        node.destroy_node()

        # Avoid shutdown error if context was already shut down
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
