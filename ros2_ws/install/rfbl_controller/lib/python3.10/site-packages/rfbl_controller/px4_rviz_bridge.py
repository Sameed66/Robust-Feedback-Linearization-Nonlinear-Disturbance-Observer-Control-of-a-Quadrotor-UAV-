import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PX4RvizBridge(Node):
    def __init__(self):
        super().__init__('px4_rviz_bridge')
        
        # Subscribe to PX4 Odometry (Best estimate from Flight Controller)
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            10)
            
        # Publishers for RViz
        self.odom_publisher = self.create_publisher(Odometry, '/px4_visualizer/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/px4_visualizer/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.path = Path()

    def listener_callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        
        # --- 1. Coordinate Frame Conversion (NED -> ENU) ---
        # PX4 (NED): x=North, y=East, z=Down
        # ROS (ENU): x=East, y=North, z=Up
        x_enu = msg.position[1]
        y_enu = msg.position[0]
        z_enu = -msg.position[2]
        
        # Orientation (Quaternion) mapping requires similar swapping
        # Note: A full math conversion is complex; for visualization, 
        # mapping x->y, y->x, z->-z roughly aligns the position.
        
        # --- 2. Publish TF (Crucial for RViz) ---
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu
        t.transform.rotation.w = 1.0 # Simplified rotation for path viz
        self.tf_broadcaster.sendTransform(t)

        # --- 3. Publish Path (Trajectory) ---
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = 'map'
        pose.pose.position.x = x_enu
        pose.pose.position.y = y_enu
        pose.pose.position.z = z_enu
        
        self.path.header.stamp = current_time
        self.path.header.frame_id = 'map'
        self.path.poses.append(pose)
        
        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PX4RvizBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()