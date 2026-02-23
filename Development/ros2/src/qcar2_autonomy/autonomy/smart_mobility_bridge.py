#!/_usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
import math

class SmartMobilityBridge(Node):
    """
    Bridge between qcar2_autonomy (ROS2 Standard / Simulation) 
    and Smart Mobility 2025 (Custom Competition Format).
    """
    def __init__(self):
        super().__init__('smart_mobility_bridge')
        
        # 1. Pose Bridge: /qcar_pose_gt (PoseStamped) -> /qcar/pose (Vector3Stamped)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/qcar_pose_gt',
            self.pose_callback,
            10
        )
        self.pose_pub = self.create_publisher(Vector3Stamped, '/qcar/pose', 10)
        
        # 2. Command Bridge: /qcar/user_command (Vector3Stamped) -> /cmd_vel_nav (Twist)
        self.cmd_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.cmd_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        self.get_logger().info("Smart Mobility Bridge Initialized")
        self.get_logger().info("Bridging: /qcar_pose_gt -> /qcar/pose")
        self.get_logger().info("Bridging: /qcar/user_command -> /cmd_vel_nav")

    def pose_callback(self, msg: PoseStamped):
        # Convert PoseStamped to Vector3Stamped (x, y, yaw)
        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x = msg.pose.position.x
        out.vector.y = msg.pose.position.y
        
        # Quat to Yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        out.vector.z = yaw
        
        self.pose_pub.publish(out)

    def cmd_callback(self, msg: Vector3Stamped):
        # Convert Vector3Stamped (v, steer, 0) to Twist
        # Smart Mobility uses: x = velocity, y = -steer (usually)
        # Looking at qcar_pure_pursuit.py: cmd.vector.y = float(-steer_cmd)
        out = Twist()
        out.linear.x = msg.vector.x
        out.angular.z = -msg.vector.y # Sign flip if needed, assuming y is steer
        self.cmd_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SmartMobilityBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
