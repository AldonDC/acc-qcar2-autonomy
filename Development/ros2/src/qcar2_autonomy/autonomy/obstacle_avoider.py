#!/usr/bin/env python3
"""
Evitación de obstáculos con controlador proporcional usando /scan.
Reduce velocidad si hay algo delante y gira hacia el lado con más espacio.
Publica en /cmd_vel_nav.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safe_distance", 0.5)
        self.declare_parameter("front_angle_deg", 60.0)  # sector delante (grados totales)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("kp_speed", 0.6)
        self.declare_parameter("kp_turn", 1.2)

        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )
        self.safe_dist = self.get_parameter("safe_distance").value
        self.front_angle_deg = self.get_parameter("front_angle_deg").value
        self.max_speed = self.get_parameter("max_speed").value
        self.kp_speed = self.get_parameter("kp_speed").value
        self.kp_turn = self.get_parameter("kp_turn").value

        self.last_scan = None
        self.timer = self.create_timer(0.05, self.control_callback)  # 20 Hz

        self.get_logger().info(
            f"Obstacle avoider: safe_dist={self.safe_dist}m, max_speed={self.max_speed}"
        )

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def control_callback(self):
        if self.last_scan is None:
            msg = Twist()
            msg.linear.x = 0.15
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        s = self.last_scan
        n = len(s.ranges)
        if n == 0:
            return
        angle_min = s.angle_min
        angle_inc = s.angle_increment
        ranges = list(s.ranges)
        for i in range(n):
            if not (s.range_min <= ranges[i] <= s.range_max):
                ranges[i] = float("inf")

        # Índices del sector frontal (centrado en 0 = delante del robot)
        half_span = (self.front_angle_deg / 2.0) * math.pi / 180.0
        front_min_dist = float("inf")
        left_min = float("inf")
        right_min = float("inf")
        for i in range(n):
            angle = angle_min + i * angle_inc
            if -half_span <= angle <= half_span:
                front_min_dist = min(front_min_dist, ranges[i])
            elif angle < 0:
                left_min = min(left_min, ranges[i])
            else:
                right_min = min(right_min, ranges[i])

        if math.isinf(front_min_dist):
            front_min_dist = 2.0
        if math.isinf(left_min):
            left_min = 2.0
        if math.isinf(right_min):
            right_min = 2.0

        # Controlador: velocidad proporcional a distancia al obstáculo frontal
        if front_min_dist <= self.safe_distance:
            speed = self.kp_speed * (front_min_dist - 0.15)
            speed = max(0.0, min(speed, self.max_speed * 0.5))
            # Giro proporcional: más espacio a la derecha -> girar derecha (omega negativo)
            turn = self.kp_turn * (right_min - left_min)
            turn = max(-1.0, min(1.0, turn))
        else:
            speed = self.max_speed
            turn = 0.0

        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(-turn)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    node.pub.publish(msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
