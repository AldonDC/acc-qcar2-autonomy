#!/usr/bin/env python3
"""
Seguidor de waypoints con controlador P.
Estima la pose por dead-reckoning (integra los cmd_vel que envía) y genera
velocidad lineal y angular hacia el siguiente waypoint.
Publica en /cmd_vel_nav.

Waypoints: parámetro 'waypoints' = lista plana [x1, y1, x2, y2, ...] en metros.
Ejemplo con tus puntos:
  ros2 run qcar2_autonomy waypoint_follower_controller --ros-args \
    -p waypoints:="[0.5, 0.0, 1.0, 0.3, 1.0, 0.8, 0.2, 0.8, 0.2, 0.0]"
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class WaypointFollowerController(Node):
    def __init__(self):
        super().__init__("waypoint_follower_controller")
        # waypoints: lista plana [x1, y1, x2, y2, ...] en metros (fácil por línea de comandos)
        self.declare_parameter(
            "waypoints",
            [1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0],  # cuadrado por defecto
        )
        self.declare_parameter("waypoint_tolerance", 0.45)
        self.declare_parameter("waypoint_timeout_sec", 25.0)  # si no llegamos en este tiempo, pasamos al siguiente (evita ciclos)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("kp_linear", 0.7)
        self.declare_parameter("kp_angular", 1.6)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("use_rviz_only", False)  # True = empezar sin waypoints, solo los de /waypoints_path

        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.route_complete = False
        self.time_at_current_waypoint = None  # para timeout y no quedarse ciclando
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("waypoints_path_topic").value,
            self.path_callback,
            10,
        )
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wp_index = 0
        self.last_v = 0.0
        self.last_omega = 0.0
        self.dt = 1.0 / self.get_parameter("control_hz").value

        use_rviz_only = self.get_parameter("use_rviz_only").value
        flat = self.get_parameter("waypoints").value
        if use_rviz_only:
            self.waypoints = []  # solo waypoints desde topic (RViz)
        elif isinstance(flat, list) and len(flat) == 0:
            self.waypoints = []
        elif isinstance(flat, list) and len(flat) >= 2:
            if isinstance(flat[0], (list, tuple)):
                self.waypoints = [[float(w[0]), float(w[1])] for w in flat]
            else:
                self.waypoints = [[float(flat[i]), float(flat[i + 1])] for i in range(0, len(flat) - 1, 2)]
        else:
            self.waypoints = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]
        self.tol = self.get_parameter("waypoint_tolerance").value
        self.wp_timeout = self.get_parameter("waypoint_timeout_sec").value
        self.max_speed = self.get_parameter("max_speed").value
        self.kp_lin = self.get_parameter("kp_linear").value
        self.kp_ang = self.get_parameter("kp_angular").value

        self.path_received = False
        self.last_path_hash = None
        self.get_logger().info(
            f"Waypoint follower (controlador P): tol={self.tol}m. "
            f"Escucha /waypoints_path (waypoints desde RViz)."
        )
        if self.waypoints:
            self.get_logger().info(f"Ruta inicial: {' -> '.join(f'({x:.2f},{y:.2f})' for x, y in self.waypoints)}")
        self.timer = self.create_timer(self.dt, self.control_callback)

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        new_waypoints = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        path_hash = tuple(tuple(w) for w in new_waypoints)
        if path_hash == self.last_path_hash and not self.route_complete:
            return
        self.waypoints = new_waypoints
        self.wp_index = 0
        self.last_path_hash = path_hash
        self.route_complete = False
        self.time_at_current_waypoint = self.get_clock().now()
        self.path_received = True
        self.get_logger().info(f"Nueva ruta desde RViz: {len(self.waypoints)} waypoints")
        self.get_logger().info(f"Ruta: {' -> '.join(f'({x:.2f},{y:.2f})' for x, y in self.waypoints)}")

    def control_callback(self):
        if self.wp_index >= len(self.waypoints):
            if len(self.waypoints) > 0 and not self.route_complete:
                self.route_complete = True
                self.get_logger().info("Ruta completada. Parado. Envía otra ruta desde RViz o la GUI si quieres.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        wx, wy = self.waypoints[self.wp_index]
        dx = wx - self.x
        dy = wy - self.y
        dist = math.hypot(dx, dy)

        if self.time_at_current_waypoint is None:
            self.time_at_current_waypoint = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.time_at_current_waypoint).nanoseconds / 1e9
        if elapsed > self.wp_timeout:
            self.get_logger().warn(f"Waypoint {self.wp_index + 1} timeout ({self.wp_timeout}s) -> siguiente.")
            self.wp_index += 1
            self.time_at_current_waypoint = self.get_clock().now()
            return

        if dist < self.tol:
            self.wp_index += 1
            self.time_at_current_waypoint = self.get_clock().now()
            if self.wp_index >= len(self.waypoints):
                self.route_complete = True
                self.get_logger().info("Ruta completada. Parado.")
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.pub.publish(msg)
                return
            wx, wy = self.waypoints[self.wp_index]
            dx = wx - self.x
            dy = wy - self.y
            dist = math.hypot(dx, dy)

        dx_robot = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        dy_robot = -dx * math.sin(self.theta) + dy * math.cos(self.theta)
        desired_heading = math.atan2(dy_robot, dx_robot)
        heading_error = wrap_angle(desired_heading)

        speed = self.kp_lin * dist
        speed = min(speed, self.max_speed)
        if dist < 0.15:
            speed = 0.12
        elif dist < 0.3:
            speed = min(speed, 0.2)
        omega = self.kp_ang * heading_error
        omega = max(-1.0, min(1.0, omega))

        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(omega)
        self.pub.publish(msg)

        self.last_v = speed
        self.last_omega = omega
        self.x += speed * math.cos(self.theta) * self.dt
        self.y += speed * math.sin(self.theta) * self.dt
        self.theta += omega * self.dt
        self.theta = wrap_angle(self.theta)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerController()
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
