#!/usr/bin/env python3
"""
Seguidor de ruta con controlador Stanley y pose real desde TF (map -> base_link).
Sigue el path (cross-track + heading) y suele pasar mejor por los waypoints que Pure Pursuit.
Publica en /cmd_vel_nav (linear.x = v, angular.z = steering_angle en rad). Escucha /waypoints_path.
Mismo protocolo que Pure Pursuit para el converter (angular.z = ángulo de dirección).
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def quaternion_to_yaw(q):
    """Yaw (en radianes) desde geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_to_segment_dist(px, py, x1, y1, x2, y2):
    """Distancia de (px,py) al segmento (x1,y1)-(x2,y2). Devuelve (dist, t) con t en [0,1]."""
    dx = x2 - x1
    dy = y2 - y1
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return math.hypot(px - x1, py - y1), 0.0
    t = ((px - x1) * dx + (py - y1) * dy) / (seg_len * seg_len)
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    dist = math.hypot(px - proj_x, py - proj_y)
    return dist, t


class WaypointFollowerStanley(Node):
    def __init__(self):
        super().__init__("waypoint_follower_stanley")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("max_speed", 0.4)
        self.declare_parameter("lookahead_distance", 0.35)
        self.declare_parameter("k_stanley", 1.0)
        self.declare_parameter("waypoint_reached_radius", 0.18)  # distancia (m) para considerar waypoint alcanzado
        self.declare_parameter("wheelbase", 0.2)
        self.declare_parameter("max_steering_angle", 0.6)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("waypoints_path_topic").value,
            self.path_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints = []
        self.last_path_hash = None
        self.route_complete = False
        self.path_progress_max = 0.0

        self.max_speed = self.get_parameter("max_speed").value
        self.lookahead = self.get_parameter("lookahead_distance").value
        self.k_stanley = self.get_parameter("k_stanley").value
        self.waypoint_reached_radius = self.get_parameter("waypoint_reached_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.max_steer = self.get_parameter("max_steering_angle").value
        self.dt = 1.0 / self.get_parameter("control_hz").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.get_logger().info(
            "Waypoint follower Stanley (pose desde TF map->base_link). "
            "Escucha /waypoints_path. Publica /cmd_vel_nav."
        )
        self.timer = self.create_timer(self.dt, self.control_callback)

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        new_wp = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        path_hash = tuple(tuple(w) for w in new_wp)
        if path_hash == self.last_path_hash and not self.route_complete:
            return
        self.waypoints = new_wp
        self.last_path_hash = path_hash
        self.route_complete = False
        self.path_progress_max = 0.0
        self.min_progress_meters = 0.6  # no marcar "ruta completada" hasta haber avanzado al menos esto
        self.get_logger().info(f"Nueva ruta: {len(self.waypoints)} waypoints (Stanley)")

    def get_pose_from_tf(self):
        """Pose (x, y, theta) en frame map. None si no hay TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            theta = quaternion_to_yaw(q)
            return (x, y, theta)
        except TransformException:
            return None

    def control_callback(self):
        if len(self.waypoints) < 2:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        pose = self.get_pose_from_tf()
        if pose is None:
            return
        x, y, theta = pose

        n = len(self.waypoints)
        # Encontrar segmento más cercano y posición a lo largo del path
        best_dist = 1e9
        best_i = 0
        best_t = 0.0
        for i in range(n - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            d, t = point_to_segment_dist(x, y, x1, y1, x2, y2)
            if d < best_dist:
                best_dist = d
                best_i = i
                best_t = t

        path_len = 0.0
        for i in range(n - 1):
            path_len += math.hypot(
                self.waypoints[i + 1][0] - self.waypoints[i][0],
                self.waypoints[i + 1][1] - self.waypoints[i][1],
            )
        seg_start = 0.0
        for i in range(best_i):
            seg_start += math.hypot(
                self.waypoints[i + 1][0] - self.waypoints[i][0],
                self.waypoints[i + 1][1] - self.waypoints[i][1],
            )
        seg_len = math.hypot(
            self.waypoints[best_i + 1][0] - self.waypoints[best_i][0],
            self.waypoints[best_i + 1][1] - self.waypoints[best_i][1],
        )
        current_path_pos = seg_start + best_t * seg_len
        # Solo aumentar progreso cuando no estamos "pasados" del final (evita path_progress_max = path_len al inicio)
        if path_len > 1e-6 and (current_path_pos + self.lookahead) < path_len - 0.05:
            self.path_progress_max = max(self.path_progress_max, current_path_pos)

        # No marcar "ruta completada" hasta haber avanzado al menos min_progress_meters (evita parar al inicio)
        last = self.waypoints[-1]
        near_last = math.hypot(x - last[0], y - last[1]) < self.waypoint_reached_radius
        if near_last and path_len > 1e-6 and self.path_progress_max >= self.min_progress_meters:
            self.route_complete = True
            self.get_logger().info("Ruta completada (Stanley). Parado.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        target_path_pos = current_path_pos + self.lookahead
        if target_path_pos >= path_len and path_len > 1e-6 and self.path_progress_max >= self.min_progress_meters:
            self.route_complete = True
            self.get_logger().info("Ruta completada (Stanley). Parado.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return
        if target_path_pos >= path_len:
            target_path_pos = path_len - 0.01

        # Interpolar punto objetivo en el path
        acc = 0.0
        tx, ty = self.waypoints[0][0], self.waypoints[0][1]
        for i in range(n - 1):
            seg_l = math.hypot(
                self.waypoints[i + 1][0] - self.waypoints[i][0],
                self.waypoints[i + 1][1] - self.waypoints[i][1],
            )
            if acc + seg_l >= target_path_pos:
                r = (target_path_pos - acc) / seg_l if seg_l > 1e-6 else 0.0
                tx = self.waypoints[i][0] + r * (self.waypoints[i + 1][0] - self.waypoints[i][0])
                ty = self.waypoints[i][1] + r * (self.waypoints[i + 1][1] - self.waypoints[i][1])
                break
            acc += seg_l
        # Tangente del path en el punto objetivo
        if best_i + 1 < n:
            dx_path = self.waypoints[best_i + 1][0] - self.waypoints[best_i][0]
            dy_path = self.waypoints[best_i + 1][1] - self.waypoints[best_i][1]
        else:
            dx_path = self.waypoints[best_i][0] - self.waypoints[best_i - 1][0]
            dy_path = self.waypoints[best_i][1] - self.waypoints[best_i - 1][1]
        path_angle = math.atan2(dy_path, dx_path)

        # Cross-track error (con signo: positivo = coche a la izquierda del path)
        x1, y1 = self.waypoints[best_i][0], self.waypoints[best_i][1]
        x2, y2 = self.waypoints[best_i + 1][0], self.waypoints[best_i + 1][1]
        dx_seg = x2 - x1
        dy_seg = y2 - y1
        cross = (x - x1) * (-dy_seg) + (y - y1) * dx_seg
        seg_norm = math.hypot(dx_seg, dy_seg)
        if seg_norm > 1e-6:
            cross /= seg_norm
        heading_error = wrap_angle(path_angle - theta)

        # Stanley: delta (ángulo de dirección) = heading_error + atan(k * e / (v + eps))
        v = min(self.max_speed, 0.15 + 0.3 * (1.0 - abs(heading_error)))
        eps = 0.01
        delta = heading_error + math.atan2(self.k_stanley * cross, v + eps)
        delta = max(-self.max_steer, min(self.max_steer, delta))

        # Igual que Pure Pursuit: el converter espera angular.z = steering_angle (rad), no omega
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(delta)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerStanley()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        if rclpy.ok():
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            node.pub.publish(msg)
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
