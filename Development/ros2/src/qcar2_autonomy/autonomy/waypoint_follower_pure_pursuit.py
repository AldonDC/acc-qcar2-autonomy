#!/usr/bin/env python3
"""
Seguidor de ruta con controlador Pure Pursuit (estilo smart_mobility_2025 / PythonRobotics).
Pose desde TF (map -> base_link). Escucha /waypoints_path. Publica /cmd_vel_nav.
Fórmula: punto objetivo a lookahead en el path; delta = atan(2*L*sin(alpha)/Lf); omega = v*tan(delta)/L.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def quaternion_to_yaw(q):
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


class WaypointFollowerPurePursuit(Node):
    def __init__(self):
        super().__init__("waypoint_follower_pure_pursuit")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("min_speed", 0.12)
        self.declare_parameter("lookahead_distance", 0.4)
        self.declare_parameter("lookahead_gain", 0.0)  # Lf = lookahead_distance + lookahead_gain * v (0 = fijo)
        self.declare_parameter("waypoint_reached_radius", 0.2)
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
        self.min_progress_meters = 0.5

        self.max_speed = self.get_parameter("max_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.lookahead = self.get_parameter("lookahead_distance").value
        self.lookahead_gain = self.get_parameter("lookahead_gain").value
        self.waypoint_reached_radius = self.get_parameter("waypoint_reached_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.max_steer = self.get_parameter("max_steering_angle").value
        self.dt = 1.0 / self.get_parameter("control_hz").value
        self.target_frame = self.get_parameter("target_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.get_logger().info(
            "Waypoint follower Pure Pursuit (pose desde TF map->base_link). "
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
        self.get_logger().info(f"Nueva ruta: {len(self.waypoints)} waypoints (Pure Pursuit)")

    def get_pose_from_tf(self):
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
        if len(self.waypoints) < 2 or self.route_complete:
            return

        pose = self.get_pose_from_tf()
        if pose is None:
            return
        x, y, theta = pose

        n = len(self.waypoints)
        path_len = sum(
            math.hypot(self.waypoints[i + 1][0] - self.waypoints[i][0],
                       self.waypoints[i + 1][1] - self.waypoints[i][1])
            for i in range(n - 1)
        )
        if path_len < 1e-6:
            return

        # Índice más cercano al coche (posición a lo largo del path)
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

        # Progreso: solo contar cuando no estamos "pasados" del final
        if (current_path_pos + self.lookahead) < path_len - 0.05:
            self.path_progress_max = max(self.path_progress_max, current_path_pos)

        # ¿Cerca del último waypoint? (Solo si hemos avanzado algo)
        last = self.waypoints[-1]
        dist_to_last = math.hypot(x - last[0], y - last[1])
        near_last = dist_to_last < self.waypoint_reached_radius
        
        if near_last and self.path_progress_max >= self.min_progress_meters:
            self.route_complete = True
            self.get_logger().info(f"🏁 Ruta completada (d_meta={dist_to_last:.2f}m). Parado.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        # Punto objetivo: lookahead por delante en el path (Pure Pursuit)
        target_path_pos = current_path_pos + self.lookahead
        if target_path_pos >= path_len and self.path_progress_max >= self.min_progress_meters:
            self.route_complete = True
            self.get_logger().info("🏁 Fin de trayecto (lookahead fuera). Parado.")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return
            
        if target_path_pos >= path_len:
            target_path_pos = path_len - 0.01

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

        # Pure Pursuit: alpha = ángulo hacia (tx, ty) respecto al heading del coche
        dx_world = tx - x
        dy_world = ty - y
        Lf = math.hypot(dx_world, dy_world)
        if Lf < 1e-6:
            msg = Twist()
            msg.linear.x = float(self.min_speed)
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        # Ángulo al objetivo en mundo; alpha = diferencia con nuestro yaw
        angle_to_target = math.atan2(dy_world, dx_world)
        alpha = angle_to_target - theta
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        # delta = atan(2 * L * sin(alpha) / Lf) — Pure Pursuit (Ackermann)
        curvature = 2.0 * math.sin(alpha) / Lf
        delta = math.atan(self.wheelbase * curvature)
        delta = max(-self.max_steer, min(self.max_steer, delta))

        v = self.max_speed * (1.0 - 0.5 * min(1.0, abs(alpha)))
        v = max(self.min_speed, min(self.max_speed, v))

        # Log cada ~2 segundos para debug
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 60 == 1:
            self.get_logger().info(
                f"PP: pose=({x:.2f},{y:.2f},θ={math.degrees(theta):.0f}°) "
                f"target=({tx:.2f},{ty:.2f}) d_end={dist_to_last:.2f}m "
                f"v={v:.2f} δ={math.degrees(delta):.1f}°"
            )

        # IMPORTANTE: el nav2_qcar2_converter pasa angular.z directo como
        # steering_angle al hardware del QCar, así que publicamos delta (ángulo),
        # NO omega (velocidad angular).
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(delta)
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerPurePursuit()
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
