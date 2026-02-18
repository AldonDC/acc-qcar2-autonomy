#!/usr/bin/env python3
"""
Waypoints visuales en RViz: escucha /clicked_point (herramienta 'Publish Point' en RViz),
acumula puntos y al pulsar Enter publica la ruta en /waypoints_path para el seguidor.
También publica marcadores en /waypoint_markers para ver los puntos en RViz.
"""
import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


class WaypointVisualNode(Node):
    def __init__(self):
        super().__init__("waypoint_visual_node")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/waypoint_markers")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("send_trigger_topic", "/waypoints_send_trigger")
        self.declare_parameter("max_waypoints", 30)

        path_topic = self.get_parameter("waypoints_path_topic").value
        trigger_topic = self.get_parameter("send_trigger_topic").value
        self.max_waypoints = self.get_parameter("max_waypoints").value
        markers_topic = self.get_parameter("markers_topic").value
        clicked_topic = self.get_parameter("clicked_point_topic").value

        self.points = []
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.last_send_time = 0.0
        self.send_cooldown_sec = 3.0
        self.sub = self.create_subscription(
            PointStamped,
            clicked_topic,
            self.clicked_callback,
            10,
        )
        self.trigger_sub = self.create_subscription(
            Empty,
            trigger_topic,
            self._trigger_callback,
            10,
        )

        self.get_logger().info(
            f"Waypoints visuales: haz clic en el mapa (RViz → Tool 'Publish Point'). "
            f"Pulsa ENTER en esta terminal para enviar la ruta al seguidor."
        )
        self.get_logger().info(f"Escuchando: {clicked_topic} | Trigger: {trigger_topic} | Publicando: {path_topic}")

    def _trigger_callback(self, _msg: Empty):
        self.publish_path(force=True)

    def clicked_callback(self, msg: PointStamped):
        if len(self.points) >= self.max_waypoints:
            self.get_logger().warn(f"Máximo {self.max_waypoints} waypoints. Escribe 'c' para borrar y empezar de nuevo.")
            return
        x, y = msg.point.x, msg.point.y
        self.points.append((x, y))
        self.get_logger().info(f"Waypoint {len(self.points)}: ({x:.2f}, {y:.2f})")
        self._publish_markers()

    def _publish_markers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.points):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.markers_pub.publish(ma)

    def publish_path(self, force=False):
        if len(self.points) < 2:
            self.get_logger().warn("Necesitas al menos 2 waypoints. Haz clic en el mapa en RViz.")
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if not force and now - self.last_send_time < self.send_cooldown_sec:
            self.get_logger().warn("Espera unos segundos antes de reenviar (evita ciclos). Para otra ruta: c = borrar, clics, Enter.")
            return
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.points:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)
        self.last_send_time = now
        self.get_logger().info(f"Ruta enviada: {len(self.points)} waypoints. El coche sigue la ruta.")
        self.get_logger().info("Para otra ruta: escribe 'c' (borrar), clic en RViz de nuevo y Enter.")

    def clear(self):
        self.points.clear()
        self._publish_markers()
        self.get_logger().info("Waypoints borrados.")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                if line.strip().lower() in ("", "s", "send", "go"):
                    node.publish_path()
                elif line.strip().lower() in ("c", "clear", "borrar"):
                    node.clear()
                else:
                    print("  Enter/s = enviar ruta al seguidor | c = borrar waypoints")
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
