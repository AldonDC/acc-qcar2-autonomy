#!/usr/bin/env python3
"""
Bridge: publica waypoints desde un archivo YAML (o lista en Python) a /waypoints_path
y /waypoint_markers. Así puedes definir la ruta en código o en un archivo sin depender
de hacer clic en RViz. El mapa lo sigues viendo en RViz; los puntos los defines aquí.

Uso:
  ros2 run qcar2_autonomy waypoint_publish_from_file
  ros2 run qcar2_autonomy waypoint_publish_from_file --ros-args -p waypoints_file:=/ruta/a/mis_waypoints.yaml
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def load_waypoints_from_yaml(filepath: str):
    """Carga waypoints desde YAML: { waypoints: [ [x,y], ... ] }."""
    import yaml
    with open(filepath, "r") as f:
        data = yaml.safe_load(f)
    wp = data.get("waypoints", [])
    return [(float(p[0]), float(p[1])) for p in wp]


class WaypointPublishFromFileNode(Node):
    def __init__(self):
        super().__init__("waypoint_publish_from_file")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/waypoint_markers")
        self.declare_parameter("delay_sec", 1.0)

        path_topic = self.get_parameter("waypoints_path_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        delay = self.get_parameter("delay_sec").value
        filepath = self.get_parameter("waypoints_file").value

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)

        waypoints = []
        if filepath:
            try:
                waypoints = load_waypoints_from_yaml(filepath)
                self.get_logger().info(f"Waypoints cargados desde {filepath}: {len(waypoints)} puntos")
            except Exception as e:
                self.get_logger().error(f"No se pudo cargar {filepath}: {e}. Usando ruta por defecto.")
                waypoints = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)]
        if not waypoints:
            waypoints = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)]
            self.get_logger().info("Sin waypoints_file: usando ruta por defecto (cuadrado 1.5 m)")

        if len(waypoints) < 2:
            self.get_logger().error("Se necesitan al menos 2 waypoints.")
            return

        self.waypoints = waypoints
        self.sent = False
        self.timer = self.create_timer(delay, self.publish_once)

        self.get_logger().info(
            f"Publicaré ruta en {path_topic} y {markers_topic}. "
            "Abre RViz para ver el mapa y la ruta (Path + WaypointMarkers)."
        )

    def publish_once(self):
        if self.sent:
            return
        self.sent = True
        self.timer.cancel()

        now = self.get_clock().now().to_msg()
        frame = "map"

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = now
        for x, y in self.waypoints:
            p = PoseStamped()
            p.header.frame_id = frame
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)

        ma = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 0.9
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.markers_pub.publish(ma)

        self.get_logger().info(
            f"Ruta publicada: {len(self.waypoints)} waypoints. "
            "El seguidor (waypoint_follower_controller) la seguirá. Ver en RViz."
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublishFromFileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
