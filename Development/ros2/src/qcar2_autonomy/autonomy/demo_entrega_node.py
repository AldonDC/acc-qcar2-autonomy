#!/usr/bin/env python3
"""
Demo de entrega: publica una ruta fija en forma de cuadrado (1.5 m lado) en el mapa
y los mismos puntos como marcadores. El QCar la sigue y se ve claro en RViz.
Ruta: (0,0) -> (1.5,0) -> (1.5,1.5) -> (0,1.5) -> (0,0).
Coloca el robot cerca del origen del mapa para la demo.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


# Cuadrado de 1.5 m de lado en el plano map (x, y)
DEMO_WAYPOINTS = [
    (0.0, 0.0),
    (1.5, 0.0),
    (1.5, 1.5),
    (0.0, 1.5),
    (0.0, 0.0),
]


class DemoEntregaNode(Node):
    def __init__(self):
        super().__init__("demo_entrega_node")
        self.declare_parameter("delay_sec", 2.0)
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/waypoint_markers")

        path_topic = self.get_parameter("waypoints_path_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        delay = self.get_parameter("delay_sec").value

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.sent = False
        self.timer = self.create_timer(delay, self.publish_once)

        self.get_logger().info(
            f"Demo entrega: en {delay}s publicaré ruta en forma de cuadrado ({path_topic}). "
            "El QCar la seguirá. En RViz verás la línea (Path) y las esferas (WaypointMarkers)."
        )

    def publish_once(self):
        if self.sent:
            return
        self.sent = True
        self.timer.cancel()

        now = self.get_clock().now().to_msg()
        frame = "map"

        # Path para el seguidor
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = now
        for x, y in DEMO_WAYPOINTS:
            p = PoseStamped()
            p.header.frame_id = frame
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)

        # Marcadores para RViz (esferas verdes)
        ma = MarkerArray()
        for i, (x, y) in enumerate(DEMO_WAYPOINTS):
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
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 0.9
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.markers_pub.publish(ma)

        self.get_logger().info(
            f"Ruta demo publicada: {len(DEMO_WAYPOINTS)} waypoints (cuadrado 1.5 m). "
            "El QCar debe seguir la ruta. Mira la línea verde en RViz."
        )


def main(args=None):
    rclpy.init(args=args)
    node = DemoEntregaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
