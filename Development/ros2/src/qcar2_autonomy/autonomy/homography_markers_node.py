#!/usr/bin/env python3
"""
Publica marcadores en RViz en las esquinas y el centro de la calibración homográfica
(pixel -> mundo). Sirve para comprobar si la homografía coincide con el circuito en RViz.

Frame: map. Añade en RViz un display "MarkerArray" con topic /homography_calibration_markers.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class HomographyMarkersNode(Node):
    def __init__(self):
        super().__init__("homography_markers_node")
        self.declare_parameter("marker_topic", "/homography_calibration_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("sphere_radius", 0.15)

        self._pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            5,
        )
        self._frame_id = self.get_parameter("frame_id").value
        self._radius = self.get_parameter("sphere_radius").value
        rate = self.get_parameter("rate_hz").value
        self._timer = self.create_timer(1.0 / rate if rate > 0 else 1.0, self._publish)

        try:
            from autonomy.homography import load_calibration, pixel_to_world
            if not load_calibration():
                self.get_logger().error("No se pudo cargar homography_calibration.yaml")
                return
            # Esquinas y centro en pixel (pista_qcar2.png 1152x896)
            self._world_points = [
                ("sup_izq", (0, 0), (1.0, 0.2, 0.2)),       # rojo
                ("sup_der", (1152, 0), (0.2, 1.0, 0.2)),     # verde
                ("inf_der", (1152, 896), (0.2, 0.2, 1.0)),   # azul
                ("inf_izq", (0, 896), (1.0, 1.0, 0.0)),     # amarillo
                ("centro", (576, 448), (1.0, 0.5, 0.0)),     # naranja
            ]
            self._ready = True
            self.get_logger().info(
                "Homografía cargada. Publicando marcadores en %s (frame %s)"
                % (self.get_parameter("marker_topic").value, self._frame_id)
            )
        except Exception as e:
            self.get_logger().error("Homografía: %s" % e)
            self._ready = False

    def _publish(self):
        if not getattr(self, "_ready", False):
            return
        try:
            from autonomy.homography import pixel_to_world
        except ImportError:
            return
        ma = MarkerArray()
        for i, (name, (px, py), color) in enumerate(self._world_points):
            wx, wy = pixel_to_world(px, py)
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "homography_cal"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = self._radius * 2
            m.scale.y = self._radius * 2
            m.scale.z = self._radius * 2
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 0.9
            ma.markers.append(m)
        self._pub.publish(ma)


def main():
    rclpy.init()
    node = HomographyMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
