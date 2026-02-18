#!/usr/bin/env python3
"""
Waypoints desde RViz: clics en el mapa (herramienta Publish Point) añaden waypoints.
Servicios: clear_waypoints, send_path. Publica /waypoints_path y MarkerArray para visualización.
Más robusto que la GUI en Python; usa las dimensiones del mapa y RViz de forma nativa.
"""
import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty


class WaypointRvizNode(Node):
    def __init__(self):
        super().__init__("waypoint_rviz_node")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/waypoint_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("max_waypoints", 100)
        self.declare_parameter("load_file", "")      # Waypoints amarillos activos
        self.declare_parameter("reference_file", "") # Guía verde de referencia

        self._waypoints = []            # Waypoints amarillos (clics)
        self._reference_waypoints = []  # Waypoints verdes (guía)
        self._frame_id = self.get_parameter("frame_id").value
        self._max_wp = self.get_parameter("max_waypoints").value
        self._path_sent = False 

        # 1. Cargar GUÍA DE REFERENCIA (Verde)
        ref_file = self.get_parameter("reference_file").value
        if ref_file and os.path.exists(ref_file):
            try:
                import json
                with open(ref_file, "r") as f:
                    data = json.load(f)
                    for wp in data.get("waypoints", []):
                        self._reference_waypoints.append((wp["x"], wp["y"]))
                self.get_logger().info(f"🟢 Guía cargada: {len(self._reference_waypoints)} puntos de {ref_file}")
            except Exception as e:
                self.get_logger().error(f"❌ Error cargando referencia: {e}")

        # 2. Cargar WAYPOINTS ACTIVOS (Amarillo)
        load_file = self.get_parameter("load_file").value
        if load_file and os.path.exists(load_file):
            try:
                import json
                with open(load_file, "r") as f:
                    data = json.load(f)
                    for wp in data.get("waypoints", []):
                        self._waypoints.append((wp["x"], wp["y"]))
                self.get_logger().info(f"🟡 Waypoints cargados: {len(self._waypoints)} de {load_file}")
            except Exception as e:
                self.get_logger().error(f"❌ Error cargando JSON: {e}")

        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter("waypoints_path_topic").value,
            10,
        )
        self._markers_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("markers_topic").value,
            10,
        )
        self._sub_click = self.create_subscription(
            PointStamped,
            self.get_parameter("clicked_point_topic").value,
            self._on_clicked_point,
            10,
        )
        self._srv_clear = self.create_service(Empty, "clear_waypoints", self._on_clear)
        self._srv_send = self.create_service(Empty, "send_path", self._on_send_path)

        # Timer para re-publicar la ruta periódicamente (el Pure Pursuit la necesita)
        self._path_timer = self.create_timer(1.0, self._republish_path)
        
        # Publicar markers iniciales
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info(
            "waypoint_rviz_node: usa en RViz la herramienta 'Publish Point' para añadir waypoints. "
            "Servicios: clear_waypoints, send_path."
        )

    def _on_clicked_point(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        self._frame_id = msg.header.frame_id
        if len(self._waypoints) >= self._max_wp:
            self.get_logger().warn(f"Máximo {self._max_wp} waypoints; ignoro clic.")
            return
        self._waypoints.append((x, y))
        self._publish_markers()
        self.get_logger().info(f"Waypoint {len(self._waypoints)}: ({x:.3f}, {y:.3f})")

    def _on_clear(self, _req, _resp):
        self._waypoints.clear()
        self._path_sent = False
        self._publish_markers()
        self.get_logger().info("Waypoints borrados.")
        return _resp

    def _on_send_path(self, _req, _resp):
        n = self._publish_path()
        if n >= 2:
            self._path_sent = True
            self.get_logger().info(f"Ruta enviada: {n} waypoints a /waypoints_path. Se re-publicará cada segundo.")
        else:
            self.get_logger().warn("Necesitas al menos 2 waypoints. Añade puntos con Publish Point en RViz.")
        return _resp

    def _republish_path(self):
        """Re-publica la ruta periódicamente para que el Pure Pursuit la reciba de forma fiable."""
        if self._path_sent and len(self._waypoints) >= 2:
            self._publish_path()

    def _publish_markers(self):
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # 🟢 1. Guía de Referencia (Verde neón, traslúcido)
        # --- Línea conectora gruesa ---
        if len(self._reference_waypoints) > 1:
            line_ref = Marker()
            line_ref.header.frame_id = self._frame_id
            line_ref.header.stamp = now
            line_ref.ns = "reference_line"
            line_ref.id = 0
            line_ref.type = Marker.LINE_STRIP
            line_ref.action = Marker.ADD
            line_ref.scale.x = 0.04  # grosor visible
            line_ref.color.r, line_ref.color.g, line_ref.color.b, line_ref.color.a = 0.1, 0.9, 0.4, 0.6
            line_ref.pose.orientation.w = 1.0
            for x, y in self._reference_waypoints:
                pt = Point()
                pt.x, pt.y, pt.z = x, y, 0.01
                line_ref.points.append(pt)
            ma.markers.append(line_ref)

        # --- Puntos de referencia ---
        for i, (x, y) in enumerate(self._reference_waypoints):
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = now
            m.ns = "reference_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.02
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.4, 0.5
            ma.markers.append(m)

        # 🚩 1b. Marcador de INICIO (Azul, primer punto de referencia)
        if self._reference_waypoints:
            sx, sy = self._reference_waypoints[0]
            start_m = Marker()
            start_m.header.frame_id = self._frame_id
            start_m.header.stamp = now
            start_m.ns = "start_flag"
            start_m.id = 0
            start_m.type = Marker.CYLINDER
            start_m.action = Marker.ADD
            start_m.pose.position.x, start_m.pose.position.y, start_m.pose.position.z = sx, sy, 0.15
            start_m.scale.x = start_m.scale.y = 0.2
            start_m.scale.z = 0.3
            start_m.color.r, start_m.color.g, start_m.color.b, start_m.color.a = 0.2, 0.5, 1.0, 0.8
            ma.markers.append(start_m)

        # 🟡 2. Waypoints Seleccionados (Amarillo/ámbar sólido)
        # --- Línea conectora ---
        if len(self._waypoints) > 1:
            line_act = Marker()
            line_act.header.frame_id = self._frame_id
            line_act.header.stamp = now
            line_act.ns = "waypoints_line"
            line_act.id = 0
            line_act.type = Marker.LINE_STRIP
            line_act.action = Marker.ADD
            line_act.scale.x = 0.06  # más gruesa
            line_act.color.r, line_act.color.g, line_act.color.b, line_act.color.a = 1.0, 0.85, 0.0, 0.9
            line_act.pose.orientation.w = 1.0
            for x, y in self._waypoints:
                pt = Point()
                pt.x, pt.y, pt.z = x, y, 0.04
                line_act.points.append(pt)
            ma.markers.append(line_act)

        for i, (x, y) in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = now
            m.ns = "waypoints_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 1.0
            ma.markers.append(m)

        # 🏁 3. Punto de meta (Cilindro Rojo)
        if self._waypoints:
            fx, fy = self._waypoints[-1]
            end_m = Marker()
            end_m.header.frame_id = self._frame_id
            end_m.header.stamp = now
            end_m.ns = "finish_flag"
            end_m.id = 0
            end_m.type = Marker.CYLINDER
            end_m.action = Marker.ADD
            end_m.pose.position.x, end_m.pose.position.y, end_m.pose.position.z = fx, fy, 0.2
            end_m.scale.x = end_m.scale.y = 0.25
            end_m.scale.z = 0.4
            end_m.color.r, end_m.color.g, end_m.color.b, end_m.color.a = 0.9, 0.1, 0.1, 0.8
            ma.markers.append(end_m)

        # 🏹 4. Flechas de Dirección (cada 3 puntos para no saturar)
        def add_arrows(points, color, ns, z_offset, step=3):
            import math
            if len(points) < 2: return
            for i in range(0, len(points) - 1, step):
                p1 = points[i]
                p2 = points[min(i + 1, len(points) - 1)]
                yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                m = Marker()
                m.header.frame_id = self._frame_id
                m.header.stamp = now
                m.ns = ns
                m.id = i
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = p1[0], p1[1], z_offset
                m.pose.orientation.z = math.sin(yaw / 2.0)
                m.pose.orientation.w = math.cos(yaw / 2.0)
                m.scale.x, m.scale.y, m.scale.z = 0.2, 0.06, 0.06
                m.color.r, m.color.g, m.color.b, m.color.a = color
                ma.markers.append(m)

        add_arrows(self._reference_waypoints, (0.1, 0.7, 0.3, 0.5), "ref_arrows", 0.03, step=5)
        add_arrows(self._waypoints, (1.0, 0.7, 0.0, 0.9), "wp_arrows", 0.07, step=3)

        self._markers_pub.publish(ma)

    def _publish_path(self) -> int:
        if len(self._waypoints) < 2:
            return 0
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = now
        from geometry_msgs.msg import PoseStamped
        for x, y in self._waypoints:
            p = PoseStamped()
            p.header.frame_id = self._frame_id
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self._path_pub.publish(path)
        return len(self._waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRvizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
