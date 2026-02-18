#!/usr/bin/env python3
"""
Route Player — Lee una ruta grabada (JSON) y la envía al Pure Pursuit.
Publica la ruta como nav_msgs/Path y markers de visualización en RViz.

Uso:
  ros2 run qcar2_autonomy route_player --ros-args -p route_file:=mi_ruta.json

Opciones:
  route_file:     Ruta al archivo JSON
  skip_every:     Usar solo 1 de cada N puntos (simplificar ruta)
  reverse:        Invertir la ruta (ir al revés)
  loop:           Repetir la ruta en bucle
  start_index:    Empezar desde el waypoint N
  end_index:      Terminar en el waypoint N (-1 = hasta el final)
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RoutePlayer(Node):
    def __init__(self):
        super().__init__("route_player")

        # Parámetros
        self.declare_parameter("route_file", "")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/route_markers")
        self.declare_parameter("skip_every", 1)   # 1 = todos, 5 = cada 5 puntos
        self.declare_parameter("reverse", False)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("end_index", -1)    # -1 = hasta el final
        self.declare_parameter("publish_hz", 1.0)  # re-publica la ruta a esta frecuencia
        self.declare_parameter("auto_start", True)  # enviar ruta al arrancar
        self.declare_parameter("map_frame", "map")

        self._route_file = self.get_parameter("route_file").value
        self._skip = max(1, self.get_parameter("skip_every").value)
        self._reverse = self.get_parameter("reverse").value
        self._loop = self.get_parameter("loop").value
        self._start_idx = self.get_parameter("start_index").value
        self._end_idx = self.get_parameter("end_index").value
        self._auto_start = self.get_parameter("auto_start").value
        self._map_frame = self.get_parameter("map_frame").value
        publish_hz = self.get_parameter("publish_hz").value

        # Publishers
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

        # TF (para mostrar distancia al coche)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Servicios
        self._srv_send = self.create_service(Empty, "send_route", self._on_send)
        self._srv_stop = self.create_service(Empty, "stop_route", self._on_stop)

        # Estado
        self._waypoints = []
        self._route_data = {}
        self._active = False

        # Cargar ruta
        if self._route_file:
            self._load_route(self._route_file)
        else:
            self.get_logger().warn(
                "⚠️ No se especificó route_file. Usa: "
                "--ros-args -p route_file:=/ruta/al/archivo.json"
            )

        # Timer para re-publicar
        self._pub_timer = self.create_timer(1.0 / publish_hz, self._republish)

        # Auto-start
        if self._auto_start and self._waypoints:
            self._active = True
            self._publish_all()
            self.get_logger().info("🚀 Ruta enviada automáticamente al arrancar.")

    def _load_route(self, filepath: str):
        """Carga la ruta desde un archivo JSON."""
        if not os.path.exists(filepath):
            self.get_logger().error(f"❌ Archivo no encontrado: {filepath}")
            return

        with open(filepath, "r") as f:
            data = json.load(f)

        self._route_data = data
        raw_wps = data.get("waypoints", [])

        # Aplicar filtros: rango de índices
        end = self._end_idx if self._end_idx >= 0 else len(raw_wps)
        raw_wps = raw_wps[self._start_idx:end]

        # Skip every N
        if self._skip > 1:
            filtered = raw_wps[::self._skip]
            # Siempre incluir el último punto
            if raw_wps and filtered[-1] != raw_wps[-1]:
                filtered.append(raw_wps[-1])
            raw_wps = filtered

        # Reverse
        if self._reverse:
            raw_wps = list(reversed(raw_wps))

        # Loop: añadir el primer punto al final para cerrar
        if self._loop and len(raw_wps) >= 2:
            raw_wps.append(raw_wps[0])

        self._waypoints = raw_wps
        route_name = data.get("route_name", os.path.basename(filepath))

        self.get_logger().info(
            f"📂 Ruta cargada: '{route_name}'\n"
            f"   📍 {len(self._waypoints)} waypoints "
            f"(de {data.get('total_points', '?')} originales, skip={self._skip})\n"
            f"   📏 {data.get('total_distance_m', '?')}m totales\n"
            f"   Servicios: send_route, stop_route"
        )

    def _on_send(self, _req, _resp):
        if not self._waypoints:
            self.get_logger().warn("⚠️ No hay ruta cargada.")
            return _resp
        self._active = True
        self._publish_all()
        self.get_logger().info(f"🚀 Ruta enviada: {len(self._waypoints)} waypoints")
        return _resp

    def _on_stop(self, _req, _resp):
        self._active = False
        self.get_logger().info("🛑 Ruta detenida. El Pure Pursuit dejará de recibir path.")
        return _resp

    def _publish_all(self):
        """Publica la ruta como Path y los markers."""
        self._publish_path()
        self._publish_markers()

    def _republish(self):
        """Re-publica la ruta periódicamente si está activa."""
        if self._active and self._waypoints:
            self._publish_path()

    def _publish_path(self):
        """Publica nav_msgs/Path para el Pure Pursuit."""
        if len(self._waypoints) < 2:
            return

        now = self.get_clock().now().to_msg()
        frame = self._route_data.get("frame", self._map_frame)

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = now

        for wp in self._waypoints:
            p = PoseStamped()
            p.header.frame_id = frame
            p.header.stamp = now
            p.pose.position.x = float(wp["x"])
            p.pose.position.y = float(wp["y"])
            p.pose.position.z = 0.0
            # Orientación desde theta si está disponible
            theta = wp.get("theta", 0.0)
            p.pose.orientation.z = math.sin(theta / 2.0)
            p.pose.orientation.w = math.cos(theta / 2.0)
            path.poses.append(p)

        self._path_pub.publish(path)

    def _publish_markers(self):
        """Publica markers para visualización en RViz."""
        if not self._waypoints:
            return

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame = self._route_data.get("frame", self._map_frame)

        # Línea conectando waypoints
        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = now
        line.ns = "route_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03  # grosor de la línea
        line.color.r = 0.2
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 0.9
        line.pose.orientation.w = 1.0

        from geometry_msgs.msg import Point
        for wp in self._waypoints:
            pt = Point()
            pt.x = float(wp["x"])
            pt.y = float(wp["y"])
            pt.z = 0.02
            line.points.append(pt)
        ma.markers.append(line)

        # Esferas en cada waypoint
        for i, wp in enumerate(self._waypoints):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = "route_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp["x"])
            m.pose.position.y = float(wp["y"])
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = m.scale.z = 0.08
            # Verde para puntos normales, rojo para inicio, azul para fin
            if i == 0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.2, 0.2  # Rojo = inicio
            elif i == len(self._waypoints) - 1:
                m.color.r, m.color.g, m.color.b = 0.2, 0.2, 1.0  # Azul = fin
            else:
                m.color.r, m.color.g, m.color.b = 0.2, 1.0, 0.3  # Verde
            m.color.a = 1.0
            ma.markers.append(m)

        # Flechas de dirección cada N puntos
        arrow_step = max(1, len(self._waypoints) // 15)
        for i in range(0, len(self._waypoints), arrow_step):
            wp = self._waypoints[i]
            if "theta" not in wp:
                continue
            a = Marker()
            a.header.frame_id = frame
            a.header.stamp = now
            a.ns = "route_arrows"
            a.id = i
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.position.x = float(wp["x"])
            a.pose.position.y = float(wp["y"])
            a.pose.position.z = 0.06
            theta = wp["theta"]
            a.pose.orientation.z = math.sin(theta / 2.0)
            a.pose.orientation.w = math.cos(theta / 2.0)
            a.scale.x = 0.15
            a.scale.y = 0.04
            a.scale.z = 0.04
            a.color.r = 1.0
            a.color.g = 0.9
            a.color.b = 0.0
            a.color.a = 0.8
            ma.markers.append(a)

        self._markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
