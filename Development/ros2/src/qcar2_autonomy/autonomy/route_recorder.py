#!/usr/bin/env python3
"""
Route Recorder — Graba la ruta del QCar mientras lo conduces con teleop.
Lee la pose desde TF (map -> base_link) y guarda waypoints cada X metros
en un archivo JSON listo para reproducir con route_player.

Uso:
  ros2 run qcar2_autonomy route_recorder --ros-args -p output_file:=mi_ruta.json
  (Conduce el coche con teleop en otra terminal)
  Ctrl+C para parar y guardar.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__("route_recorder")

        # Parámetros
        self.declare_parameter("output_file", "recorded_route.json")
        self.declare_parameter("route_name", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("min_distance", 0.10)   # metros entre puntos
        self.declare_parameter("min_angle", 0.15)       # radianes entre puntos (si gira mucho)
        self.declare_parameter("sample_hz", 20.0)       # frecuencia de muestreo

        self._output_file = self.get_parameter("output_file").value
        self._route_name = self.get_parameter("route_name").value
        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._min_dist = self.get_parameter("min_distance").value
        self._min_angle = self.get_parameter("min_angle").value
        sample_hz = self.get_parameter("sample_hz").value

        if not self._route_name:
            self._route_name = os.path.splitext(os.path.basename(self._output_file))[0]

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Datos grabados
        self._waypoints = []
        self._last_x = None
        self._last_y = None
        self._last_theta = None
        self._start_time = time.time()
        self._recording = True

        # Timer de muestreo
        self._timer = self.create_timer(1.0 / sample_hz, self._sample)

        self.get_logger().info(
            f"🎥 Route Recorder iniciado. Grabando ruta '{self._route_name}' "
            f"cada {self._min_dist}m. Conduce el coche y pulsa Ctrl+C para guardar."
        )

    def _sample(self):
        if not self._recording:
            return

        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        theta = quaternion_to_yaw(t.transform.rotation)

        # ¿Es suficientemente diferente del último punto?
        if self._last_x is not None:
            dist = math.hypot(x - self._last_x, y - self._last_y)
            angle_diff = abs(theta - self._last_theta)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            if dist < self._min_dist and angle_diff < self._min_angle:
                return

        # Guardar waypoint
        wp = {
            "id": len(self._waypoints),
            "x": round(x, 4),
            "y": round(y, 4),
            "theta": round(theta, 4),
            "time": round(time.time() - self._start_time, 3),
        }
        self._waypoints.append(wp)
        self._last_x = x
        self._last_y = y
        self._last_theta = theta

        if len(self._waypoints) % 10 == 0:
            self.get_logger().info(
                f"📍 {len(self._waypoints)} puntos grabados "
                f"(último: x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.0f}°)"
            )

    def save(self):
        """Guarda la ruta grabada en un archivo JSON."""
        self._recording = False

        if len(self._waypoints) < 2:
            self.get_logger().warn("⚠️ Menos de 2 puntos grabados. No se guarda nada.")
            return

        # Calcular distancia total
        total_dist = 0.0
        for i in range(1, len(self._waypoints)):
            dx = self._waypoints[i]["x"] - self._waypoints[i - 1]["x"]
            dy = self._waypoints[i]["y"] - self._waypoints[i - 1]["y"]
            total_dist += math.hypot(dx, dy)

        route_data = {
            "route_name": self._route_name,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "frame": self._map_frame,
            "total_points": len(self._waypoints),
            "total_distance_m": round(total_dist, 3),
            "recording_duration_s": round(time.time() - self._start_time, 1),
            "min_sample_distance_m": self._min_dist,
            "waypoints": self._waypoints,
        }

        # Asegurar que el directorio existe
        output_dir = os.path.dirname(self._output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        with open(self._output_file, "w") as f:
            json.dump(route_data, f, indent=2)

        self.get_logger().info(
            f"✅ Ruta guardada: {self._output_file}\n"
            f"   📍 {len(self._waypoints)} waypoints\n"
            f"   📏 {total_dist:.2f} metros\n"
            f"   ⏱️  {route_data['recording_duration_s']}s de grabación"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
