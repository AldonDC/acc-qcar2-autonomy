#!/usr/bin/env python3
"""
Sincroniza la pista (screenshot + homografía) con el mapa real de Cartographer (/map).
Lee una vez el topic /map, calcula el extent del mundo (xmin, xmax, ymin, ymax) y actualiza:
  - config/homography_calibration.yaml  (pixel -> mundo con las mismas referencias que /map)
  - config/circuit_extent_from_map.yaml (para que circuit_map_publisher use ese extent)

Así la imagen de la pista y las coordenadas en RViz coinciden con el mismo marco que QLabs/Cartographer.

Uso: con QLabs y Cartographer corriendo (y /map publicado), ejecuta una vez:
  ros2 run qcar2_autonomy sync_circuit_from_map_node
"""
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# Tamaño de pista_qcar2.png
IMAGE_W = 1152
IMAGE_H = 896


def _find_config_dir(node: Node) -> str:
    """Ruta al directorio config del paquete (instalado o source)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("qcar2_autonomy")
        d = os.path.join(pkg, "config")
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    for base in [os.getcwd(), os.path.join(os.getcwd(), "src", "qcar2_autonomy")]:
        d = os.path.join(base, "config")
        if os.path.isdir(d):
            return d
    return os.path.join(os.getcwd(), "config")


class SyncCircuitFromMapNode(Node):
    def __init__(self):
        super().__init__("sync_circuit_from_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("timeout_sec", 15.0)
        self._received = False
        self._timed_out = False
        timeout = self.get_parameter("timeout_sec").value
        self._timeout_timer = self.create_timer(float(timeout), self._on_timeout)
        self._sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            1,
        )
        self._timer = self.create_timer(1.0, self._check_done)

    def _on_map(self, msg: OccupancyGrid):
        if self._received:
            return
        self._received = True
        info = msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height
        res = info.resolution
        xmin = ox
        ymin = oy
        xmax = ox + w * res
        ymax = oy + h * res

        config_dir = _find_config_dir(self)
        homography_path = os.path.join(config_dir, "homography_calibration.yaml")
        extent_path = os.path.join(config_dir, "circuit_extent_from_map.yaml")

        # Homografía: imagen (0,0)=top-left -> mundo (xmin, ymax); (W,0)->(xmax,ymax); (W,H)->(xmax,ymin); (0,H)->(xmin,ymin)
        homography_data = {
            "image_file": "pista_qcar2.png",
            "points": [
                {"pixel": [0, 0], "world": [xmin, ymax]},
                {"pixel": [IMAGE_W, 0], "world": [xmax, ymax]},
                {"pixel": [IMAGE_W, IMAGE_H], "world": [xmax, ymin]},
                {"pixel": [0, IMAGE_H], "world": [xmin, ymin]},
            ],
        }
        try:
            import yaml
            os.makedirs(config_dir, exist_ok=True)
            with open(homography_path, "w", encoding="utf-8") as f:
                f.write("# Generado por sync_circuit_from_map_node desde /map (Cartographer).\n")
                f.write("# Pixel (pista_qcar2.png) -> mundo (frame map).\n")
                yaml.dump(homography_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            self.get_logger().info("Escrito: %s" % homography_path)
        except Exception as e:
            self.get_logger().error("No se pudo escribir homografía: %s" % e)

        # Parámetros para circuit_map_publisher (mismo extent que /map). Formato ROS2: /** o nombre del nodo.
        resolution_fine = (xmax - xmin) / IMAGE_W
        extent_data = {
            "/**": {
                "ros__parameters": {
                    "circuit_image_path": "pista_qcar2.png",
                    "use_fixed_extent": True,
                    "fixed_extent": [xmin, xmax, ymin, ymax],
                    "fixed_resolution": resolution_fine,
                }
            }
        }
        try:
            import yaml
            with open(extent_path, "w", encoding="utf-8") as f:
                f.write("# Generado por sync_circuit_from_map_node. Usar: --params-file config/circuit_extent_from_map.yaml\n")
                yaml.dump(extent_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            self.get_logger().info("Escrito: %s" % extent_path)
        except Exception as e:
            self.get_logger().error("No se pudo escribir extent: %s" % e)

        self.get_logger().info(
            "Extent desde /map: x=[%.3f, %.3f] m, y=[%.3f, %.3f] m (res=%.4f)"
            % (xmin, xmax, ymin, ymax, res)
        )
        self.get_logger().info(
            "Reinicia waypoint_rviz_launch o pasa --params-file %s al circuit_map_publisher para usar este extent."
            % extent_path
        )

    def _on_timeout(self):
        if not self._received and not self._timed_out:
            self._timed_out = True
            self.get_logger().warn("Timeout: no se recibió /map. ¿Está Cartographer publicando?")
            raise SystemExit(1)

    def _check_done(self):
        if self._received:
            self.get_logger().info("Sincronización lista. Saliendo.")
            raise SystemExit(0)


def main():
    rclpy.init()
    node = SyncCircuitFromMapNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
