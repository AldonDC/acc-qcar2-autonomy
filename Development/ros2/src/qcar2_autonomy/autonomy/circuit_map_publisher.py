#!/usr/bin/env python3
"""
Publica la imagen de la pista (pista_qcar2.png) como OccupancyGrid en /map_circuit
con las mismas dimensiones y frame que /map. RViz muestra este mapa como fondo;
las coordenadas coinciden con el mapa real para waypoints y TF.
"""
import os
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


def _find_package_path(node: Node, filename: str) -> str:
    """Ruta absoluta a filename en config del paquete o cwd."""
    p = (node.get_parameter("circuit_image_path").value or "").strip()
    if not p:
        return ""
    if os.path.isabs(p) and os.path.isfile(p):
        return p
    if os.path.isfile(p):
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        d = get_package_share_directory("qcar2_autonomy")
        candidate = os.path.join(d, "config", os.path.basename(p))
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    cwd = os.getcwd()
    for base in [cwd, os.path.join(cwd, "src", "qcar2_autonomy", "config")]:
        candidate = os.path.join(base, p) if os.path.isabs(p) else os.path.join(base, os.path.basename(p))
        if os.path.isfile(candidate):
            return candidate
    return p if os.path.isfile(p) else ""


class CircuitMapPublisher(Node):
    def __init__(self):
        super().__init__("circuit_map_publisher")
        self.declare_parameter("circuit_image_path", "pista_qcar2.png")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_circuit_topic", "/map_circuit")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("circuit_rotation_deg", 0.0) # Soporta cualquier ángulo
        self.declare_parameter("circuit_offset_x", 0.0)
        self.declare_parameter("circuit_offset_y", 0.0)
        
        # Extent fijo [xmin, xmax, ymin, ymax]
        # RECOMENDACIÓN: El ancho/alto debe coincidir con el aspect ratio de la imagen (1.286)
        # Ejemplo: Ancho 8.2m -> Alto 6.38m
        self.declare_parameter("use_fixed_extent", True)
        self.declare_parameter("fixed_extent", [-4.1, 4.1, -1.0, 5.38]) 
        self.declare_parameter("fixed_resolution", 0.05)

        self._map_info = None
        self._grid_data = None
        self._frame_id = "map"

        self._sub_map = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            1,
        )
        self._pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("map_circuit_topic").value,
            1,
        )
        
        if self.get_parameter("use_fixed_extent").value:
            self._load_fixed_map()

        rate = self.get_parameter("publish_rate").value
        self._timer = self.create_timer(1.0 / rate if rate > 0 else 1.0, self._publish)

    def _load_fixed_map(self):
        ex = self.get_parameter("fixed_extent").value
        res = self.get_parameter("fixed_resolution").value
        off_x = self.get_parameter("circuit_offset_x").value
        off_y = self.get_parameter("circuit_offset_y").value

        w = int((ex[1] - ex[0]) / res)
        h = int((ex[3] - ex[2]) / res)
        self._map_info = {
            "width": w, "height": h, "resolution": res,
            "origin_x": ex[0] + off_x, "origin_y": ex[2] + off_y,
        }
        self._process_image()

    def _on_map(self, msg: OccupancyGrid):
        self._frame_id = msg.header.frame_id
        if self.get_parameter("use_fixed_extent").value:
            return
            
        # Modo dinámico (ajustado a /map)
        self._map_info = {
            "width": int(msg.info.width), "height": int(msg.info.height), "resolution": msg.info.resolution,
            "origin_x": msg.info.origin.position.x, "origin_y": msg.info.origin.position.y,
        }
        self._process_image()

    def _process_image(self):
        if self._map_info is None:
            return
        w, h = self._map_info["width"], self._map_info["height"]
        path = _find_package_path(self, self.get_parameter("circuit_image_path").value)
        
        if not path or not os.path.isfile(path):
            self.get_logger().error(f"Imagen no encontrada: {path}")
            self._grid_data = [0] * (w * h)
            return

        try:
            from PIL import Image
            import numpy as np
            
            # 1. Cargar con PIL para rotación arbitraria
            img = Image.open(path).convert("L") # Escala de grises
            
            # 2. Rotación
            rot = self.get_parameter("circuit_rotation_deg").value
            if rot != 0:
                # expand=True evita que se corte, pero cambia el tamaño.
                # Para mapas ROS, mejor False y centrar bien el extent.
                img = img.rotate(rot, resample=Image.BICUBIC, expand=False)
            
            # 3. Redimensionar al tamaño del grid de ROS
            img = img.resize((w, h), Image.LANCZOS)
            
            # 4. Convertir a OccupancyGrid (-1=unknown, 0=free, 100=occupied)
            # ROS Grid se lee de abajo a arriba, PIL de arriba a abajo.
            arr = np.array(img)
            arr = np.flipud(arr) # Voltear verticalmente
            
            # Binarizar: Blanco (255) -> Libre (0), Negro (0) -> Desconocido o Pared (-1)
            # En este caso usamos -1 para que el mapa SLAM se vea encima sin problemas.
            data = np.where(arr > 128, 0, -1).astype(np.int8)
            
            self._grid_data = data.flatten().tolist()
            self.get_logger().info(f"Pista alineada: {w}x{h}, rot={rot}°, offset=({self.get_parameter('circuit_offset_x').value}, {self.get_parameter('circuit_offset_y').value})")
            
        except Exception as e:
            self.get_logger().error(f"Error PIL: {e}")
            self._grid_data = [0] * (w * h)

    def _publish(self):
        if self._map_info is None or self._grid_data is None:
            return
        msg = OccupancyGrid()
        msg.header = Header(frame_id=self._frame_id, stamp=self.get_clock().now().to_msg())
        msg.info.resolution = float(self._map_info["resolution"])
        msg.info.width = int(self._map_info["width"])
        msg.info.height = int(self._map_info["height"])
        msg.info.origin.position.x = float(self._map_info["origin_x"])
        msg.info.origin.position.y = float(self._map_info["origin_y"])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self._grid_data
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()

