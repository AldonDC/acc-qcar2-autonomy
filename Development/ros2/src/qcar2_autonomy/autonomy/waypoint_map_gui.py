#!/usr/bin/env python3
"""
Ventana en Python: fondo = circuito (imagen fija) o mapa de ocupación (/map).
Pon waypoints con clic y ves la trayectoria en verde; al enviar, el QCar sigue en QLabs.

Fondo circuito: imagen del circuito ACC (pista fija), no lo que ve el láser.
  - Pon una imagen (PNG/JPG) en config/pista_qcar2.png y parámetros extent.
  - O deja circuit_image_path vacío para usar el /map de Cartographer.
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import List, Tuple, Optional

import math

# #region agent log
def _debug_log_path():
    try:
        cwd = os.getcwd()
        if "ros2" in cwd or "isaac_ros-dev" in cwd:
            return os.path.join(cwd, ".cursor", "debug.log")
        base = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):
            cand = os.path.join(base, ".cursor", "debug.log")
            if base.endswith("ros2") or os.path.basename(base) == "Development":
                return cand
            parent = os.path.dirname(base)
            if parent == base:
                break
            base = parent
    except Exception:
        pass
    return os.path.join(os.path.expanduser("~"), ".cursor_acc_debug.log")

def _debug_log(location: str, message: str, data: dict, hypothesis_id: str) -> None:
    try:
        path = _debug_log_path()
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "a") as f:
            f.write(json.dumps({"id": f"log_{int(time.time()*1000)}", "timestamp": int(time.time() * 1000), "location": location, "message": message, "data": data, "hypothesisId": hypothesis_id}) + "\n")
    except Exception:
        pass
# #endregion
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import TransformException as TFException
    _TF_AVAILABLE = True
except ImportError:
    _TF_AVAILABLE = False

# Mapa en memoria (desde el callback en el otro hilo)
_map_data: Optional[bytes] = None
# Pose del robot en mapa (x, y, theta) para dibujar en el GUI
_robot_pose: Optional[Tuple[float, float, float]] = None
_map_info: Optional[dict] = None
_map_frame: Optional[str] = None


def _map_callback(msg: OccupancyGrid, node: Node) -> None:
    global _map_data, _map_info, _map_frame
    _map_data = bytes(msg.data)
    _map_info = {
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
    }
    _map_frame = msg.header.frame_id


def _pixel_to_world(px: int, py: int, info: dict) -> Tuple[float, float]:
    """Convierte pixel (col, row) del mapa a coordenadas mundo (x, y) en metros."""
    res = info["resolution"]
    ox = info["origin_x"]
    oy = info["origin_y"]
    h = info["height"]
    # En OccupancyGrid, fila 0 está abajo. En imagen (matplotlib) fila 0 arriba.
    world_x = ox + (px + 0.5) * res
    world_y = oy + (h - 1 - py + 0.5) * res
    return (world_x, world_y)


def _world_to_pixel(wx: float, wy: float, info: dict) -> Tuple[int, int]:
    res = info["resolution"]
    ox, oy = info["origin_x"], info["origin_y"]
    h, w = info["height"], info["width"]
    px = int((wx - ox) / res - 0.5)
    py = int(h - 1 - (wy - oy) / res + 0.5)
    return (max(0, min(px, w - 1)), max(0, min(py, h - 1)))


class WaypointMapGuiNode(Node):
    def __init__(self):
        super().__init__("waypoint_map_gui")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("markers_topic", "/waypoint_markers")
        self.declare_parameter("max_waypoints", 50)
        # Fondo = circuito (imagen fija). Vacío = usar /map (ocupación).
        self.declare_parameter("circuit_image_path", "pista_qcar2.png")
        self.declare_parameter("circuit_extent", [-7.35, 0.65, -1.49, 3.51]) # [xmin, xmax, ymin, ymax] coincidente con RViz
        self.declare_parameter("circuit_extent_from_map", False)  # Usar dimensiones fijas para evitar saltos al iniciar
        self.declare_parameter("circuit_rotation_deg", 0)  # 0 = orientación natural (roundabout arriba-izq)
        self.declare_parameter("pose_offset_x", 0.0)  # Ya no hace falta offset manual si el extent está bien
        self.declare_parameter("pose_offset_y", 0.0)
        self.declare_parameter("pose_offset_theta", 0.22)  # rad: orientación triángulo = coche en QLabs
        self.declare_parameter("display_flip_y", True)  # Y invertido para alinear con imagen del circuito
        self.declare_parameter("display_rot180", True)  # True = misma transformación que imagen (180°), para que pose = QLabs

        map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("waypoints_path_topic").value
        self.markers_topic = self.get_parameter("markers_topic").value
        self.max_waypoints = self.get_parameter("max_waypoints").value
        self.circuit_image_path = self.get_parameter("circuit_image_path").value or ""
        self.circuit_extent = list(self.get_parameter("circuit_extent").value)
        if len(self.circuit_extent) != 4:
            self.circuit_extent = [0.0, 10.0, 0.0, 10.0]
        self.circuit_rotation_deg = int(self.get_parameter("circuit_rotation_deg").value) % 360
        if self.circuit_rotation_deg not in (0, 90, 180, 270):
            self.circuit_rotation_deg = 180
        self.circuit_extent_from_map = self.get_parameter("circuit_extent_from_map").value
        self.pose_offset_x = float(self.get_parameter("pose_offset_x").value)
        self.pose_offset_y = float(self.get_parameter("pose_offset_y").value)
        self.pose_offset_theta = float(self.get_parameter("pose_offset_theta").value)
        self.display_flip_y = self.get_parameter("display_flip_y").value
        self.display_rot180 = self.get_parameter("display_rot180").value

        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            lambda msg: _map_callback(msg, self),
            10,
        )
        self.waypoints: List[Tuple[float, float]] = []
        self._robot_pose: Optional[Tuple[float, float, float]] = None
        self._robot_pose_raw: Optional[Tuple[float, float, float]] = None  # TF sin offsets (para calibración)
        self._tf_log_count = 0
        if _TF_AVAILABLE:
            self.declare_parameter("robot_frame", "base_link")
            self.declare_parameter("map_frame_tf", "map")  # "map_rotated" = misma convención que QLabs, luego se ajusta con display_flip_y
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._tf_timer = self.create_timer(0.033, self._update_robot_pose)  # ~30 Hz para posición más precisa en GUI
        else:
            self._tf_timer = None
        if self.circuit_image_path:
            self.get_logger().info(
                f"Fondo: circuito ({self.circuit_image_path}). Waypoints → {self.path_topic}. "
                "Clic en el circuito para waypoints; Enviar = ruta al coche (trayectoria en GUI y QLabs)."
            )
        else:
            self.get_logger().info(
                f"Fondo: /map (ocupación). Waypoints → {self.path_topic} y {self.markers_topic}. "
                "Abre la ventana y haz clic en el mapa."
            )

    def add_waypoint(self, world_x: float, world_y: float) -> bool:
        if len(self.waypoints) >= self.max_waypoints:
            return False
        self.waypoints.append((world_x, world_y))
        return True

    def clear_waypoints(self) -> None:
        self.waypoints.clear()

    def _update_robot_pose(self) -> None:
        global _robot_pose
        if not _TF_AVAILABLE:
            return
        try:
            map_f = self.get_parameter("map_frame_tf").value
            robot_f = self.get_parameter("robot_frame").value
            t = self._tf_buffer.lookup_transform(
                map_f, robot_f, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            raw_x = t.transform.translation.x
            raw_y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            raw_theta = math.atan2(siny, cosy)
            self._robot_pose_raw = (raw_x, raw_y, raw_theta)
            x = raw_x + self.pose_offset_x
            y = raw_y + self.pose_offset_y
            theta = raw_theta + self.pose_offset_theta
            self._robot_pose = (x, y, theta)
            _robot_pose = self._robot_pose
            # #region agent log
            self._tf_log_count += 1
            if self._tf_log_count <= 1 or self._tf_log_count % 10 == 1:
                _debug_log("waypoint_map_gui.py:_update_robot_pose", "TF pose", {
                    "map_frame_tf": map_f, "robot_frame": robot_f,
                    "raw_x": round(raw_x, 4), "raw_y": round(raw_y, 4), "raw_theta": round(raw_theta, 4),
                    "pose_offset_x": self.pose_offset_x, "pose_offset_y": self.pose_offset_y, "pose_offset_theta": self.pose_offset_theta,
                    "final_x": round(x, 4), "final_y": round(y, 4), "final_theta": round(theta, 4),
                }, "H1_H3_H5")
            # #endregion
        except Exception as e:
            _robot_pose = None
            self._robot_pose_raw = None
            # #region agent log
            _debug_log("waypoint_map_gui.py:_update_robot_pose", "TF lookup failed", {"error": str(e)[:200]}, "H3")
            # #endregion

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        return getattr(self, "_robot_pose", None) or _robot_pose

    def publish_path(self) -> int:
        if len(self.waypoints) < 2:
            return 0
        now = self.get_clock().now().to_msg()
        frame = _map_frame if _map_frame else "map"
        if self.circuit_image_path:
            frame = frame or "map"
        path = PathMsg()
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
        n = len(self.waypoints)
        self.get_logger().info(f"Ruta enviada: {n} waypoints. (Si el coche no se mueve, asegúrate de tener Terminal 3 activo con waypoint_visual_launch.py)")
        return n


def _run_ros_spin(node: WaypointMapGuiNode) -> None:
    rclpy.spin(node)


def _run_terminal_fallback(node: WaypointMapGuiNode) -> None:
    """Sin matplotlib: modo terminal. Escribe 'x y' para añadir waypoint, 'enviar' para publicar, 'borrar' para limpiar."""
    t = threading.Thread(target=_run_ros_spin, args=(node,), daemon=True)
    t.start()
    print("Modo terminal. Comandos: 'x y' = añadir waypoint (ej: 1.5 0.3), 'enviar' = mandar ruta, 'borrar' = limpiar, 'salir' = terminar.")
    try:
        while rclpy.ok():
            line = input(">>> ").strip().lower()
            if not line:
                continue
            if line in ("salir", "quit", "q"):
                break
            if line in ("enviar", "send", "s", "go"):
                n = node.publish_path()
                print(f"  Enviado: {n} waypoints." if n else "  Necesitas al menos 2 waypoints.")
                continue
            if line in ("borrar", "clear", "c"):
                node.clear_waypoints()
                print("  Waypoints borrados.")
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    if node.add_waypoint(x, y):
                        print(f"  Waypoint {len(node.waypoints)}: ({x}, {y})")
                    else:
                        print(f"  Máximo {node.max_waypoints} waypoints. Usa 'borrar' primero.")
                except ValueError:
                    print("  Escribe dos números: x y (ej: 1.0 0.5)")
            else:
                print("  Comandos: x y | enviar | borrar | salir")
    except (EOFError, KeyboardInterrupt):
        pass


def _run_gui(node: WaypointMapGuiNode) -> None:
    global _map_data, _map_info
    # Sin DISPLAY la ventana no puede abrirse (contenedor sin acceso a la X del host)
    if not os.environ.get("DISPLAY"):
        print(
            "DISPLAY no está configurado → no se puede abrir la ventana.\n"
            "  En el HOST (fuera del contenedor) ejecuta:  xhost +local:\n"
            "  Entra al contenedor de nuevo. Si usas run_dev.sh, el script suele exportar DISPLAY.\n"
            "Usando modo terminal (escribe x y y 'enviar')."
        )
        _run_terminal_fallback(node)
        return
    # En contenedores suele faltar tkinter: sin ventana, usamos modo terminal
    try:
        import tkinter  # noqa: F401
    except ModuleNotFoundError:
        print("Sin tkinter en el contenedor → modo terminal (escribe coordenadas o usa RViz/YAML).")
        _run_terminal_fallback(node)
        return
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button
    except ImportError:
        print("Instala matplotlib: pip install matplotlib. Usando modo terminal.")
        _run_terminal_fallback(node)
        return

    try:
        fig, ax = plt.subplots(figsize=(9.5, 6.5))
    except Exception as e:
        err = str(e).lower()
        if "display" in err or "tk" in err or "x11" in err or "connect" in err:
            print(
                "No se pudo abrir la ventana (display/ventana no disponible).\n"
                "  En el HOST ejecuta:  xhost +local:\n"
                "  Asegúrate de entrar al contenedor con: ./isaac_ros_common/scripts/run_dev.sh ./Development -- bash\n"
                "  (run_dev.sh suele pasar DISPLAY al contenedor).\n"
                "Usando modo terminal."
            )
            _run_terminal_fallback(node)
            return
        raise
    try:
        fig.canvas.manager.set_window_title("Circuito ACC – Waypoints | Enviar ruta al coche (QLabs)")
    except Exception:
        pass
    ax.set_aspect("equal")
    try:
        fig.tight_layout()
    except Exception:
        pass
    points_line, = ax.plot([], [], "go-", markersize=14, linewidth=3.5, label="Trayectoria", zorder=10)
    robot_line, = ax.plot([], [], "r-", linewidth=2.5, label="QCar (posición en tiempo real)", zorder=15)
    pos_text = ax.text(0.02, 0.98, "Pos: --", transform=ax.transAxes, fontsize=9, verticalalignment="top", color="darkred", bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.8))
    calib_text = ax.text(0.02, 0.82, "", transform=ax.transAxes, fontsize=8, verticalalignment="top", color="green", bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.9), wrap=True)
    use_circuit = bool(node.circuit_image_path)
    calibration_mode = [False]  # list para poder modificar desde on_click
    circuit_image_artist = [None]  # imagen del circuito para capturar clic con pick_event
    circuit_drawn = False
    drawn = False
    waypoints_xy: List[Tuple[float, float]] = []
    _draw_log_count = [0]

    def _resolve_circuit_path():
        p = (node.circuit_image_path or "").strip()
        if not p:
            return ""
        if os.path.isabs(p) and os.path.isfile(p):
            return p
        if os.path.isfile(p):
            return p
        cwd = os.getcwd()
        bases = [cwd]
        # Workspace src (cuando ejecutas desde ros2): src/qcar2_autonomy/config/
        if cwd.rstrip("/").endswith("ros2"):
            bases.append(os.path.join(cwd, "src", "qcar2_autonomy", "config"))
        bases.append(os.path.join(cwd, "config"))
        try:
            from ament_index_python.packages import get_package_share_directory
            bases.append(os.path.join(get_package_share_directory("qcar2_autonomy"), "config"))
        except Exception:
            pass
        bases.append(os.path.join(os.path.dirname(__file__), "..", "config"))
        for base in bases:
            candidate = os.path.join(base, p) if not os.path.isabs(p) else p
            if os.path.isfile(candidate):
                return candidate
        return ""

    def draw_robot(pose):
        """Dibuja triángulo del QCar en (x, y) con orientación theta."""
        if pose is None:
            robot_line.set_data([], [])
            pos_text.set_text("Pos: --\nTF: sin conexión (revisa T2 + QLabs)")
            return
        x, y, theta = pose
        draw_flip_y = getattr(node, "display_flip_y", True)
        draw_rot180 = getattr(node, "display_rot180", False)
        if draw_flip_y:
            y = -y
            theta = -theta
        if draw_rot180:
            x, y = -x, -y
            theta = theta + math.pi
        drawn_x, drawn_y, drawn_theta = x, y, theta
        # #region agent log
        _draw_log_count[0] += 1
        if _draw_log_count[0] <= 1 or _draw_log_count[0] % 7 == 1:
            _debug_log("waypoint_map_gui.py:draw_robot", "Draw pose", {
                "pose_in": [round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)],
                "display_flip_y": draw_flip_y, "display_rot180": draw_rot180,
                "drawn_x": round(drawn_x, 4), "drawn_y": round(drawn_y, 4), "drawn_theta": round(drawn_theta, 4),
            }, "H2")
        # #endregion
        L = 0.12
        W = 0.08
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        xf = x + L * cos_t
        yf = y + L * sin_t
        xbl = x - L * cos_t + W * sin_t
        ybl = y - L * sin_t - W * cos_t
        xbr = x - L * cos_t - W * sin_t
        ybr = y - L * sin_t + W * cos_t
        robot_line.set_data([xf, xbl, xbr, xf], [yf, ybl, ybr, yf])
        raw = getattr(node, "_robot_pose_raw", None)
        if raw is not None:
            pos_text.set_text(f"Pos: x={drawn_x:.3f} y={drawn_y:.3f}\nTF: OK (raw x={raw[0]:.3f} y={raw[1]:.3f})")
        else:
            pos_text.set_text(f"Pos: x={drawn_x:.3f} y={drawn_y:.3f}")

    def redraw_points():
        nonlocal waypoints_xy
        waypoints_xy = list(node.waypoints)
        if waypoints_xy:
            xs, ys = zip(*waypoints_xy)
            points_line.set_data(xs, ys)
        else:
            points_line.set_data([], [])
        draw_robot(node.get_robot_pose())
        n = len(node.waypoints)
        if use_circuit:
            ax.set_title(f"{n} waypoints | Clic = añadir | Enviar = ruta (GUI + QLabs)")
        else:
            ax.set_title(f"Waypoints: {n} (clic = añadir, Enviar = mandar ruta)")
        ax.figure.canvas.draw_idle()

    def on_send(_event):
        n = node.publish_path()
        if n > 0:
            ax.set_title(f"Ruta enviada ({n} pts). T3 = waypoint_visual_launch para que el coche se mueva.")

    def on_clear(_event):
        node.clear_waypoints()
        redraw_points()
        ax.set_title("Waypoints borrados. Clic en el circuito para añadir.")

    ax_btn_send = plt.axes([0.12, 0.02, 0.22, 0.05])
    ax_btn_clear = plt.axes([0.38, 0.02, 0.22, 0.05])
    ax_btn_calib = plt.axes([0.66, 0.02, 0.28, 0.05])
    button_axes = (ax_btn_send, ax_btn_clear, ax_btn_calib)
    btn_send = Button(ax_btn_send, "Enviar ruta al coche")
    btn_clear = Button(ax_btn_clear, "Borrar")
    btn_calib = Button(ax_btn_calib, "Calibrar: clic en el mapa de esta ventana")
    btn_send.on_clicked(on_send)
    btn_clear.on_clicked(on_clear)

    def on_calib(_event):
        calibration_mode[0] = True
        calib_text.set_text(
            "Haz clic en el MAPA DE ESTA VENTANA\n"
            "(la pista 2D de arriba), en el punto donde\n"
            "está el coche. NO cliques en la ventana de QLabs."
        )
        ax.set_title("Calibración: clic en el MAPA de esta ventana (pista 2D), donde está el coche. No en QLabs.")
        ax.figure.canvas.draw_idle()
    btn_calib.on_clicked(on_calib)

    def process_map_click(wx, wy):
        """Procesa un clic en el mapa: calibración o añadir waypoint. Devuelve True si se procesó."""
        if not (use_circuit and circuit_drawn):
            global _map_info
            if _map_info is None:
                return False
        raw = getattr(node, "_robot_pose_raw", None)
        if calibration_mode[0]:
            if raw is None:
                calib_text.set_text("TF sin conexión.\nComprueba T2 y QLabs.\nClic en el MAPA de esta ventana (no en QLabs).")
                ax.set_title("Calibración: sin TF. Luego clic en el mapa de ESTA ventana.")
                ax.figure.canvas.draw_idle()
                return True
            ox = -float(wx) - raw[0]
            oy = float(wy) - raw[1]
            calibration_mode[0] = False
            calib_text.set_text(f"Clic en ({wx:.2f}, {wy:.2f})\nReinicia el GUI con:\n--ros-args -p pose_offset_x:={ox:.3f} -p pose_offset_y:={oy:.3f}")
            ax.set_title("Offsets calculados. Reinicia el GUI con los parámetros indicados (ver recuadro verde).")
            ax.figure.canvas.draw_idle()
            return True
        if node.add_waypoint(float(wx), float(wy)):
            redraw_points()
            return True
        return False

    def on_click(event):
        if event.button != 1:
            return
        if event.inaxes in button_axes:
            return
        wx, wy = None, None
        if event.inaxes == ax and event.xdata is not None and event.ydata is not None:
            wx, wy = event.xdata, event.ydata
        else:
            try:
                bbox = ax.bbox
                if bbox is not None and getattr(bbox, "width", 0) > 0 and getattr(bbox, "height", 0) > 0:
                    xy = (event.x, event.y)
                    if bbox.contains(xy[0], xy[1]):
                        inv = ax.transData.inverted()
                        pt = inv.transform(xy)
                        wx, wy = float(pt[0]), float(pt[1])
            except Exception:
                pass
        if wx is None or wy is None:
            return
        process_map_click(wx, wy)

    def on_pick(event):
        """Clic directo sobre la imagen del circuito (alternativa cuando button_press no llega)."""
        if event.mouseevent.button != 1:
            return
        if circuit_image_artist[0] is None or event.artist != circuit_image_artist[0]:
            return
        try:
            inv = ax.transData.inverted()
            pt = inv.transform((event.mouseevent.x, event.mouseevent.y))
            wx, wy = float(pt[0]), float(pt[1])
            process_map_click(wx, wy)
        except Exception:
            pass

    fig.canvas.mpl_connect("button_press_event", on_click)
    # No llamar tight_layout aquí: los ejes de los botones no son compatibles y puede provocar warning/crash al cerrar
    try:
        fig.subplots_adjust(bottom=0.12, left=0.08, right=0.96, top=0.94)
    except Exception:
        pass

    # Fondo = circuito (imagen fija del circuito ACC), no el mapa de ocupación
    circuit_path = _resolve_circuit_path()
    if use_circuit and circuit_path:
        try:
            img = plt.imread(circuit_path)
            # Mismas dimensiones que el mapa del QCar: usar /map si está disponible
            ex = list(node.circuit_extent)
            if node.circuit_extent_from_map and _map_info:
                w, h = _map_info["width"], _map_info["height"]
                res = _map_info["resolution"]
                ox, oy = _map_info["origin_x"], _map_info["origin_y"]
                ex = [ox, ox + w * res, oy, oy + h * res]
            rot = node.circuit_rotation_deg
            try:
                import numpy as np
                if rot == 90:
                    img = np.rot90(img, 1)
                elif rot == 180:
                    img = np.rot90(img, 2)
                elif rot == 270:
                    img = np.rot90(img, 3)
            except Exception:
                pass
            # #region agent log
            _debug_log("waypoint_map_gui.py:circuit_extent", "Circuit extent used for plot", {
                "extent": ex, "circuit_extent_from_map": node.circuit_extent_from_map,
                "map_frame": _map_frame if _map_frame else "none",
            }, "H4")
            # #endregion
            im = ax.imshow(img, extent=[ex[0], ex[1], ex[2], ex[3]], origin="lower", aspect="equal", zorder=0)
            ax.set_xlim(ex[0], ex[1])
            ax.set_ylim(ex[2], ex[3])
            if node.circuit_extent_from_map and _map_info:
                ax._circuit_extent_synced = True
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            circuit_drawn = True
            circuit_image_artist[0] = im
            try:
                im.set_picker(5)
                fig.canvas.mpl_connect("pick_event", on_pick)
            except Exception:
                pass
            redraw_points()
            ax.set_title("Clic = waypoint | Enviar = ruta al coche (tray. en GUI y QLabs)")
        except Exception as e:
            ax.set_title(f"Error cargando circuito: {e}. Usa circuit_image_path y circuit_extent.")
    elif use_circuit:
        # Ruta donde poner la captura del circuito (preferir src/ para que quede en el repo)
        cwd = os.getcwd()
        if cwd.rstrip("/").endswith("ros2"):
            path_instruccion = os.path.join(cwd, "src", "qcar2_autonomy", "config", "pista_qcar2.png")
        else:
            try:
                from ament_index_python.packages import get_package_share_directory
                path_instruccion = os.path.join(get_package_share_directory("qcar2_autonomy"), "config", "pista_qcar2.png")
            except Exception:
                path_instruccion = os.path.join(cwd, "config", "pista_qcar2.png")
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect("equal")
        ax.text(5, 6, "Pista de fondo = captura del circuito QLabs", ha="center", fontsize=12, wrap=True)
        ax.text(5, 4.5, "Guarda la imagen con nombre:", ha="center", fontsize=10)
        ax.text(5, 3.5, "pista_qcar2.png", ha="center", fontsize=11, style="italic")
        ax.text(5, 2.5, "en esta carpeta:", ha="center", fontsize=10)
        ax.text(5, 1.2, path_instruccion, ha="center", fontsize=8, wrap=True, family="monospace")
        ax.set_title("Circuito ACC – Sin imagen de pista. Añade pista_qcar2.png en config/ y reinicia el GUI.")

    ros_thread = threading.Thread(target=_run_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    while True:
        try:
            if not plt.fignum_exists(fig.number):
                break
        except Exception:
            break
        try:
            plt.pause(0.15)
            draw_robot(node.get_robot_pose())
            # Sincronizar dimensiones del circuito con /map cuando llegue (mismo tamaño que QCar)
            if use_circuit and circuit_drawn and node.circuit_extent_from_map and _map_info and not getattr(ax, "_circuit_extent_synced", False):
                w, h = _map_info["width"], _map_info["height"]
                res = _map_info["resolution"]
                ox, oy = _map_info["origin_x"], _map_info["origin_y"]
                ex = [ox, ox + w * res, oy, oy + h * res]
                if ax.images:
                    ax.images[0].set_extent(ex)
                ax.set_xlim(ex[0], ex[1])
                ax.set_ylim(ex[2], ex[3])
                ax._circuit_extent_synced = True
            ax.figure.canvas.draw_idle()
        except Exception:
            break
        if not (use_circuit and circuit_drawn) and _map_data is not None and _map_info is not None and not drawn:
            w, h = _map_info["width"], _map_info["height"]
            res = _map_info["resolution"]
            ox, oy = _map_info["origin_x"], _map_info["origin_y"]
            try:
                import numpy as np
                grid = np.array(list(_map_data), dtype=np.int8).reshape((h, w))
            except ImportError:
                grid = [list(_map_data)[i * w:(i + 1) * w] for i in range(h)]
            # -1 = unknown -> gris; 0 = libre -> blanco; 100 = ocupado -> negro
            try:
                import numpy as np
                rgb = np.ones((h, w, 3))
                rgb[grid == -1] = [0.7, 0.7, 0.7]
                rgb[grid == 0] = [1, 1, 1]
                rgb[grid == 100] = [0.2, 0.2, 0.2]
            except (NameError, TypeError):
                rgb = [[[0.7, 0.7, 0.7] if grid[i][j] == -1 else [1, 1, 1] if grid[i][j] == 0 else [0.2, 0.2, 0.2] for j in range(w)] for i in range(h)]
            ax.imshow(
                rgb,
                extent=[ox, ox + w * res, oy, oy + h * res],
                origin="lower",
                interpolation="nearest",
            )
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title("Mapa QLabs – Clic = waypoint | Enviar = mandar ruta al coche")
            drawn = True
            redraw_points()

    try:
        plt.close(fig)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMapGuiNode()
    try:
        _run_gui(node)
    except KeyboardInterrupt:
        pass
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
