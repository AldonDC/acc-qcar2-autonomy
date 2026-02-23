#!/usr/bin/env python3
"""
Dashboard GUI: pista de QLabs (imagen estática o vista en vivo desde QLabs) + posición del QCar
+ ruta + gráficas de comportamiento. Modo "vista en vivo": conecta a QLabs, usa una cámara
cenital (FreeCamera.get_image) como "screen mirror" sobre el que se dibuja el overlay (robot, ruta).
"""
from __future__ import annotations

import math
import os
import threading
import time
from collections import deque
from typing import Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import TransformException as TFException
    _TF_AVAILABLE = True
except ImportError:
    _TF_AVAILABLE = False

# Vista en vivo desde QLabs (opcional; requiere qvl en el mismo proceso)
_QLABS_VIEW_AVAILABLE = False
try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.free_camera import QLabsFreeCamera
    _QLABS_VIEW_AVAILABLE = True
except ImportError:
    pass


def _find_package_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("qcar2_autonomy"), "config")
    except Exception:
        pass
    for base in [os.getcwd(), os.path.join(os.getcwd(), "src", "qcar2_autonomy")]:
        d = os.path.join(base, "config")
        if os.path.isdir(d):
            return d
    return os.path.join(os.getcwd(), "config")


def _load_extent_from_yaml():
    """Carga fixed_extent desde circuit_extent_from_map.yaml si existe."""
    config_dir = _find_package_config()
    yaml_path = os.path.join(config_dir, "circuit_extent_from_map.yaml")
    if not os.path.isfile(yaml_path):
        return None
    try:
        import yaml
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        params = (data or {}).get("/**", {}).get("ros__parameters", {})
        return params.get("fixed_extent")
    except Exception:
        return None


def _resolve_circuit_image_path(circuit_image_path: str) -> str:
    if not circuit_image_path:
        return ""
    if os.path.isabs(circuit_image_path) and os.path.isfile(circuit_image_path):
        return circuit_image_path
    config_dir = _find_package_config()
    candidate = os.path.join(config_dir, os.path.basename(circuit_image_path))
    if os.path.isfile(candidate):
        return candidate
    if os.path.isfile(circuit_image_path):
        return circuit_image_path
    return ""


class QCarDashboardGuiNode(Node):
    def __init__(self):
        super().__init__("qcar_dashboard_gui")
        self.declare_parameter("circuit_image_path", "pista_qcar2.png")
        self.declare_parameter("circuit_extent", [-4.1, 4.1, -1.0, 5.38])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("path_topic", "/waypoints_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("use_pose_gt_topic", True)  # True = usar /qcar_pose_gt si está (sin depender de TF)
        self.declare_parameter("pose_gt_topic", "/qcar_pose_gt")
        self.declare_parameter("history_sec", 15.0)
        self.declare_parameter("update_hz", 30.0)  # Frecuencia de redibujado del GUI (más alto = menos lag)
        self.declare_parameter("qlabs_capture_hz", 25.0)  # Frecuencia de captura de imagen QLabs (vista 2D)
        # Vista 2D directa desde QLabs (FreeCamera.get_image): mismo template que la ventana QLabs
        self.declare_parameter("use_qlabs_view", True)  # True = coger imagen/screen directo de QLabs en 2D
        self.declare_parameter("qlabs_host", "localhost")
        self.declare_parameter("qlabs_camera_location", [0.15, 1.7, 5.0])  # cenital en coords QLabs full-scale
        self.declare_parameter("qlabs_camera_rotation_deg", [0.0, 90.0, 0.0])  # 90° = vista 2D top-down
        self.declare_parameter("qlabs_camera_fov_deg", 60.0)
        self.declare_parameter("qlabs_image_width", 1280)
        self.declare_parameter("qlabs_image_height", 720)
        # scale_to_meters: MISMO factor que usa qcar_pose_from_qlabs_node (0.1 por defecto).
        # Las coordenadas de la cámara están en QLabs full-scale; el pose_gt se publica en
        # metros (QLabs × scale_to_meters). El overlay debe operar en el mismo frame que pose_gt.
        self.declare_parameter("scale_to_meters", 0.1)
        # Centro del overlay en frame map (metros). 0.0 = auto-calcular desde camera_location × scale_to_meters.
        self.declare_parameter("qlabs_overlay_center_x", 0.0)
        self.declare_parameter("qlabs_overlay_center_y", 0.0)
        self.declare_parameter("qlabs_overlay_meters_per_pixel", 0.0)  # 0 = auto desde altura+FOV+scale
        self.declare_parameter("qlabs_flip_vertical", True)   # True = invertir imagen para que coincida con QLabs
        self.declare_parameter("qlabs_flip_horizontal", False)

        extent = _load_extent_from_yaml()
        if extent is not None and len(extent) == 4:
            self._extent = list(extent)
            self.get_logger().info("Extent desde circuit_extent_from_map.yaml")
        else:
            self._extent = list(self.get_parameter("circuit_extent").value)
        self._circuit_image_path = _resolve_circuit_image_path(
            self.get_parameter("circuit_image_path").value
        )

        self._pose: Optional[Tuple[float, float, float]] = None
        self._pose_from_gt: Optional[Tuple[float, float, float]] = None
        self._path_points: List[Tuple[float, float]] = []
        self._vel_history: deque = deque(maxlen=500)
        self._path_header_frame = ""
        # Waypoints por clic en el mapa (interactivo)
        self._clicked_waypoints: List[Tuple[float, float]] = []
        self._clicked_waypoints_lock = threading.Lock()
        self._path_sent = False
        self._path_pub = self.create_publisher(
            PathMsg,
            self.get_parameter("path_topic").value,
            10,
        )
        self._path_repub_timer = self.create_timer(1.0, self._republish_path_if_sent)

        _uv = self.get_parameter("use_qlabs_view").value
        self._use_qlabs_view = _uv if isinstance(_uv, bool) else (str(_uv).lower() == "true")
        self._qlabs_frame: Any = None
        self._qlabs_frame_lock = threading.Lock()
        self._qlabs_stop = False
        self._qlabs_camera: Any = None
        self._qlabs_conn = None
        # Overlay: map (m) -> pixel. Si overlay_meters_per_pixel<=0 se calcula desde altura+FOV+scale
        self._qlabs_img_size = (
            int(self.get_parameter("qlabs_image_width").value),
            int(self.get_parameter("qlabs_image_height").value),
        )
        self._scale_to_meters = float(self.get_parameter("scale_to_meters").value)
        cam_loc = list(self.get_parameter("qlabs_camera_location").value)
        _ocx = float(self.get_parameter("qlabs_overlay_center_x").value)
        _ocy = float(self.get_parameter("qlabs_overlay_center_y").value)
        # Auto-calcular centro del overlay si no fue configurado manualmente (0.0 = auto)
        if _ocx == 0.0 and _ocy == 0.0 and len(cam_loc) >= 2:
            # Convertir coordenadas de la cámara QLabs (full-scale) al frame map (metros)
            _ocx = cam_loc[0] * self._scale_to_meters
            _ocy = cam_loc[1] * self._scale_to_meters
            self.get_logger().info(
                "Overlay center auto desde cámara (%.2f, %.2f) × %.2f = (%.4f, %.4f) [map]"
                % (cam_loc[0], cam_loc[1], self._scale_to_meters, _ocx, _ocy)
            )
        self._qlabs_overlay_center = (_ocx, _ocy)
        self._qlabs_overlay_mpp = float(self.get_parameter("qlabs_overlay_meters_per_pixel").value)
        _fv = self.get_parameter("qlabs_flip_vertical").value
        _fh = self.get_parameter("qlabs_flip_horizontal").value
        self._qlabs_flip_v = _fv if isinstance(_fv, bool) else (str(_fv).lower() == "true")
        self._qlabs_flip_h = _fh if isinstance(_fh, bool) else (str(_fh).lower() == "true")
        self._overlay_calibrated = False  # para auto-calibración con primeiro pose_gt

        if _TF_AVAILABLE:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._map_frame = self.get_parameter("map_frame").value
            self._robot_frame = self.get_parameter("robot_frame").value
            self._tf_timer = self.create_timer(0.033, self._update_pose)  # ~30 Hz para pose más fluida en el mapa
        else:
            self._tf_timer = None

        self._path_sub = self.create_subscription(
            PathMsg,
            self.get_parameter("path_topic").value,
            self._on_path,
            5,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self._on_cmd_vel,
            10,
        )
        _use_gt = self.get_parameter("use_pose_gt_topic").value
        self._use_pose_gt = _use_gt if isinstance(_use_gt, bool) else (str(_use_gt).lower() == "true")
        if self._use_pose_gt:
            self._pose_gt_sub = self.create_subscription(
                PoseStamped,
                self.get_parameter("pose_gt_topic").value,
                self._on_pose_gt,
                5,
            )
        else:
            self._pose_gt_sub = None

        self.get_logger().info(
            "Dashboard: pista + QCar + ruta + gráficas (velocidad, giro). "
            "Cierra la ventana para salir."
        )
        if self._use_qlabs_view and not _QLABS_VIEW_AVAILABLE:
            self.get_logger().warn(
                "use_qlabs_view=true pero qvl no disponible; se usará imagen PNG. "
                "Para vista 2D directa de QLabs ejecuta donde tengas qvl (ej. contenedor Quanser)."
            )
            self._use_qlabs_view = False

    def _on_pose_gt(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        self._pose_from_gt = (p.x, p.y, theta)
        if self._use_pose_gt:
            self._pose = self._pose_from_gt
        # Auto-verificación: comparar centro del overlay con la posición real del QCar
        if not self._overlay_calibrated and self._use_qlabs_view:
            cx, cy = self._qlabs_overlay_center
            self.get_logger().info(
                "[AUTO-CAL] QCar real en (%.3f, %.3f). Overlay center en (%.4f, %.4f). "
                "Si el QCar no está centrado en la imagen, ajusta overlay_center_x/y."
                % (p.x, p.y, cx, cy)
            )
            self._overlay_calibrated = True

    def _update_pose(self):
        if self._use_pose_gt and self._pose_from_gt is not None:
            self._pose = self._pose_from_gt
            return
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny, cosy)
            self._pose = (x, y, theta)
        except TFException:
            if self._use_pose_gt and self._pose_from_gt is not None:
                self._pose = self._pose_from_gt
            else:
                self._pose = None

    def _on_path(self, msg: PathMsg):
        self._path_points = []
        self._path_header_frame = msg.header.frame_id or ""
        for p in msg.poses:
            self._path_points.append((p.pose.position.x, p.pose.position.y))

    def _on_cmd_vel(self, msg: Twist):
        t = time.time()
        self._vel_history.append((t, msg.linear.x, msg.angular.z))

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._pose

    def get_path_points(self) -> List[Tuple[float, float]]:
        return list(self._path_points)

    def get_vel_history(self) -> List[Tuple[float, float, float]]:
        return list(self._vel_history)

    def setup_qlabs_view(self) -> bool:
        """Conecta a QLabs, spawnea una FreeCamera cenital y arranca el hilo que pide get_image()."""
        if not _QLABS_VIEW_AVAILABLE or not self._use_qlabs_view:
            return False
        try:
            host = self.get_parameter("qlabs_host").value
            loc = list(self.get_parameter("qlabs_camera_location").value)
            rot_deg = list(self.get_parameter("qlabs_camera_rotation_deg").value)
            w, h = self._qlabs_img_size[0], self._qlabs_img_size[1]
            self.get_logger().info("Conectando a QLabs en %s para vista en vivo..." % host)
            self._qlabs_conn = QuanserInteractiveLabs()
            if not self._qlabs_conn.open(host):
                self.get_logger().error("No se pudo conectar a QLabs.")
                return False
            self._qlabs_camera = QLabsFreeCamera(self._qlabs_conn)
            self._qlabs_camera.spawn_degrees(location=loc, rotation=rot_deg)
            self._qlabs_camera.set_image_capture_resolution(w, h)
            if self._qlabs_overlay_mpp <= 0 and len(loc) >= 3:
                fov_rad = math.radians(float(self.get_parameter("qlabs_camera_fov_deg").value))
                # REGLA DE ORO: Si loc[2] es <= 10.0, probablemente ya está en escala metros.
                # Si es > 10.0, probablemente está en escala QLabs full-scale y requiere multiplicar por scale_to_meters.
                cam_z = loc[2]
                if cam_z > 10.0:
                    effective_z = cam_z * self._scale_to_meters
                else:
                    effective_z = cam_z
                
                half_w_meters = effective_z * math.tan(fov_rad / 2.0)
                self._qlabs_overlay_mpp = (2.0 * half_w_meters) / w if w > 0 else 0.005
                self.get_logger().info(
                    "Overlay MPP Calibrado: %.6f m/px (height=%.2f m, half_w=%.3f m)"
                    % (self._qlabs_overlay_mpp, effective_z, half_w_meters)
                )
            elif self._qlabs_overlay_mpp <= 0:
                self._qlabs_overlay_mpp = 0.01
            self._qlabs_stop = False
            capture_hz = max(10.0, min(30.0, float(self.get_parameter("qlabs_capture_hz").value)))
            capture_dt = 1.0 / capture_hz

            def capture_loop():
                while not self._qlabs_stop and rclpy.ok():
                    try:
                        if self._qlabs_camera:
                            ok, img = self._qlabs_camera.get_image()
                            if ok and img is not None:
                                with self._qlabs_frame_lock:
                                    self._qlabs_frame = img
                    except Exception as e:
                        self.get_logger().warn("QLabs get_image: %s" % e)
                    time.sleep(capture_dt)

            self._qlabs_capture_thread = threading.Thread(target=capture_loop, daemon=True)
            self._qlabs_capture_thread.start()
            self.get_logger().info(
                "Vista 2D directa desde QLabs activa (mismo template que la ventana del simulador)."
            )
            return True
        except Exception as e:
            self.get_logger().error("setup_qlabs_view: %s" % e)
            return False

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """Convierte coordenadas mapa (m) a pixel en la imagen QLabs (origen abajo-izq en matplotlib)."""
        cx, cy = self._qlabs_overlay_center
        mpp = self._qlabs_overlay_mpp
        w, h = self._qlabs_img_size[0], self._qlabs_img_size[1]
        px = (x - cx) / mpp + w / 2.0
        py = (y - cy) / mpp
        py = h - py
        return (px, py)

    def get_qlabs_frame(self):
        """Devuelve la última imagen de QLabs (o None) y tamaño (w,h)."""
        with self._qlabs_frame_lock:
            img = self._qlabs_frame
        if img is None:
            return None, self._qlabs_img_size[0], self._qlabs_img_size[1]
        h, w = img.shape[:2]
        return img, w, h

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Convierte pixel (imagen QLabs, origen abajo-izq) a coordenadas mapa (m).
        Mismo frame que /qcar_pose_gt y TF map->base_link para que el QCar vaya hacia los puntos del GUI."""
        cx, cy = self._qlabs_overlay_center
        mpp = self._qlabs_overlay_mpp
        w, h = self._qlabs_img_size[0], self._qlabs_img_size[1]
        x = cx + (px - w / 2.0) * mpp
        y = cy + (h - py) * mpp
        return (x, y)

    def add_waypoint(self, x: float, y: float) -> None:
        with self._clicked_waypoints_lock:
            if len(self._clicked_waypoints) < 100:
                self._clicked_waypoints.append((x, y))
                self.get_logger().info("Waypoint %d: (%.3f, %.3f)" % (len(self._clicked_waypoints), x, y))

    def clear_waypoints(self) -> None:
        with self._clicked_waypoints_lock:
            self._clicked_waypoints.clear()
        self._path_sent = False
        self.get_logger().info("Waypoints borrados.")

    def get_clicked_waypoints(self) -> List[Tuple[float, float]]:
        with self._clicked_waypoints_lock:
            return list(self._clicked_waypoints)

    def _republish_path_if_sent(self):
        """Re-publica la ruta cada segundo para que el Pure Pursuit la reciba de forma fiable."""
        if not self._path_sent:
            return
        with self._clicked_waypoints_lock:
            wps = list(self._clicked_waypoints)
        if len(wps) < 2:
            self._path_sent = False
            return
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value
        for x, y in wps:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self._path_pub.publish(path)

    def publish_clicked_path(self) -> int:
        """Publica la ruta de waypoints clicados en /waypoints_path. Devuelve número de puntos."""
        with self._clicked_waypoints_lock:
            wps = list(self._clicked_waypoints)
        if len(wps) < 2:
            self.get_logger().warn("Añade al menos 2 puntos en el mapa antes de Enviar.")
            return 0
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value
        for x, y in wps:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self._path_pub.publish(path)
        self._path_sent = True
        self.get_logger().info(
            "Ruta enviada: %d waypoints a /waypoints_path (se reenvía cada 1 s). "
            "Primer punto (%.3f, %.3f), último (%.3f, %.3f) [map]. "
            "Si el QCar no para en el último, ajusta qlabs_overlay_center_x/y o waypoint_reached_radius."
            % (len(wps), wps[0][0], wps[0][1], wps[-1][0], wps[-1][1])
        )
        return len(wps)


def _run_ros_spin(node: QCarDashboardGuiNode):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def _run_gui(node: QCarDashboardGuiNode):
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button
        from matplotlib.patches import FancyBboxPatch
        import matplotlib.patheffects as path_effects
        import numpy as np
    except ImportError as e:
        node.get_logger().error("matplotlib no disponible: %s" % e)
        return

    extent = node._extent
    circuit_path = node._circuit_image_path
    use_qlabs = node._use_qlabs_view
    history_sec = node.get_parameter("history_sec").value

    # ═══════════════════════════════════════════════════════════════════
    #  🎨 PREMIUM DARK THEME — Estilo telemetría F1
    # ═══════════════════════════════════════════════════════════════════
    # Colores principales
    BG_DARK = "#0d1117"        # Fondo principal (GitHub dark)
    BG_PANEL = "#161b22"       # Fondo de paneles
    BG_CARD = "#1c2333"        # Fondo de tarjetas/gráficas
    BORDER = "#30363d"         # Bordes sutiles
    TEXT_PRIMARY = "#e6edf3"   # Texto principal
    TEXT_SECONDARY = "#8b949e" # Texto secundario
    TEXT_MUTED = "#484f58"     # Texto apagado

    # Acentos neón
    CYAN = "#00d4ff"           # Velocidad lineal / QCar
    MAGENTA = "#ff006e"        # Velocidad angular
    AMBER = "#ffbe0b"          # Waypoints clicados
    GREEN = "#06d6a0"          # Ruta / Enviar
    RED = "#ef476f"            # Stop / Robot marker glow
    PURPLE = "#b388ff"         # Detalles premium

    # Grid sutil
    GRID_COLOR = "#21262d"
    GRID_ALPHA = 0.6

    # Reset de estilos previos
    plt.style.use("dark_background")

    # ─── Layout: 4 filas (status + mapa + vel + omega) ───────────────
    fig = plt.figure(figsize=(13, 10.5))
    fig.patch.set_facecolor(BG_DARK)

    # GridSpec para control preciso del layout
    gs = fig.add_gridspec(
        4, 1,
        height_ratios=[0.08, 1.4, 0.5, 0.5],
        hspace=0.30,
        left=0.07, right=0.96,
        top=0.95, bottom=0.08,
    )

    # Axes
    ax_status = fig.add_subplot(gs[0])  # Barra de estado
    ax_track = fig.add_subplot(gs[1])   # Mapa/pista
    ax_v = fig.add_subplot(gs[2])       # Velocidad lineal
    ax_w = fig.add_subplot(gs[3])       # Velocidad angular

    # ─── Barra de estado (sin ejes, solo texto) ──────────────────────
    ax_status.set_facecolor(BG_PANEL)
    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    ax_status.axis("off")
    for spine in ax_status.spines.values():
        spine.set_visible(False)

    # Título principal en la barra de estado
    title_text = ax_status.text(
        0.01, 0.5, "◈  QCAR2 TELEMETRY DASHBOARD",
        transform=ax_status.transAxes, fontsize=13, fontweight="bold",
        verticalalignment="center", color=CYAN,
        fontfamily="monospace",
    )
    title_text.set_path_effects([
        path_effects.withStroke(linewidth=2, foreground=BG_DARK),
    ])

    # Indicadores de estado (se actualizan en update_plots)
    status_tf = ax_status.text(
        0.52, 0.5, "◉ TF: ---",
        transform=ax_status.transAxes, fontsize=9.5, fontweight="bold",
        verticalalignment="center", color=TEXT_MUTED,
        fontfamily="monospace",
    )
    status_qlabs = ax_status.text(
        0.67, 0.5, "◉ QLabs: ---",
        transform=ax_status.transAxes, fontsize=9.5, fontweight="bold",
        verticalalignment="center", color=TEXT_MUTED,
        fontfamily="monospace",
    )
    status_pursuit = ax_status.text(
        0.84, 0.5, "◉ Pursuit: ---",
        transform=ax_status.transAxes, fontsize=9.5, fontweight="bold",
        verticalalignment="center", color=TEXT_MUTED,
        fontfamily="monospace",
    )

    # Línea separadora debajo de la barra
    ax_status.axhline(y=0.0, color=BORDER, linewidth=1.5, xmin=0, xmax=1)

    # ─── Panel del mapa ──────────────────────────────────────────────
    ax_track.set_facecolor(BG_CARD)
    ax_track.set_aspect("equal")
    ax_track.set_xlabel(
        "x (m)" if not use_qlabs else "pixel",
        fontsize=10, color=TEXT_SECONDARY, fontfamily="monospace",
    )
    ax_track.set_ylabel(
        "y (m)" if not use_qlabs else "pixel",
        fontsize=10, color=TEXT_SECONDARY, fontfamily="monospace",
    )
    ax_track.tick_params(labelsize=8, colors=TEXT_SECONDARY)
    for spine in ax_track.spines.values():
        spine.set_color(BORDER)
        spine.set_linewidth(1.2)

    # Fondo: PNG o vista QLabs (se actualiza en update_plots)
    if not use_qlabs and circuit_path and os.path.isfile(circuit_path):
        try:
            img = plt.imread(circuit_path)
            ax_track.imshow(img, extent=extent, origin="lower", aspect="equal", zorder=0)
        except Exception as e:
            node.get_logger().warn("No se pudo cargar imagen de pista: %s" % e)
    if not use_qlabs:
        ax_track.set_xlim(extent[0], extent[1])
        ax_track.set_ylim(extent[2], extent[3])

    ax_track.set_title(
        "TRACK MAP" + ("  ●  LIVE FEED" if use_qlabs else "  ●  STATIC IMAGE"),
        fontsize=11, fontweight="bold", color=TEXT_PRIMARY,
        fontfamily="monospace", pad=8,
    )
    ax_track.grid(True, alpha=GRID_ALPHA, linestyle=":", color=GRID_COLOR, linewidth=0.5)

    # Ruta recibida (verde neón con glow)
    path_line, = ax_track.plot([], [], color=GREEN, linewidth=2.8, label="Ruta", zorder=5, alpha=0.9)
    path_line.set_path_effects([
        path_effects.withStroke(linewidth=5, foreground=GREEN + "40"),
    ])
    path_pts, = ax_track.plot([], [], "o", color=GREEN, markersize=4, zorder=6, alpha=0.7)

    # QCar robot triangle (cyan neón con glow)
    robot_tri, = ax_track.plot([], [], color=CYAN, linewidth=3.5, label="QCar", zorder=10)
    robot_tri.set_path_effects([
        path_effects.withStroke(linewidth=6, foreground=CYAN + "50"),
    ])
    # Robot glow circle (halo)
    robot_glow, = ax_track.plot([], [], "o", color=CYAN, markersize=18, alpha=0.15, zorder=9)

    # Posición texto (estilo HUD)
    pos_text = ax_track.text(
        0.02, 0.97, "POS: ---",
        transform=ax_track.transAxes, fontsize=10, fontweight="bold",
        verticalalignment="top", fontfamily="monospace",
        bbox=dict(
            boxstyle="round,pad=0.5",
            facecolor=BG_DARK + "dd",
            edgecolor=CYAN + "80",
            linewidth=1.5,
        ),
        color=CYAN,
    )

    # Badge LIVE (si usa QLabs)
    if use_qlabs:
        live_text = ax_track.text(
            0.98, 0.97, "● LIVE",
            transform=ax_track.transAxes, fontsize=10, fontweight="bold",
            verticalalignment="top", horizontalalignment="right",
            fontfamily="monospace",
            bbox=dict(
                boxstyle="round,pad=0.35",
                facecolor=RED + "cc",
                edgecolor=RED,
                linewidth=1.5,
            ),
            color="white",
        )

    # Waypoints clicados (amber/oro con glow)
    clicked_line, = ax_track.plot(
        [], [], color=AMBER, linewidth=2.2, linestyle="--",
        label="Waypoints", zorder=7, alpha=0.9,
    )
    clicked_line.set_path_effects([
        path_effects.withStroke(linewidth=4, foreground=AMBER + "30"),
    ])
    clicked_pts, = ax_track.plot(
        [], [], "D", color=AMBER, markersize=9,
        markeredgecolor="white", markeredgewidth=1.5, zorder=8,
    )

    # Contador de waypoints (estilo badge)
    wp_count_text = ax_track.text(
        0.5, 0.02,
        "⊚ WAYPOINTS: 0  —  Clic en la pista para añadir, luego ENVIAR",
        transform=ax_track.transAxes, fontsize=9, ha="center", va="bottom",
        fontfamily="monospace",
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor=BG_DARK + "dd",
            edgecolor=AMBER + "60",
            linewidth=1,
        ),
        color=AMBER,
    )

    # Legend personalizada
    legend = ax_track.legend(
        loc="upper right", fontsize=9, framealpha=0.85,
        facecolor=BG_PANEL, edgecolor=BORDER,
        labelcolor=TEXT_PRIMARY,
    )
    legend.get_frame().set_linewidth(1)

    # ─── Botones premium ─────────────────────────────────────────────
    ax_btn_enviar = fig.add_axes([0.25, 0.015, 0.15, 0.04])
    ax_btn_borrar = fig.add_axes([0.42, 0.015, 0.15, 0.04])
    ax_btn_info = fig.add_axes([0.59, 0.015, 0.18, 0.04])

    # Botón ENVIAR (verde neón)
    btn_enviar = Button(ax_btn_enviar, "▶  ENVIAR RUTA", color="#0a3d2a", hovercolor="#0d5c3f")
    ax_btn_enviar.set_facecolor("#0a3d2a")
    for spine in ax_btn_enviar.spines.values():
        spine.set_color(GREEN)
        spine.set_linewidth(1.5)
    try:
        btn_enviar.label.set_fontsize(10)
        btn_enviar.label.set_fontweight("bold")
        btn_enviar.label.set_color(GREEN)
        btn_enviar.label.set_fontfamily("monospace")
    except Exception:
        pass

    # Botón BORRAR (rojo sutil)
    btn_borrar = Button(ax_btn_borrar, "✕  BORRAR", color="#3d1a1a", hovercolor="#5c2020")
    ax_btn_borrar.set_facecolor("#3d1a1a")
    for spine in ax_btn_borrar.spines.values():
        spine.set_color(RED + "80")
        spine.set_linewidth(1.5)
    try:
        btn_borrar.label.set_fontsize(10)
        btn_borrar.label.set_fontweight("bold")
        btn_borrar.label.set_color(RED)
        btn_borrar.label.set_fontfamily("monospace")
    except Exception:
        pass

    # Botón INFO (morado, muestra info de waypoints)
    btn_info = Button(ax_btn_info, "◈  ROUTE INFO", color="#1a1a3d", hovercolor="#2a2a5c")
    ax_btn_info.set_facecolor("#1a1a3d")
    for spine in ax_btn_info.spines.values():
        spine.set_color(PURPLE + "80")
        spine.set_linewidth(1.5)
    try:
        btn_info.label.set_fontsize(10)
        btn_info.label.set_fontweight("bold")
        btn_info.label.set_color(PURPLE)
        btn_info.label.set_fontfamily("monospace")
    except Exception:
        pass

    def on_enviar(_event):
        n = node.publish_clicked_path()
        if n >= 2:
            wp_count_text.set_text("⊚ WAYPOINTS: %d  —  ✓ RUTA ENVIADA AL COCHE" % n)
            wp_count_text.set_color(GREEN)
        fig.canvas.draw_idle()

    def on_borrar(_event):
        node.clear_waypoints()
        wp_count_text.set_text("⊚ WAYPOINTS: 0  —  Clic en la pista para añadir, luego ENVIAR")
        wp_count_text.set_color(AMBER)
        clicked_line.set_data([], [])
        clicked_pts.set_data([], [])
        fig.canvas.draw_idle()

    def on_info(_event):
        wps = node.get_clicked_waypoints()
        if len(wps) < 2:
            wp_count_text.set_text("⊚ INFO: Añade al menos 2 puntos para ver la ruta")
            wp_count_text.set_color(PURPLE)
        else:
            total_dist = 0.0
            for i in range(1, len(wps)):
                dx = wps[i][0] - wps[i - 1][0]
                dy = wps[i][1] - wps[i - 1][1]
                total_dist += math.sqrt(dx * dx + dy * dy)
            wp_count_text.set_text(
                "⊚ ROUTE: %d pts  |  %.2f m  |  START (%.2f, %.2f)  →  END (%.2f, %.2f)"
                % (len(wps), total_dist, wps[0][0], wps[0][1], wps[-1][0], wps[-1][1])
            )
            wp_count_text.set_color(PURPLE)
        fig.canvas.draw_idle()

    btn_enviar.on_clicked(on_enviar)
    btn_borrar.on_clicked(on_borrar)
    btn_info.on_clicked(on_info)

    # ─── Click handler del mapa ──────────────────────────────────────
    def on_map_click(event):
        if event.inaxes != ax_track or event.button != 1:
            return
        if use_qlabs:
            px, py = display_to_pixel(event.xdata, event.ydata)
            x, y = node.pixel_to_world(px, py)
        else:
            x, y = event.xdata, event.ydata
        node.add_waypoint(x, y)
        wps = node.get_clicked_waypoints()
        wp_count_text.set_text("⊚ WAYPOINTS: %d  —  Clic para más, luego ENVIAR" % len(wps))
        wp_count_text.set_color(AMBER)
        if use_qlabs:
            pts = [node.world_to_pixel(wx, wy) for wx, wy in wps]
            pts_disp = [pixel_to_display(px, py) for px, py in pts]
            xs, ys = zip(*pts_disp) if pts_disp else ([], [])
        else:
            xs, ys = zip(*wps) if wps else ([], [])
        clicked_line.set_data(xs, ys)
        clicked_pts.set_data(xs, ys)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_map_click)

    # ─── Gráfica velocidad lineal (cyan neón + fill) ─────────────────
    ax_v.set_facecolor(BG_CARD)
    for spine in ax_v.spines.values():
        spine.set_color(BORDER)
        spine.set_linewidth(1)
    ax_v.set_ylabel("v (m/s)", fontsize=10, color=CYAN, fontfamily="monospace")
    ax_v.set_title(
        "LINEAR VELOCITY", fontsize=10, fontweight="bold",
        color=TEXT_PRIMARY, fontfamily="monospace", pad=6,
    )
    ax_v.grid(True, alpha=GRID_ALPHA, linestyle=":", color=GRID_COLOR, linewidth=0.5)
    ax_v.set_xlim(0, history_sec)
    ax_v.set_ylim(-0.5, 0.5)
    ax_v.tick_params(labelsize=8, colors=TEXT_SECONDARY)

    line_vel, = ax_v.plot([], [], color=CYAN, linewidth=2.2, label="v", zorder=5)
    line_vel.set_path_effects([
        path_effects.withStroke(linewidth=4, foreground=CYAN + "30"),
    ])
    # Fill under the velocity curve (se actualiza en update_plots)
    vel_fill = [None]

    # Valor actual de velocidad (texto grande)
    vel_value_text = ax_v.text(
        0.98, 0.92, "0.00",
        transform=ax_v.transAxes, fontsize=16, fontweight="bold",
        verticalalignment="top", horizontalalignment="right",
        fontfamily="monospace", color=CYAN, alpha=0.85,
    )
    vel_unit_text = ax_v.text(
        0.98, 0.6, "m/s",
        transform=ax_v.transAxes, fontsize=8,
        verticalalignment="top", horizontalalignment="right",
        fontfamily="monospace", color=TEXT_MUTED,
    )

    # ─── Gráfica velocidad angular (magenta + fill) ──────────────────
    ax_w.set_facecolor(BG_CARD)
    for spine in ax_w.spines.values():
        spine.set_color(BORDER)
        spine.set_linewidth(1)
    ax_w.set_xlabel("TIME (s)", fontsize=10, color=TEXT_SECONDARY, fontfamily="monospace")
    ax_w.set_ylabel("ω (rad/s)", fontsize=10, color=MAGENTA, fontfamily="monospace")
    ax_w.set_title(
        "ANGULAR VELOCITY", fontsize=10, fontweight="bold",
        color=TEXT_PRIMARY, fontfamily="monospace", pad=6,
    )
    ax_w.grid(True, alpha=GRID_ALPHA, linestyle=":", color=GRID_COLOR, linewidth=0.5)
    ax_w.set_xlim(0, history_sec)
    ax_w.set_ylim(-1.0, 1.0)
    ax_w.tick_params(labelsize=8, colors=TEXT_SECONDARY)

    line_w, = ax_w.plot([], [], color=MAGENTA, linewidth=2.2, label="ω", zorder=5)
    line_w.set_path_effects([
        path_effects.withStroke(linewidth=4, foreground=MAGENTA + "30"),
    ])
    omega_fill = [None]

    # Valor actual de omega (texto grande)
    omega_value_text = ax_w.text(
        0.98, 0.92, "0.00",
        transform=ax_w.transAxes, fontsize=16, fontweight="bold",
        verticalalignment="top", horizontalalignment="right",
        fontfamily="monospace", color=MAGENTA, alpha=0.85,
    )
    omega_unit_text = ax_w.text(
        0.98, 0.6, "rad/s",
        transform=ax_w.transAxes, fontsize=8,
        verticalalignment="top", horizontalalignment="right",
        fontfamily="monospace", color=TEXT_MUTED,
    )

    # ─── Variables de estado ─────────────────────────────────────────
    t0 = [None]
    qlabs_im_artist = [None]
    frame_count = [0]
    last_vel = [0.0]
    last_omega = [0.0]

    def pixel_to_display(px, py):
        w, h = node._qlabs_img_size[0], node._qlabs_img_size[1]
        px_d = (w - px) if node._qlabs_flip_h else px
        py_d = (h - py) if node._qlabs_flip_v else py
        return (px_d, py_d)

    def display_to_pixel(px_disp, py_disp):
        w, h = node._qlabs_img_size[0], node._qlabs_img_size[1]
        px = (w - px_disp) if node._qlabs_flip_h else px_disp
        py = (h - py_disp) if node._qlabs_flip_v else py_disp
        return (px, py)

    def draw_robot(pose: Optional[Tuple[float, float, float]], in_pixels: bool):
        if pose is None:
            robot_tri.set_data([], [])
            robot_glow.set_data([], [])
            pos_text.set_text("POS: — no TF —")
            pos_text.set_color(RED)
            return
        x, y, theta = pose
        if in_pixels:
            px, py = node.world_to_pixel(x, y)
            L, W = 16.0, 8.0  # Más grande para mejor visibilidad
            c, s = math.cos(theta), math.sin(theta)
            xf = px + L * c
            yf = py + L * s
            xbl = px - L * c + W * s
            ybl = py - L * s - W * c
            xbr = px - L * c - W * s
            ybr = py - L * s + W * c
            xf_d, yf_d = pixel_to_display(xf, yf)
            xbl_d, ybl_d = pixel_to_display(xbl, ybl)
            xbr_d, ybr_d = pixel_to_display(xbr, ybr)
            robot_tri.set_data([xf_d, xbl_d, xbr_d, xf_d], [yf_d, ybl_d, ybr_d, yf_d])
            cx_d, cy_d = pixel_to_display(px, py)
            robot_glow.set_data([cx_d], [cy_d])
            pos_text.set_text("POS  x=%.3f  y=%.3f  θ=%.2f°" % (x, y, math.degrees(theta)))
            pos_text.set_color(CYAN)
        else:
            L, W = 0.18, 0.10  # Ligeramente más grande
            c, s = math.cos(theta), math.sin(theta)
            xf = x + L * c
            yf = y + L * s
            xbl = x - L * c + W * s
            ybl = y - L * s - W * c
            xbr = x - L * c - W * s
            ybr = y - L * s + W * c
            robot_tri.set_data([xf, xbl, xbr, xf], [yf, ybl, ybr, yf])
            robot_glow.set_data([x], [y])
            pos_text.set_text("POS  x=%.3f  y=%.3f  θ=%.2f°" % (x, y, math.degrees(theta)))
            pos_text.set_color(CYAN)

    def update_plots():
        frame_count[0] += 1
        pose = node.get_pose()
        path_pts_list = node.get_path_points()
        vel_list = node.get_vel_history()
        in_pixels = use_qlabs

        # ── Actualizar indicadores de estado ──
        # TF
        if pose is not None:
            status_tf.set_text("◉ TF: OK")
            status_tf.set_color(GREEN)
        else:
            status_tf.set_text("◉ TF: ---")
            status_tf.set_color(RED)

        # QLabs
        if use_qlabs:
            qlabs_img_check, _, _ = node.get_qlabs_frame()
            if qlabs_img_check is not None:
                status_qlabs.set_text("◉ QLabs: LIVE")
                status_qlabs.set_color(GREEN)
            else:
                status_qlabs.set_text("◉ QLabs: WAIT")
                status_qlabs.set_color(AMBER)
        else:
            status_qlabs.set_text("◉ QLabs: PNG")
            status_qlabs.set_color(TEXT_SECONDARY)

        # Pure Pursuit (basado en si hay velocidad)
        if vel_list and len(vel_list) > 0:
            _, latest_v, latest_w = vel_list[-1]
            last_vel[0] = latest_v
            last_omega[0] = latest_w
            if abs(latest_v) > 0.01:
                status_pursuit.set_text("◉ Pursuit: ▶ RUN")
                status_pursuit.set_color(GREEN)
            else:
                status_pursuit.set_text("◉ Pursuit: ⏸ IDLE")
                status_pursuit.set_color(AMBER)
        else:
            status_pursuit.set_text("◉ Pursuit: ---")
            status_pursuit.set_color(TEXT_MUTED)

        # ── Vista QLabs ──
        if use_qlabs:
            qlabs_img, w, h = node.get_qlabs_frame()
            if qlabs_img is not None:
                if qlabs_img.ndim == 3 and qlabs_img.shape[2] == 3:
                    qlabs_img = qlabs_img[:, :, ::-1].copy()
                if node._qlabs_flip_v:
                    qlabs_img = qlabs_img[::-1, :].copy()
                if node._qlabs_flip_h:
                    qlabs_img = qlabs_img[:, ::-1].copy()
                if qlabs_im_artist[0] is None:
                    qlabs_im_artist[0] = ax_track.imshow(
                        qlabs_img, extent=[0, w, 0, h], origin="lower", aspect="equal", zorder=0
                    )
                    ax_track.set_xlim(0, w)
                    ax_track.set_ylim(0, h)
                else:
                    qlabs_im_artist[0].set_data(qlabs_img)

        # ── Ruta ──
        if path_pts_list:
            if in_pixels:
                pts = [node.world_to_pixel(x, y) for x, y in path_pts_list]
                pts_disp = [pixel_to_display(px, py) for px, py in pts]
                xs, ys = zip(*pts_disp) if pts_disp else ([], [])
            else:
                xs, ys = zip(*path_pts_list)
            path_line.set_data(xs, ys)
            path_pts.set_data(xs, ys)
        else:
            path_line.set_data([], [])
            path_pts.set_data([], [])

        # ── Waypoints clicados ──
        clicked_wps = node.get_clicked_waypoints()
        if clicked_wps:
            if in_pixels:
                c_pts = [node.world_to_pixel(x, y) for x, y in clicked_wps]
                c_disp = [pixel_to_display(px, py) for px, py in c_pts]
                cx, cy = zip(*c_disp) if c_disp else ([], [])
            else:
                cx, cy = zip(*clicked_wps)
            clicked_line.set_data(cx, cy)
            clicked_pts.set_data(cx, cy)
            wp_count_text.set_text("⊚ WAYPOINTS: %d  —  Clic para más, luego ENVIAR" % len(clicked_wps))
        else:
            clicked_line.set_data([], [])
            clicked_pts.set_data([], [])

        # ── Robot ──
        draw_robot(pose, in_pixels)

        # ── Gráficas de velocidad ──
        if vel_list:
            if t0[0] is None:
                t0[0] = vel_list[0][0]
            times = [t - t0[0] for t, _, _ in vel_list]
            vx = [v for _, v, _ in vel_list]
            wz = [w for _, _, w in vel_list]
            line_vel.set_data(times, vx)
            line_w.set_data(times, wz)

            # Fill under curves (remove old, add new)
            if vel_fill[0] is not None:
                try:
                    vel_fill[0].remove()
                except Exception:
                    pass
            if omega_fill[0] is not None:
                try:
                    omega_fill[0].remove()
                except Exception:
                    pass

            if len(times) > 1:
                try:
                    vel_fill[0] = ax_v.fill_between(
                        times, 0, vx, alpha=0.12, color=CYAN, zorder=2,
                    )
                    omega_fill[0] = ax_w.fill_between(
                        times, 0, wz, alpha=0.12, color=MAGENTA, zorder=2,
                    )
                except Exception:
                    pass

            if times:
                t_min, t_max = max(0, times[-1] - history_sec), times[-1] + 1
                ax_v.set_xlim(t_min, t_max)
                ax_w.set_xlim(t_min, t_max)
                if vx:
                    v_lo, v_hi = min(vx), max(vx)
                    margin = 0.15 * (v_hi - v_lo) if v_hi != v_lo else 0.15
                    ax_v.set_ylim(v_lo - margin, v_hi + margin)
                if wz:
                    w_lo, w_hi = min(wz), max(wz)
                    margin = 0.15 * (w_hi - w_lo) if w_hi != w_lo else 0.15
                    ax_w.set_ylim(w_lo - margin, w_hi + margin)

            # Actualizar valores en pantalla
            vel_value_text.set_text("%.3f" % last_vel[0])
            omega_value_text.set_text("%.3f" % last_omega[0])

        fig.canvas.draw_idle()

    # ─── Iniciar hilo ROS ────────────────────────────────────────────
    ros_thread = threading.Thread(target=_run_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        fig.canvas.manager.set_window_title("◈ QCar2 Telemetry Dashboard — ACC Autonomy")
    except Exception:
        pass

    update_hz = max(15.0, min(45.0, node.get_parameter("update_hz").value))
    timer_ms = max(22, int(1000.0 / update_hz))
    timer = fig.canvas.new_timer(interval=timer_ms)
    timer.add_callback(update_plots)
    timer.start()

    plt.show()
    timer.stop()
    try:
        plt.close(fig)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = QCarDashboardGuiNode()
    if node._use_qlabs_view and _QLABS_VIEW_AVAILABLE:
        if not node.setup_qlabs_view():
            node.get_logger().warn(
                "No se pudo conectar a QLabs (¿está abierto con el escenario?). "
                "Se usará imagen PNG de la pista."
            )
            node._use_qlabs_view = False
    try:
        _run_gui(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error("GUI: %s" % e)
    finally:
        node._qlabs_stop = True
        if getattr(node, "_qlabs_capture_thread", None) and node._qlabs_capture_thread.is_alive():
            node._qlabs_capture_thread.join(timeout=1.0)
    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

