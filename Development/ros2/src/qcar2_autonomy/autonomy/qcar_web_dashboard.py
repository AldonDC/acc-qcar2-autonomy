#!/usr/bin/env python3
"""
QCar2 Web Dashboard — Professional browser-based control center.

Features:
  - QLabs overhead live view with waypoint/car overlay
  - CSI camera feed (front/rear)
  - RGBD color + depth camera feeds (colorized)
  - Real-time velocity & steering telemetry with mini-charts
  - Click-to-add waypoints on the track map
  - Route management (send / clear / info)

Usage:
  ros2 run qcar2_autonomy qcar_web_dashboard
  Then open http://localhost:8085 in a browser.

No external dependencies beyond ROS2, OpenCV, and numpy.
"""
from __future__ import annotations

import io
import json
import math
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler
from socketserver import TCPServer, ThreadingMixIn
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg, OccupancyGrid
from sensor_msgs.msg import Image as ImageMsg

# QLabs overhead (optional — qvl is copied into python_resources so it's
# always reachable at /workspaces/isaac_ros-dev/python_resources inside the
# Isaac ROS container where Development/ is mounted as isaac_ros-dev/).
import sys as _sys, os as _os
_QLABS_OK = False
_QVL_PATHS = [
    "/workspaces/isaac_ros-dev/python_resources",   # ← primary: always mounted
    "/usr/local/lib/python3/dist-packages",
    "/opt/quanser/python",
    "/home/qcar2_scripts/python",
]
for _p in _QVL_PATHS:
    if _os.path.isdir(_os.path.join(_p, "qvl")) and _p not in _sys.path:
        _sys.path.insert(0, _p)
try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.free_camera import QLabsFreeCamera
    _QLABS_OK = True
except ImportError:
    pass

# ═══════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════

_bridge = CvBridge()


def _no_signal(w: int = 640, h: int = 480, label: str = "NO SIGNAL") -> np.ndarray:
    """Bright placeholder frame — impossible to confuse with a dark/black bug."""
    f = np.zeros((h, w, 3), dtype=np.uint8)
    f[:] = (50, 40, 30)          # dark blue-ish — clearly NOT black
    # Bright border so it's obvious the frame is rendering
    cv2.rectangle(f, (2, 2), (w - 3, h - 3), (0, 180, 220), 2)
    sz = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)[0]
    cx, cy = (w - sz[0]) // 2, (h + sz[1]) // 2
    cv2.putText(f, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (0, 220, 255), 2, cv2.LINE_AA)
    return f


def _colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Normalise and apply TURBO colourmap to a depth image."""
    if depth is None:
        return _no_signal(label="NO DEPTH")
    if depth.dtype == np.uint16:
        d8 = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    elif depth.dtype == np.float32:
        clip = np.clip(depth, 0.0, 10.0)
        d8 = (clip / 10.0 * 255).astype(np.uint8)
    else:
        d8 = depth.astype(np.uint8) if depth.ndim == 2 else cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
    return cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)


# ═══════════════════════════════════════════════════════════════════
#  ROS 2 Node
# ═══════════════════════════════════════════════════════════════════

class QCarWebDashboard(Node):
    """Single ROS 2 node that subscribes to cameras, pose and velocity,
    manages waypoints, drives the QLabs overhead capture, and exposes
    everything through an HTTP server (started externally).
    """

    def __init__(self):
        super().__init__("qcar_web_dashboard")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("port", 8085)
        self.declare_parameter("pose_gt_topic", "/qcar_pose_gt")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("path_topic", "/waypoints_path")
        self.declare_parameter("csi_topic", "/camera/csi_image")
        self.declare_parameter("color_topic", "/camera/color_image")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("overhead_fps", 25)
        self.declare_parameter("camera_fps", 20)
        self.declare_parameter("jpeg_quality", 75)
        # QLabs overhead camera
        self.declare_parameter("use_qlabs_view", True)
        self.declare_parameter("qlabs_host", "localhost")
        self.declare_parameter("qlabs_camera_location", [0.15, 1.7, 5.0])
        self.declare_parameter("qlabs_camera_rotation_deg", [0.0, 90.0, 0.0])
        self.declare_parameter("qlabs_camera_fov_deg", 60.0)
        self.declare_parameter("qlabs_image_width", 1280)
        self.declare_parameter("qlabs_image_height", 720)
        self.declare_parameter("scale_to_meters", 0.1)
        self.declare_parameter("qlabs_overlay_center_x", 0.0)
        self.declare_parameter("qlabs_overlay_center_y", 0.0)
        self.declare_parameter("qlabs_overlay_meters_per_pixel", 0.0)
        self.declare_parameter("qlabs_flip_vertical", True)
        self.declare_parameter("qlabs_flip_horizontal", False)
        # QLabs overhead HTTP bridge (qlabs_overhead_bridge.py on port 8086)
        self.declare_parameter("overhead_bridge_host", "localhost")
        self.declare_parameter("overhead_bridge_port", 8086)

        # ── Internal state ──────────────────────────────────────────
        self._lock = threading.Lock()
        self._frame_csi: Optional[np.ndarray] = None
        self._frame_color: Optional[np.ndarray] = None
        self._frame_depth_raw: Optional[np.ndarray] = None
        self._frame_overhead: Optional[np.ndarray] = None
        self._overhead_stop = False

        # OccupancyGrid fallback (used when QLabs overhead is unavailable)
        self._occ_grid_img: Optional[np.ndarray] = None

        self._pose: Optional[Tuple[float, float, float]] = None
        self._vel_history: deque = deque(maxlen=300)
        self._path_points: List[Tuple[float, float]] = []
        self._waypoints: List[Tuple[float, float]] = []
        self._path_sent = False

        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # ── Overlay calibration ─────────────────────────────────────
        self._img_w = int(self.get_parameter("qlabs_image_width").value)
        self._img_h = int(self.get_parameter("qlabs_image_height").value)
        self._scale = float(self.get_parameter("scale_to_meters").value)
        cam_loc = list(self.get_parameter("qlabs_camera_location").value)
        _ocx = float(self.get_parameter("qlabs_overlay_center_x").value)
        _ocy = float(self.get_parameter("qlabs_overlay_center_y").value)
        if _ocx == 0.0 and _ocy == 0.0 and len(cam_loc) >= 2:
            _ocx = cam_loc[0] * self._scale
            _ocy = cam_loc[1] * self._scale
        self._overlay_cx = _ocx
        self._overlay_cy = _ocy

        _mpp = float(self.get_parameter("qlabs_overlay_meters_per_pixel").value)
        if _mpp <= 0.0 and len(cam_loc) >= 3:
            fov = math.radians(float(self.get_parameter("qlabs_camera_fov_deg").value))
            z = cam_loc[2] if cam_loc[2] <= 10.0 else cam_loc[2] * self._scale
            half_w = z * math.tan(fov / 2.0)
            _mpp = (2.0 * half_w) / self._img_w if self._img_w > 0 else 0.005
        self._mpp = _mpp if _mpp > 0 else 0.005

        _fv = self.get_parameter("qlabs_flip_vertical").value
        _fh = self.get_parameter("qlabs_flip_horizontal").value
        self._flip_v = _fv if isinstance(_fv, bool) else str(_fv).lower() == "true"
        self._flip_h = _fh if isinstance(_fh, bool) else str(_fh).lower() == "true"

        _uv = self.get_parameter("use_qlabs_view").value
        self._use_qlabs = (_uv if isinstance(_uv, bool) else str(_uv).lower() == "true")

        # ── ROS subscriptions ──────────────────────────────────────
        self._sub_pose = self.create_subscription(
            PoseStamped, self.get_parameter("pose_gt_topic").value, self._on_pose, 5)
        self._sub_vel = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_vel, 10)
        self._sub_path = self.create_subscription(
            PathMsg, self.get_parameter("path_topic").value, self._on_path, 5)
        self._sub_csi = self.create_subscription(
            ImageMsg, self.get_parameter("csi_topic").value, self._on_csi, 5)
        self._sub_color = self.create_subscription(
            ImageMsg, self.get_parameter("color_topic").value, self._on_color, 5)
        self._sub_depth = self.create_subscription(
            ImageMsg, self.get_parameter("depth_topic").value, self._on_depth, 5)

        # Subscribe to /map (Cartographer OccupancyGrid) — transient local so we
        # receive the last published map even if we subscribe after it was sent.
        _map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, _map_qos)

        # Publisher for waypoints path
        self._path_pub = self.create_publisher(PathMsg, self.get_parameter("path_topic").value, 10)
        self._repub_timer = self.create_timer(1.0, self._republish)

        self._bridge_url: Optional[str] = None
        self.get_logger().info("QCar Web Dashboard node ready.")

    # ── ROS callbacks ───────────────────────────────────────────────
    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (p.x, p.y, yaw)

    def _on_vel(self, msg: Twist):
        with self._lock:
            self._vel_history.append((time.time(), msg.linear.x, msg.angular.z))

    def _on_path(self, msg: PathMsg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path_points = pts

    def _on_csi(self, msg: ImageMsg):
        try:
            frame = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frame_csi = frame
        except Exception:
            pass

    def _on_color(self, msg: ImageMsg):
        try:
            frame = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frame_color = frame
        except Exception:
            pass

    def _on_depth(self, msg: ImageMsg):
        try:
            frame = _bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._lock:
                self._frame_depth_raw = frame
        except Exception:
            pass

    def _on_map(self, msg: OccupancyGrid):
        """Render OccupancyGrid to a BGR canvas, used when QLabs overhead is unavailable."""
        try:
            info = msg.info
            res  = float(info.resolution)
            w_c  = int(info.width)
            h_c  = int(info.height)
            ox   = float(info.origin.position.x)
            oy   = float(info.origin.position.y)

            if w_c == 0 or h_c == 0 or res <= 0:
                return

            # ── Build colour image from OG values ──────────────────────
            data = np.array(msg.data, dtype=np.int8).reshape(h_c, w_c)
            img  = np.full((h_c, w_c, 3), (40, 44, 52), dtype=np.uint8)   # unknown = dark
            img[data == 0]  = (210, 210, 215)   # free space = light
            img[data > 0]   = (30,  25,  20)    # occupied   = very dark

            # ROS OccupancyGrid row 0 = south (+Y), flip to match screen coords
            img = img[::-1, :, :].copy()

            # ── Resize to canvas dimensions ─────────────────────────────
            canvas_img = cv2.resize(img, (self._img_w, self._img_h),
                                    interpolation=cv2.INTER_NEAREST)

            # ── Update coordinate system for overlays ───────────────────
            # mpp: how many world-meters per canvas pixel
            mpp_x = (w_c * res) / self._img_w
            mpp_y = (h_c * res) / self._img_h
            new_mpp = max(mpp_x, mpp_y)
            new_cx  = ox + w_c * res / 2.0
            new_cy  = oy + h_c * res / 2.0

            with self._lock:
                if self._frame_overhead is None:   # don't override when QLabs active
                    self._mpp        = new_mpp
                    self._overlay_cx = new_cx
                    self._overlay_cy = new_cy

            with self._lock:
                self._occ_grid_img = canvas_img

            self.get_logger().info(
                "[dashboard] /map rendered: %dx%d cells  res=%.3f  mpp=%.4f" %
                (w_c, h_c, res, new_mpp)
            )
        except Exception as e:
            self.get_logger().warn("[dashboard] _on_map error: %s" % e)

    # ── Republish waypoints ─────────────────────────────────────────
    def _republish(self):
        with self._lock:
            if not self._path_sent or len(self._waypoints) < 2:
                return
            wps = list(self._waypoints)
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value
        for x, y in wps:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self._path_pub.publish(path)

    # ── QLabs overhead ──────────────────────────────────────────────
    def setup_qlabs(self):
        if not _QLABS_OK:
            self.get_logger().warn(
                "qvl not importable (quanser SDK absent from Isaac ROS container). "
                "Starting HTTP bridge poller instead — run qlabs_overhead_bridge.py on the host.")
        if not self._use_qlabs or not _QLABS_OK:
            self._use_qlabs = False
            # Start bridge poller as fallback
            self._start_bridge_poller()
            return
        try:
            host = self.get_parameter("qlabs_host").value
            loc = list(self.get_parameter("qlabs_camera_location").value)
            rot = list(self.get_parameter("qlabs_camera_rotation_deg").value)
            self.get_logger().info("Connecting to QLabs @ %s ..." % host)
            conn = QuanserInteractiveLabs()
            if not conn.open(host):
                self.get_logger().error("Could not connect to QLabs.")
                self._use_qlabs = False
                return
            cam = QLabsFreeCamera(conn)
            cam.spawn_degrees(location=loc, rotation=rot)
            cam.set_image_capture_resolution(self._img_w, self._img_h)
            self._qlabs_cam = cam
            self._overhead_stop = False
            fps = max(10, min(30, int(self.get_parameter("overhead_fps").value)))
            dt = 1.0 / fps

            def _loop():
                while not self._overhead_stop and rclpy.ok():
                    try:
                        ok, img = cam.get_image()
                        if ok and img is not None:
                            with self._lock:
                                self._frame_overhead = img
                    except Exception:
                        pass
                    time.sleep(dt)

            t = threading.Thread(target=_loop, daemon=True)
            t.start()
            self.get_logger().info("QLabs overhead live feed active.")
        except Exception as e:
            self.get_logger().error("QLabs setup failed: %s" % e)
            self._use_qlabs = False

    def _start_bridge_poller(self):
        """Poll the qlabs_overhead_bridge.py HTTP server for overhead frames.
        Falls back gracefully if the bridge isn't running — the OccupancyGrid
        fallback will still show the Cartographer map.
        """
        host = str(self.get_parameter("overhead_bridge_host").value)
        port = int(self.get_parameter("overhead_bridge_port").value)
        url  = "http://%s:%d/frame.jpg" % (host, port)
        self._bridge_url = url
        fps  = max(5, min(30, int(self.get_parameter("overhead_fps").value)))
        dt   = 1.0 / fps

        def _poll():
            connected = False
            while not self._overhead_stop and rclpy.ok():
                try:
                    with urllib.request.urlopen(url, timeout=0.5) as resp:
                        raw = resp.read()
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        with self._lock:
                            self._frame_overhead = img
                        if not connected:
                            self.get_logger().info(
                                "[dashboard] Bridge conectado: %s" % url)
                            connected = True
                except Exception:
                    if connected:
                        self.get_logger().warn(
                            "[dashboard] Bridge desconectado: %s" % url)
                        connected = False
                time.sleep(dt)

        t = threading.Thread(target=_poll, daemon=True)
        t.start()
        self.get_logger().info(
            "[dashboard] Overhead bridge poller iniciado → %s  "
            "(corre qlabs_overhead_bridge.py en el host para ver la pista de QLabs)" % url)

    # ── Coordinate helpers ──────────────────────────────────────────
    def _world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        px = (x - self._overlay_cx) / self._mpp + self._img_w / 2.0
        py = (y - self._overlay_cy) / self._mpp
        py = self._img_h - py
        return px, py

    def _pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        x = self._overlay_cx + (px - self._img_w / 2.0) * self._mpp
        y = self._overlay_cy + (self._img_h - py) * self._mpp
        return x, y

    def _pixel_to_display(self, px: float, py: float) -> Tuple[int, int]:
        dx = (self._img_w - px) if self._flip_h else px
        dy = (self._img_h - py) if self._flip_v else py
        return int(dx), int(dy)

    def _display_to_pixel(self, dx: float, dy: float) -> Tuple[float, float]:
        px = (self._img_w - dx) if self._flip_h else dx
        py = (self._img_h - dy) if self._flip_v else dy
        return px, py

    def world_to_display(self, x: float, y: float) -> Tuple[int, int]:
        return self._pixel_to_display(*self._world_to_pixel(x, y))

    def display_to_world(self, dx: float, dy: float) -> Tuple[float, float]:
        return self._pixel_to_world(*self._display_to_pixel(dx, dy))

    # ── Frame getters (with overlay for overhead) ───────────────────
    def get_overhead_frame(self) -> np.ndarray:
        with self._lock:
            raw = self._frame_overhead.copy() if self._frame_overhead is not None else None
            pose = self._pose
            wps = list(self._waypoints)
            path = list(self._path_points)

        if raw is None:
            # ── Fallback: use Cartographer OccupancyGrid if available ─
            with self._lock:
                occ = self._occ_grid_img.copy() if self._occ_grid_img is not None else None
            if occ is not None:
                frame = occ
            else:
                frame = _no_signal(self._img_w, self._img_h, "WAITING FOR QLABS BRIDGE...")
                lines = [
                    "Para ver la pista de QLabs en tiempo real ejecuta en el host:",
                    "  cd Development/python_resources",
                    "  python3 qlabs_overhead_bridge.py",
                    "",
                    "Alternativa: lanza Cartographer para recibir /map (OccupancyGrid)",
                ]
                for i, line in enumerate(lines):
                    cv2.putText(frame, line,
                                (20, self._img_h // 2 + 50 + i * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 140, 160), 1, cv2.LINE_AA)
        else:
            # cv2.imdecode (used by QLabsFreeCamera.get_image) already returns BGR.
            # Just copy and apply optional flips — do NOT reverse channels.
            frame = raw.copy()
            if self._flip_v:
                frame = frame[::-1, :].copy()
            if self._flip_h:
                frame = frame[:, ::-1].copy()

        # Draw path (green)
        if len(path) >= 2:
            pts = [self.world_to_display(x, y) for x, y in path]
            for i in range(1, len(pts)):
                cv2.line(frame, pts[i - 1], pts[i], (0, 214, 166), 2, cv2.LINE_AA)
            for pt in pts:
                cv2.circle(frame, pt, 3, (0, 214, 166), -1, cv2.LINE_AA)

        # Draw waypoints (amber diamonds)
        for i, (wx, wy) in enumerate(wps):
            dx, dy = self.world_to_display(wx, wy)
            cv2.drawMarker(frame, (dx, dy), (0, 190, 255), cv2.MARKER_DIAMOND, 14, 2, cv2.LINE_AA)
            cv2.putText(frame, str(i + 1), (dx + 10, dy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 190, 255), 1, cv2.LINE_AA)
        # Waypoint connecting line (dashed effect)
        if len(wps) >= 2:
            pts = [self.world_to_display(x, y) for x, y in wps]
            for i in range(1, len(pts)):
                cv2.line(frame, pts[i - 1], pts[i], (0, 150, 220), 1, cv2.LINE_AA)

        # Draw QCar (cyan triangle)
        if pose is not None:
            x, y, theta = pose
            cx, cy = self.world_to_display(x, y)
            L, W = 16, 8
            # In display coords, theta direction depends on flips.
            # We compute triangle vertices in world, then convert each to display.
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            scale = self._mpp * 16  # world-space length
            front = (x + scale * cos_t, y + scale * sin_t)
            bl = (x - scale * cos_t + scale * 0.5 * sin_t,
                  y - scale * sin_t - scale * 0.5 * cos_t)
            br = (x - scale * cos_t - scale * 0.5 * sin_t,
                  y - scale * sin_t + scale * 0.5 * cos_t)
            tri = np.array([
                self.world_to_display(*front),
                self.world_to_display(*bl),
                self.world_to_display(*br),
            ], dtype=np.int32)
            cv2.fillPoly(frame, [tri], (255, 212, 0), cv2.LINE_AA)
            cv2.polylines(frame, [tri], True, (0, 255, 255), 2, cv2.LINE_AA)
            # Glow circle
            cv2.circle(frame, (cx, cy), 20, (0, 255, 255), 1, cv2.LINE_AA)

            # HUD text
            cv2.putText(frame, "x=%.3f  y=%.3f  %.1f deg" % (x, y, math.degrees(theta)),
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 212, 255), 1, cv2.LINE_AA)

        # WP count badge
        n = len(wps)
        if n > 0:
            total_dist = sum(
                math.hypot(wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1])
                for i in range(1, n)
            )
            badge = "%d WPs | %.2f m" % (n, total_dist)
        else:
            badge = "Click to add waypoints"
        cv2.putText(frame, badge, (10, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 190, 255), 1, cv2.LINE_AA)

        return frame

    def get_camera_frame(self, name: str) -> np.ndarray:
        with self._lock:
            if name == "csi":
                f = self._frame_csi
            elif name == "color":
                f = self._frame_color
            elif name == "depth":
                f = self._frame_depth_raw
            else:
                f = None

        if name == "depth":
            return _colorize_depth(f) if f is not None else _no_signal(640, 480, "DEPTH — NO SIGNAL")
        return f.copy() if f is not None else _no_signal(640, 480, "%s — NO SIGNAL" % name.upper())

    # ── Waypoint management ─────────────────────────────────────────
    def add_waypoint_display(self, dx: float, dy: float) -> dict:
        wx, wy = self.display_to_world(dx, dy)
        with self._lock:
            if len(self._waypoints) < 100:
                self._waypoints.append((wx, wy))
                n = len(self._waypoints)
        self.get_logger().info("WP %d: (%.3f, %.3f)" % (n, wx, wy))
        return self._wp_info()

    def clear_waypoints(self) -> dict:
        with self._lock:
            self._waypoints.clear()
            self._path_sent = False
        self.get_logger().info("Waypoints cleared.")
        return self._wp_info()

    def undo_waypoint(self) -> dict:
        with self._lock:
            if self._waypoints:
                removed = self._waypoints.pop()
                self.get_logger().info("Undo WP: removed (%.3f, %.3f) — %d left" %
                                        (removed[0], removed[1], len(self._waypoints)))
        return self._wp_info()

    def send_route(self) -> dict:
        with self._lock:
            wps = list(self._waypoints)
        if len(wps) < 2:
            return {"ok": False, "msg": "Need at least 2 waypoints"}
        path = PathMsg()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value
        for x, y in wps:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self._path_pub.publish(path)
        with self._lock:
            self._path_sent = True
        self.get_logger().info("Route sent: %d waypoints" % len(wps))
        return {"ok": True, "msg": "Route sent (%d pts)" % len(wps), **self._wp_info()}

    def _wp_info(self) -> dict:
        with self._lock:
            wps = list(self._waypoints)
            sent = self._path_sent
        n = len(wps)
        dist = sum(
            math.hypot(wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1])
            for i in range(1, n)
        ) if n >= 2 else 0.0
        return {"count": n, "distance": round(dist, 3), "sent": sent}

    # ── Telemetry ───────────────────────────────────────────────────
    def get_telemetry(self) -> dict:
        with self._lock:
            pose = self._pose
            vel = list(self._vel_history)
            wp_info = self._wp_info()
        v, w = 0.0, 0.0
        vel_arr, omega_arr = [], []
        if vel:
            v = vel[-1][1]
            w = vel[-1][2]
            vel_arr = [round(e[1], 4) for e in vel[-100:]]
            omega_arr = [round(e[2], 4) for e in vel[-100:]]
        return {
            "pose": {"x": round(pose[0], 4), "y": round(pose[1], 4),
                     "yaw": round(math.degrees(pose[2]), 2)} if pose else None,
            "v": round(v, 4),
            "w": round(w, 4),
            "vel_history": vel_arr,
            "omega_history": omega_arr,
            "waypoints": wp_info,
        }


# ═══════════════════════════════════════════════════════════════════
#  HTTP Server
# ═══════════════════════════════════════════════════════════════════

class _Handler(BaseHTTPRequestHandler):
    node: QCarWebDashboard = None  # type: ignore

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self._serve_html()
        elif self.path.startswith("/frame/"):
            cam = self.path.split("/frame/")[1].split("?")[0]
            self._serve_single_frame(cam)
        elif self.path.startswith("/stream/"):
            cam = self.path.split("/stream/")[1].split("?")[0]
            self._stream_mjpeg(cam)
        elif self.path == "/api/telemetry":
            data = self.node.get_telemetry()
            self._json_response(data)
        elif self.path == "/api/debug":
            n = self.node
            with n._lock:
                has_overhead = n._frame_overhead is not None
                has_occ      = n._occ_grid_img is not None
                has_csi      = n._frame_csi is not None
            d = {
                "qlabs_ok": _QLABS_OK,
                "use_qlabs": n._use_qlabs,
                "bridge_url": n._bridge_url,
                "has_overhead": has_overhead,
                "has_occ_grid": has_occ,
                "has_csi": has_csi,
                "img_w": n._img_w,
                "img_h": n._img_h,
                "mpp": n._mpp,
            }
            self._json_response(d)
        else:
            self.send_error(404)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
        except Exception:
            body = {}

        if self.path == "/api/waypoints/add":
            px = float(body.get("px", 0))
            py = float(body.get("py", 0))
            result = self.node.add_waypoint_display(px, py)
            self._json_response(result)
        elif self.path == "/api/waypoints/clear":
            result = self.node.clear_waypoints()
            self._json_response(result)
        elif self.path == "/api/waypoints/undo":
            result = self.node.undo_waypoint()
            self._json_response(result)
        elif self.path == "/api/waypoints/send":
            result = self.node.send_route()
            self._json_response(result)
        else:
            self.send_error(404)

    # ── helpers ─────────────────────────────────────────────────────
    def _json_response(self, data: dict, code: int = 200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_html(self):
        body = _HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self, cam: str):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()

        fps_map = {"overhead": self.node.get_parameter("overhead_fps").value,
                    "csi": self.node.get_parameter("camera_fps").value,
                    "color": self.node.get_parameter("camera_fps").value,
                    "depth": self.node.get_parameter("camera_fps").value}
        fps = int(fps_map.get(cam, 20))
        q = self.node._jpeg_quality
        dt = 1.0 / max(5, fps)

        while True:
            try:
                if cam == "overhead":
                    frame = self.node.get_overhead_frame()
                else:
                    frame = self.node.get_camera_frame(cam)
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
                data = jpg.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                time.sleep(dt)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def _serve_single_frame(self, cam: str):
        """Return a single JPEG frame (used for canvas polling)."""
        try:
            if cam == "overhead":
                frame = self.node.get_overhead_frame()
            else:
                frame = self.node.get_camera_frame(cam)
            if frame is None or frame.size == 0:
                frame = _no_signal(label="FRAME ERROR: None")
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            _, jpg = cv2.imencode(".jpg", frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.node._jpeg_quality])
            data = jpg.tobytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        except Exception as e:
            self.node.get_logger().error("[frame/%s] %s" % (cam, e))
            # Return an error frame instead of 500 so the canvas always renders
            try:
                err = _no_signal(640, 480, "ERROR: %s" % str(e)[:40])
                _, jpg = cv2.imencode(".jpg", err)
                data = jpg.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(data)
            except Exception:
                self.send_error(500)

    def log_message(self, fmt, *args):
        pass  # suppress per-request logs


class _ThreadedServer(ThreadingMixIn, TCPServer):
    allow_reuse_address = True
    daemon_threads = True


# ═══════════════════════════════════════════════════════════════════
#  HTML / CSS / JS  (embedded — zero external files needed)
# ═══════════════════════════════════════════════════════════════════

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QCar2 Dashboard</title>
<style>
:root{
  --bg:#0a0e14;--card:#141820;--card2:#1a2030;--border:#1e2530;
  --cyan:#00d4ff;--magenta:#ff006e;--amber:#ffbe0b;--green:#06d6a0;
  --red:#ef476f;--purple:#b388ff;--text:#e0e6f0;--text2:#8899aa;
  --text3:#4a5568;
  --font:'Segoe UI',system-ui,-apple-system,sans-serif;
  --mono:'JetBrains Mono','Cascadia Code','Fira Code','Consolas',monospace;
}
*,*::before,*::after{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:var(--bg);color:var(--text);font-family:var(--font);overflow:hidden}

/* ── Dashboard grid ─────────────────── */
.dashboard{
  display:grid;
  grid-template-rows:34px 1fr 76px;
  height:100vh; height:100dvh;
  gap:2px; padding:2px;
}

/* ── Header ─────────────────────────── */
.header{
  background:var(--card);border:1px solid var(--border);border-radius:6px;
  display:flex;align-items:center;justify-content:space-between;padding:0 14px;
}
.logo{font-size:13px;font-weight:700;font-family:var(--mono);color:var(--cyan);letter-spacing:.8px;
  text-shadow:0 0 10px rgba(0,212,255,.25)}
.status-bar{display:flex;gap:12px}
.ind{font-size:10px;font-family:var(--mono);font-weight:600;color:var(--text3);transition:color .3s}
.ind::before{content:'◉ '}
.ind.ok{color:var(--green)}.ind.warn{color:var(--amber)}.ind.err{color:var(--red)}

/* ── Content ────────────────────────── */
.content{display:grid;grid-template-columns:1.2fr 1fr;gap:2px;min-height:0}

/* ── Panel ──────────────────────────── */
.panel{
  background:var(--card);border:1px solid var(--border);border-radius:6px;
  display:flex;flex-direction:column;overflow:hidden;min-height:0;
}
.panel-hdr{
  font-size:9px;font-weight:700;font-family:var(--mono);color:var(--text2);
  padding:3px 10px;border-bottom:1px solid var(--border);letter-spacing:.8px;
  display:flex;align-items:center;gap:6px;flex-shrink:0;
}
.live-dot{
  width:6px;height:6px;background:var(--red);border-radius:50%;
  animation:pulse 1.5s ease infinite;box-shadow:0 0 4px var(--red);
}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}

/* ── Camera container ───────────────── */
.cam-wrap{flex:1;position:relative;overflow:hidden;background:#0d1117;min-height:0}
.cam-wrap img{width:100%;height:100%;object-fit:contain;display:block}
.cam-wrap canvas{width:100%;height:100%;display:block;cursor:crosshair}
.cam-wrap.clickable{cursor:crosshair}

/* ── Click ripple ───────────────────── */
.click-ripple{
  position:absolute;width:20px;height:20px;border:2px solid var(--amber);
  border-radius:50%;transform:translate(-50%,-50%);
  animation:ripple .6s ease-out forwards;pointer-events:none;z-index:5;
}
@keyframes ripple{
  0%{transform:translate(-50%,-50%) scale(.5);opacity:1}
  100%{transform:translate(-50%,-50%) scale(2.5);opacity:0}
}

/* ── Right cameras ──────────────────── */
.cameras-col{display:grid;grid-template-rows:1.2fr 1fr;gap:2px;min-height:0}
.cam-row{display:grid;grid-template-columns:1fr 1fr;gap:2px;min-height:0}

/* ── Map controls ───────────────────── */
.map-controls{
  display:flex;align-items:center;gap:6px;padding:3px 8px;
  border-top:1px solid var(--border);background:var(--card2);flex-shrink:0;
}
.wp-info{flex:1;font-size:10px;font-family:var(--mono);color:var(--amber)}
.btn{
  border:none;border-radius:4px;padding:3px 10px;font-size:10px;
  font-weight:700;font-family:var(--mono);cursor:pointer;transition:all .2s;letter-spacing:.3px;
}
.btn-send{background:#0a3d2a;color:var(--green);border:1px solid var(--green)}
.btn-send:hover{background:#0d5c3f;box-shadow:0 0 8px rgba(6,214,160,.3)}
.btn-clear{background:#3d1a1a;color:var(--red);border:1px solid rgba(239,71,111,.5)}
.btn-clear:hover{background:#5c2020}
.btn-undo{background:#1a1a3d;color:var(--purple);border:1px solid rgba(179,136,255,.5)}
.btn-undo:hover{background:#2a2a5c}

/* ── Telemetry ──────────────────────── */
.telemetry{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:2px}
.tel-card{
  background:var(--card);border:1px solid var(--border);border-radius:6px;
  padding:4px 10px;display:flex;flex-direction:column;min-height:0;overflow:hidden;
}
.tel-label{font-size:8px;font-weight:700;font-family:var(--mono);color:var(--text3);letter-spacing:.8px;margin-bottom:1px}
.tel-row{display:flex;align-items:baseline;gap:4px}
.tel-value{font-size:20px;font-weight:700;font-family:var(--mono);line-height:1.1}
.tel-unit{font-size:8px;font-family:var(--mono);color:var(--text3)}
.tel-chart{flex:1;min-height:0}
.tel-chart canvas{width:100%;height:100%;display:block}
.c-cyan{color:var(--cyan)}.c-mag{color:var(--magenta)}.c-green{color:var(--green)}.c-amber{color:var(--amber)}
</style>
</head>
<body>
<div class="dashboard">

  <!-- Header -->
  <header class="header">
    <div class="logo">&#x1F3CE;&#xFE0F; QCAR2 AUTONOMY DASHBOARD</div>
    <div class="status-bar">
      <span class="ind" id="ind-pose">POSE</span>
      <span class="ind" id="ind-qlabs">QLABS</span>
      <span class="ind" id="ind-csi">CSI</span>
      <span class="ind" id="ind-rgbd">RGBD</span>
      <span class="ind" id="ind-pp">PURSUIT</span>
    </div>
  </header>

  <!-- Content -->
  <div class="content">

    <!-- Map panel (left) -->
    <div class="panel">
      <div class="panel-hdr">&#x1F5FA;&#xFE0F; TRACK MAP <div class="live-dot"></div> LIVE</div>
      <div class="cam-wrap clickable" id="map-container">
        <canvas id="map-canvas"></canvas>
      </div>
      <div class="map-controls">
        <div class="wp-info" id="wp-info">&#x229A; Click on the track to add waypoints</div>
        <button class="btn btn-undo" onclick="undoWP()">&#x21B6; UNDO</button>
        <button class="btn btn-clear" onclick="clearWPs()">&#x2715; CLEAR</button>
        <button class="btn btn-send" onclick="sendRoute()">&#x25B6; SEND</button>
      </div>
    </div>

    <!-- Cameras panel (right) -->
    <div class="cameras-col">
      <div class="panel">
        <div class="panel-hdr">&#x1F4F7; CSI CAMERA</div>
        <div class="cam-wrap"><img src="/stream/csi" alt="CSI"></div>
      </div>
      <div class="cam-row">
        <div class="panel">
          <div class="panel-hdr">&#x1F3A8; RGBD COLOR</div>
          <div class="cam-wrap"><img src="/stream/color" alt="Color"></div>
        </div>
        <div class="panel">
          <div class="panel-hdr">&#x1F30A; RGBD DEPTH</div>
          <div class="cam-wrap"><img src="/stream/depth" alt="Depth"></div>
        </div>
      </div>
    </div>

  </div>

  <!-- Telemetry -->
  <div class="telemetry">
    <div class="tel-card">
      <div class="tel-label">LINEAR VELOCITY</div>
      <div class="tel-row">
        <div class="tel-value c-cyan" id="tv-vel">0.000</div>
        <div class="tel-unit">m/s</div>
      </div>
      <div class="tel-chart"><canvas id="chart-vel"></canvas></div>
    </div>
    <div class="tel-card">
      <div class="tel-label">ANGULAR VELOCITY</div>
      <div class="tel-row">
        <div class="tel-value c-mag" id="tv-omega">0.000</div>
        <div class="tel-unit">rad/s</div>
      </div>
      <div class="tel-chart"><canvas id="chart-omega"></canvas></div>
    </div>
    <div class="tel-card">
      <div class="tel-label">POSITION</div>
      <div class="tel-value c-green" id="tv-pos" style="font-size:14px">--.--- , --.---</div>
      <div class="tel-unit">x , y (m)</div>
      <div style="display:flex;align-items:baseline;gap:3px;margin-top:auto">
        <div id="tv-heading" style="font-size:22px;font-weight:700;font-family:var(--mono);color:var(--amber)">---</div>
        <div style="font-size:8px;color:var(--text3)">&deg;</div>
      </div>
    </div>
    <div class="tel-card">
      <div class="tel-label">ROUTE STATUS</div>
      <div class="tel-value c-amber" id="tv-wps" style="font-size:14px">0 waypoints</div>
      <div class="tel-unit" id="tv-dist">0.000 m</div>
      <div style="margin-top:auto">
        <div id="tv-sent" style="font-size:11px;font-weight:700;font-family:var(--mono);color:var(--text3)">NOT SENT</div>
      </div>
    </div>
  </div>

</div>

<script>
/* ── API helper ─────────────────────── */
async function api(method, path, body) {
  const opts = { method };
  if (body) { opts.headers = {'Content-Type':'application/json'}; opts.body = JSON.stringify(body); }
  const r = await fetch(path, opts);
  return r.json();
}

/* ── Click ripple visual feedback ───── */
function showRipple(container, x, y) {
  const el = document.createElement('div');
  el.className = 'click-ripple';
  el.style.left = x + 'px';
  el.style.top  = y + 'px';
  container.appendChild(el);
  setTimeout(() => el.remove(), 650);
}

/* ── Map canvas (polled frames + click) ─ */
const mapC   = document.getElementById('map-container');
const canvas = document.getElementById('map-canvas');
const ctx2d  = canvas.getContext('2d');
let mapNatW = 1280, mapNatH = 720;

/* Poll overhead frames onto canvas */
let mapBusy = false;
let mapErrors = 0;
function pollMap() {
  if (mapBusy) return;
  mapBusy = true;
  const img = new Image();
  img.onload = function() {
    mapErrors = 0;
    mapNatW = img.naturalWidth;
    mapNatH = img.naturalHeight;
    const rect = mapC.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) { mapBusy = false; return; }
    const dpr  = window.devicePixelRatio || 1;
    canvas.width  = rect.width  * dpr;
    canvas.height = rect.height * dpr;
    canvas.style.width  = rect.width  + 'px';
    canvas.style.height = rect.height + 'px';
    ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
    /* object-fit: contain logic */
    const imgR = mapNatW / mapNatH;
    const elR  = rect.width / rect.height;
    let dw, dh, ox, oy;
    if (elR > imgR) { dh = rect.height; dw = dh * imgR; ox = (rect.width - dw)/2; oy = 0; }
    else { dw = rect.width; dh = dw / imgR; ox = 0; oy = (rect.height - dh)/2; }
    ctx2d.clearRect(0, 0, rect.width, rect.height);
    ctx2d.drawImage(img, ox, oy, dw, dh);
    mapBusy = false;
  };
  img.onerror = function() {
    mapBusy = false;
    mapErrors++;
    /* After 5 consecutive errors, draw error message on canvas */
    if (mapErrors > 5) {
      const rect = mapC.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      canvas.style.width = rect.width + 'px';
      canvas.style.height = rect.height + 'px';
      ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx2d.fillStyle = '#1a2030';
      ctx2d.fillRect(0, 0, rect.width, rect.height);
      ctx2d.strokeStyle = '#ff006e';
      ctx2d.lineWidth = 2;
      ctx2d.strokeRect(4, 4, rect.width-8, rect.height-8);
      ctx2d.fillStyle = '#ff006e';
      ctx2d.font = '16px monospace';
      ctx2d.fillText('ERROR: /frame/overhead not responding', 20, rect.height/2 - 10);
      ctx2d.fillStyle = '#8899aa';
      ctx2d.font = '13px monospace';
      ctx2d.fillText('Check: colcon build + source install/setup.bash + restart node', 20, rect.height/2 + 20);
    }
  };
  img.src = '/frame/overhead?t=' + Date.now();
}
setInterval(pollMap, 100);
pollMap();

/* Click on canvas → add waypoint */
canvas.addEventListener('click', function(e) {
  const rect = mapC.getBoundingClientRect();
  const imgR = mapNatW / mapNatH;
  const elR  = rect.width / rect.height;
  let dw, dh, ox, oy;
  if (elR > imgR) { dh = rect.height; dw = dh * imgR; ox = (rect.width - dw)/2; oy = 0; }
  else { dw = rect.width; dh = dw / imgR; ox = 0; oy = (rect.height - dh)/2; }
  const cx = e.clientX - rect.left - ox;
  const cy = e.clientY - rect.top  - oy;
  if (cx < 0 || cy < 0 || cx > dw || cy > dh) return;
  const px = (cx / dw) * mapNatW;
  const py = (cy / dh) * mapNatH;
  showRipple(mapC, e.clientX - rect.left, e.clientY - rect.top);
  api('POST', '/api/waypoints/add', {px, py}).then(updateWpInfo).catch(console.error);
});

/* ── Buttons ────────────────────────── */
async function sendRoute() {
  const r = await api('POST', '/api/waypoints/send');
  if (r.msg) updateWpInfo(r);
}
async function clearWPs() {
  const r = await api('POST', '/api/waypoints/clear');
  updateWpInfo(r);
}
async function undoWP() {
  const r = await api('POST', '/api/waypoints/undo');
  updateWpInfo(r);
}

function updateWpInfo(d) {
  const info = document.getElementById('wp-info');
  if (d.count > 0) {
    info.textContent = '\u229A ' + d.count + ' waypoints | ' + d.distance + ' m';
  } else {
    info.textContent = '\u229A Click on the track to add waypoints';
  }
}

/* ── Telemetry polling ──────────────── */
async function pollTelemetry() {
  try {
    const d = await api('GET', '/api/telemetry');

    document.getElementById('tv-vel').textContent   = d.v.toFixed(3);
    document.getElementById('tv-omega').textContent  = d.w.toFixed(3);

    const ip = document.getElementById('ind-pose');
    if (d.pose) {
      document.getElementById('tv-pos').textContent     = d.pose.x.toFixed(3) + ' , ' + d.pose.y.toFixed(3);
      document.getElementById('tv-heading').textContent  = d.pose.yaw.toFixed(1);
      ip.className = 'ind ok';
    } else {
      document.getElementById('tv-pos').textContent     = '--.--- , --.---';
      document.getElementById('tv-heading').textContent  = '---';
      ip.className = 'ind err';
    }

    const wp = d.waypoints;
    document.getElementById('tv-wps').textContent   = wp.count + ' waypoints';
    document.getElementById('tv-dist').textContent  = wp.distance.toFixed(3) + ' m';
    document.getElementById('tv-sent').textContent  = wp.sent ? 'SENT \u2714' : 'NOT SENT';
    document.getElementById('tv-sent').style.color  = wp.sent ? 'var(--green)' : 'var(--text3)';

    const ipp = document.getElementById('ind-pp');
    if (Math.abs(d.v) > 0.01) { ipp.className = 'ind ok';   ipp.textContent = 'PURSUIT \u25B6'; }
    else if (d.vel_history.length > 0)  { ipp.className = 'ind warn'; ipp.textContent = 'PURSUIT \u23F8'; }
    else { ipp.className = 'ind'; }

    if (d.vel_history.length)   drawChart('chart-vel',   d.vel_history,   '#00d4ff');
    if (d.omega_history.length) drawChart('chart-omega', d.omega_history, '#ff006e');
  } catch(e) {}
}

/* ── Mini chart ─────────────────────── */
function drawChart(id, data, color) {
  const canvas = document.getElementById(id);
  if (!canvas) return;
  const ctx  = canvas.getContext('2d');
  const dpr  = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 10 || rect.height < 4) return;
  canvas.width  = rect.width  * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  const w = rect.width, h = rect.height;
  ctx.clearRect(0, 0, w, h);
  if (data.length < 2) return;
  let mn = Math.min(...data), mx = Math.max(...data);
  if (mx - mn < 0.01) { mn -= 0.05; mx += 0.05; }
  const range = mx - mn;

  ctx.beginPath();
  for (let i = 0; i < data.length; i++) {
    const x = (i / (data.length - 1)) * w;
    const y = h - ((data[i] - mn) / range) * (h - 3) - 1;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, color + '30');
  grad.addColorStop(1, color + '05');
  ctx.fillStyle = grad; ctx.fill();

  ctx.beginPath();
  for (let i = 0; i < data.length; i++) {
    const x = (i / (data.length - 1)) * w;
    const y = h - ((data[i] - mn) / range) * (h - 3) - 1;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
}

/* ── Stream status ──────────────────── */
function checkStreams() {
  const csi   = document.querySelector('img[alt="CSI"]');
  const color = document.querySelector('img[alt="Color"]');
  document.getElementById('ind-csi').className  = (csi   && csi.naturalWidth   > 0) ? 'ind ok' : 'ind';
  document.getElementById('ind-rgbd').className = (color && color.naturalWidth > 0) ? 'ind ok' : 'ind';
  document.getElementById('ind-qlabs').className= (mapNatW > 0 && mapErrors === 0) ? 'ind ok' : 'ind err';
}

/* ── Start ──────────────────────────── */
setInterval(pollTelemetry, 150);
setInterval(checkStreams, 2000);
pollTelemetry();
checkStreams();
</script>
</body>
</html>"""


# ═══════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = QCarWebDashboard()
    node.setup_qlabs()

    port = int(node.get_parameter("port").value)
    _Handler.node = node
    server = _ThreadedServer(("0.0.0.0", port), _Handler)

    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    node.get_logger().info(
        "\n"
        "  ┌──────────────────────────────────────────────────┐\n"
        "  │  🏎️  QCar2 Web Dashboard running                 │\n"
        "  │  Open in browser: http://localhost:%d          │\n"
        "  └──────────────────────────────────────────────────┘" % port
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._overhead_stop = True
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
