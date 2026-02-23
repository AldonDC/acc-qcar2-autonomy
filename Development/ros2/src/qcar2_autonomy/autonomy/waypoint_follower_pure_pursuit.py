#!/usr/bin/env python3
"""
Pure Pursuit v5 — Port directo del codigo de la competencia (smart_mobility).

El PP de la competencia (qcar_pure_pursuit.py) FUNCIONA con:
  v_ref=0.065, lookahead=0.1, k_gain=0.8, wheelbase=0.256, max_steer=0.5

Este v5 es ese mismo algoritmo, adaptado para nuestro setup:
  - Pose via TF (map -> base_link) en vez de /qcar/pose
  - Publica Twist en /cmd_vel_nav en vez de Vector3Stamped
  - Spline cubica para interpolar los 7 waypoints del dashboard
  - EKF opcional para suavizar la pose (OFF por defecto en sim)

QCar2 dimensiones:
  - Wheelbase: 0.256 m (25.6 cm, escala 1:10)
  - Max steering command: 0.5 rad
  - Max steering angle fisico: ~0.3 rad

IMPORTANTE: linear.x = motor_throttle directo (NO m/s).
            El converter (nav2_qcar_command_convert) pasa linear.x -> motor_throttle
            y angular.z -> steering_angle SIN modificar.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path


# ========================== UTILIDADES ==========================

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Normaliza angulo a [-pi, pi] (misma tecnica que la competencia)."""
    return math.atan2(math.sin(a), math.cos(a))


# ========================== SPLINE CUBICA ==========================

def _cubic_spline_coeffs(pts):
    """Coeficientes de spline cubica natural para pts [[x,y], ...]."""
    n = len(pts)
    if n < 2:
        return [], []

    def _solve_1d(vals):
        m = len(vals) - 1
        if m == 0:
            return [(vals[0], 0.0, 0.0, 0.0)]
        if m == 1:
            return [(vals[0], vals[1] - vals[0], 0.0, 0.0)]

        alpha = [0.0] * (m + 1)
        for i in range(1, m):
            alpha[i] = 3.0 * (vals[i + 1] - 2.0 * vals[i] + vals[i - 1])

        l = [1.0] * (m + 1)
        mu = [0.0] * (m + 1)
        z = [0.0] * (m + 1)

        for i in range(1, m):
            l[i] = 4.0 - mu[i - 1]
            mu[i] = 1.0 / l[i]
            z[i] = (alpha[i] - z[i - 1]) / l[i]

        c = [0.0] * (m + 1)
        b = [0.0] * m
        d = [0.0] * m
        a = [vals[i] for i in range(m)]

        for j in range(m - 1, -1, -1):
            c[j] = z[j] - mu[j] * c[j + 1]
            b[j] = (vals[j + 1] - vals[j]) - (c[j + 1] + 2.0 * c[j]) / 3.0
            d[j] = (c[j + 1] - c[j]) / 3.0

        return [(a[i], b[i], c[i], d[i]) for i in range(m)]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return _solve_1d(xs), _solve_1d(ys)


def spline_interpolate(waypoints, step=0.02):
    """Interpola waypoints con spline cubica, muestreando cada 'step' metros.
    Retorna listas cx, cy (coordenadas x,y del path denso)."""
    if len(waypoints) < 2:
        return [w[0] for w in waypoints], [w[1] for w in waypoints]

    coeffs_x, coeffs_y = _cubic_spline_coeffs(waypoints)
    n_seg = len(coeffs_x)
    if n_seg == 0:
        return [w[0] for w in waypoints], [w[1] for w in waypoints]

    # Estimar longitud de cada segmento
    FINE = 50
    seg_lengths = []
    for s in range(n_seg):
        L = 0.0
        ax, bx, cxc, dxc = coeffs_x[s]
        ay, by, cyc, dyc = coeffs_y[s]
        px0, py0 = ax, ay
        for k in range(1, FINE + 1):
            t = k / FINE
            px = ax + bx * t + cxc * t * t + dxc * t * t * t
            py = ay + by * t + cyc * t * t + dyc * t * t * t
            L += math.hypot(px - px0, py - py0)
            px0, py0 = px, py
        seg_lengths.append(L)

    total_len = sum(seg_lengths)
    if total_len < 1e-6:
        return [w[0] for w in waypoints], [w[1] for w in waypoints]

    n_total = max(10, int(math.ceil(total_len / step)))
    ds = total_len / n_total

    # Tabla acumulada de longitudes por segmento
    seg_cum = [0.0]
    for sl in seg_lengths:
        seg_cum.append(seg_cum[-1] + sl)

    cx_out, cy_out = [], []
    for k in range(n_total + 1):
        target_s = min(k * ds, total_len - 1e-9)

        # Encontrar segmento
        seg_idx = 0
        for si in range(n_seg):
            if target_s <= seg_cum[si + 1]:
                seg_idx = si
                break
        else:
            seg_idx = n_seg - 1

        # Parametro t en [0,1] dentro del segmento
        s_in = target_s - seg_cum[seg_idx]
        t = (s_in / seg_lengths[seg_idx]) if seg_lengths[seg_idx] > 1e-9 else 0.0
        t = max(0.0, min(1.0, t))
        t2, t3 = t * t, t * t * t

        ax, bx, cxc, dxc = coeffs_x[seg_idx]
        ay, by, cyc, dyc = coeffs_y[seg_idx]

        cx_out.append(ax + bx * t + cxc * t2 + dxc * t3)
        cy_out.append(ay + by * t + cyc * t2 + dyc * t3)

    return cx_out, cy_out


# ========================== EKF (OPCIONAL) ==========================

class SimpleEKF:
    """EKF con modelo bicicleta para suavizar la pose.
    Estado: [x, y, theta]. Medicion: [x, y, theta] del TF."""

    def __init__(self, wheelbase, dt,
                 q_xy=1e-4, q_theta=1e-3,
                 r_xy=1e-3, r_theta=1e-2):
        self.L = wheelbase
        self.dt = dt
        self.x = np.zeros(3)          # [x, y, theta]
        self.P = np.eye(3) * 0.01     # covarianza
        self.Q = np.diag([q_xy, q_xy, q_theta])  # ruido de proceso
        self.R = np.diag([r_xy, r_xy, r_theta])  # ruido de medicion
        self.initialized = False

    def init_state(self, x, y, theta):
        self.x = np.array([x, y, theta])
        self.P = np.eye(3) * 0.01
        self.initialized = True

    def predict(self, v_cmd, delta_cmd):
        """Prediccion con modelo bicicleta."""
        if not self.initialized:
            return
        x, y, th = self.x
        dt = self.dt

        # Modelo bicicleta
        x_new = x + v_cmd * math.cos(th) * dt
        y_new = y + v_cmd * math.sin(th) * dt
        th_new = th + (v_cmd / self.L) * math.tan(delta_cmd) * dt
        th_new = normalize_angle(th_new)

        # Jacobiano F
        F = np.eye(3)
        F[0, 2] = -v_cmd * math.sin(th) * dt
        F[1, 2] = v_cmd * math.cos(th) * dt

        self.x = np.array([x_new, y_new, th_new])
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_x, z_y, z_theta):
        """Actualizacion con medicion del TF."""
        if not self.initialized:
            self.init_state(z_x, z_y, z_theta)
            return

        z = np.array([z_x, z_y, z_theta])
        H = np.eye(3)  # medicion directa
        y = z - H @ self.x
        y[2] = normalize_angle(y[2])  # wrap angle

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    def get_state(self):
        return float(self.x[0]), float(self.x[1]), float(self.x[2])


# ========================== NODO ROS2 ==========================

class WaypointFollowerPurePursuit(Node):
    """Pure Pursuit v5 — port directo del codigo de la competencia."""

    def __init__(self):
        super().__init__("waypoint_follower_pure_pursuit")

        # --- Parametros ---
        # PP formula: delta = atan2(2*L*sin(alpha), Lf)
        # En QLabs sim, la relación throttle→velocidad (pose escalada 0.1×) es:
        #   v_real ≈ 0.12 × motor_throttle  (m/s en frame pose)
        # Con throttle=2.0: v_real ≈ 0.25 m/s → 2m en ~8s (razonable)
        # Lf = Lfc + k*v = 0.80 + 0.05*2.0 = 0.90m
        #   -> 2L/Lf = 0.57 → steering MUY suave, anti-zigzag
        # max_steer=0.3 rad limita heading rate a ~14°/s (vs 24°/s con 0.5)
        self.declare_parameter("v_ref", 2.0)                 # throttle = motor_throttle (sim needs higher)
        self.declare_parameter("lookahead", 0.80)            # Lfc base [m] (grande = suave, anti-zigzag)
        self.declare_parameter("k_gain", 0.05)               # Lf = Lfc + k*v (minimo para no inflar Lf)
        self.declare_parameter("wheelbase", 0.256)           # QCar2 = 25.6 cm
        self.declare_parameter("max_steer_cmd", 0.3)         # clamp [rad] (0.3=17°, limita oscilación)
        self.declare_parameter("steering_rate", 1.5)         # rad/s rate limiter (mas lento = menos zigzag)
        self.declare_parameter("waypoints_path_topic", "/waypoints_path")
        self.declare_parameter("pose_topic", "/qcar_pose_gt")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("interpolation_step", 0.02)   # spline cada 2cm
        self.declare_parameter("waypoint_reached_radius", 0.15)
        self.declare_parameter("brake_distance", 0.20)       # frenado final [m]
        self.declare_parameter("use_kalman", False)           # EKF off por defecto en sim

        # Cachear
        self.v_ref = self.get_parameter("v_ref").value
        self.Lfc = self.get_parameter("lookahead").value
        self.k_gain = self.get_parameter("k_gain").value
        self.L = self.get_parameter("wheelbase").value
        self.max_steer = self.get_parameter("max_steer_cmd").value
        self.steer_rate = self.get_parameter("steering_rate").value
        self.hz = self.get_parameter("control_hz").value
        self.dt = 1.0 / self.hz
        self.interp_step = self.get_parameter("interpolation_step").value
        self.wp_radius = self.get_parameter("waypoint_reached_radius").value
        self.brake_dist = self.get_parameter("brake_distance").value
        self.use_kalman = self.get_parameter("use_kalman").value
        self.max_delta_per_step = self.steer_rate * self.dt  # rad change per control step

        # --- Pub/Sub ---
        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("waypoints_path_topic").value,
            self.path_callback, 10)

        # --- Pose directa (sin TF — evita flickering del buffer TF2) ---
        self._current_pose = None  # (x, y, theta)
        self._pose_stamp = None
        self._last_valid_pose = None  # para filtro de salto
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._pose_callback, 10)

        # --- EKF (opcional) ---
        self.ekf = SimpleEKF(self.L, self.dt) if self.use_kalman else None

        # --- Estado (replica competencia: nearest_ind + target_ind separados) ---
        self.cx = []           # path x coords
        self.cy = []           # path y coords
        self.nearest_ind = None   # old_nearest_point_index (como la competencia)
        self.target_ind = 0       # monotonicamente creciente (como la competencia)
        self.last_path_hash = None
        self.route_active = False
        self._log_counter = 0
        self._last_v_cmd = 0.0
        self._last_delta_cmd = 0.0  # para rate limiter
        self._raw_waypoints = []  # para debug
        self._debug_dense_count = 0  # ciclos con log denso

        # --- VELOCITY DEBUG: medir velocidad real del carro ---
        self._prev_pose = None       # (x, y, theta, time_sec)
        self._vel_samples = []       # últimas N velocidades reales medidas
        self._total_distance = 0.0   # distancia total recorrida
        self._control_start_time = None  # cuando empezó el control
        self._initial_dist_to_wp0 = None  # distancia inicial al primer WP
        self._pose_rx_count = 0      # cuántas poses hemos recibido
        self._cmd_pub_count = 0      # cuántos Twist hemos publicado

        # Lookahead efectivo (constante ya que v_ref es constante)
        Lf_eff = self.Lfc + self.k_gain * abs(self.v_ref)
        gain = 2.0 * self.L / Lf_eff

        self.get_logger().info(
            "Pure Pursuit v9 (anti-zigzag)\n"
            f"  v_ref={self.v_ref} (throttle)  Lfc={self.Lfc}  k={self.k_gain}\n"
            f"  Lf={Lf_eff:.3f}m  2L/Lf={gain:.2f}  wheelbase={self.L}m\n"
            f"  max_steer={self.max_steer}rad  steer_rate={self.steer_rate}rad/s\n"
            f"  Hz={self.hz}  spline={self.interp_step}m  wp_r={self.wp_radius}m\n"
            f"  brake={self.brake_dist}m  EKF={'ON' if self.use_kalman else 'OFF'}\n"
            f"  Pose: DIRECT from {self.get_parameter('pose_topic').value} (no TF buffer)"
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

    # ───────── PATH CALLBACK ─────────

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return

        raw = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        path_hash = tuple(tuple(w) for w in raw)

        if path_hash == self.last_path_hash and self.route_active:
            return

        # Spline cubica -> path denso
        self.cx, self.cy = spline_interpolate(raw, step=self.interp_step)

        self.nearest_ind = None   # reset nearest (primera vez: argmin)
        self.target_ind = 0
        self._last_delta_cmd = 0.0  # reset rate limiter
        self.last_path_hash = path_hash
        self.route_active = True

        self._raw_waypoints = raw  # guardar para debug
        self._debug_dense_count = 0  # reset dense logging

        # Calcular longitud total del path
        total_len = 0.0
        for i in range(len(self.cx) - 1):
            total_len += math.hypot(self.cx[i+1] - self.cx[i],
                                    self.cy[i+1] - self.cy[i])

        # === DEBUG: Log exhaustivo de waypoints ===
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"NUEVA RUTA RECIBIDA\n"
            f"  Raw waypoints: {len(raw)}\n"
            f"  Spline points: {len(self.cx)} (step={self.interp_step}m)\n"
            f"  Path length:   {total_len:.4f} m\n"
            f"{'='*60}"
        )
        for i, w in enumerate(raw):
            self.get_logger().info(
                f"  WP[{i}]: ({w[0]:.4f}, {w[1]:.4f})"
            )
        # Distancias entre waypoints consecutivos
        for i in range(len(raw) - 1):
            d = math.hypot(raw[i+1][0] - raw[i][0], raw[i+1][1] - raw[i][1])
            self.get_logger().info(
                f"  WP[{i}]->WP[{i+1}]: dist={d:.4f}m"
            )
        # Primeros y ultimos puntos del spline
        if len(self.cx) > 4:
            self.get_logger().info(
                f"  Spline[0]:  ({self.cx[0]:.4f}, {self.cy[0]:.4f})\n"
                f"  Spline[1]:  ({self.cx[1]:.4f}, {self.cy[1]:.4f})\n"
                f"  Spline[-2]: ({self.cx[-2]:.4f}, {self.cy[-2]:.4f})\n"
                f"  Spline[-1]: ({self.cx[-1]:.4f}, {self.cy[-1]:.4f})"
            )

    # ───────── POSE ─────────

    def _pose_callback(self, msg: PoseStamped):
        """Recibe pose directa de /qcar_pose_gt. Filtra saltos imposibles."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = quaternion_to_yaw(msg.pose.orientation)
        now = self.get_clock().now().nanoseconds * 1e-9

        self._pose_rx_count += 1

        # Filtro anti-salto: rechazar si la posición salta más de 0.10m en un ciclo
        # (con throttle=0.5, velocidad real ~0.1m/s, en 33ms max 0.003m, margen amplio)
        if self._last_valid_pose is not None:
            dx = x - self._last_valid_pose[0]
            dy = y - self._last_valid_pose[1]
            jump = math.hypot(dx, dy)
            if jump > 0.10:
                if self._debug_dense_count < 100:
                    self.get_logger().warn(
                        f"POSE JUMP REJECTED: ({x:.4f},{y:.4f}) "
                        f"jump={jump:.4f}m from last valid "
                        f"({self._last_valid_pose[0]:.4f},{self._last_valid_pose[1]:.4f})")
                return

        # Medir velocidad real
        if self._prev_pose is not None:
            dt_real = now - self._prev_pose[3]
            if dt_real > 0.005:  # evitar division por 0
                dx = x - self._prev_pose[0]
                dy = y - self._prev_pose[1]
                dist_step = math.hypot(dx, dy)
                v_real = dist_step / dt_real
                self._vel_samples.append(v_real)
                if len(self._vel_samples) > 30:  # últimas 30 muestras (~1s)
                    self._vel_samples.pop(0)
                self._total_distance += dist_step

        self._prev_pose = (x, y, theta, now)
        self._current_pose = (x, y, theta)
        self._last_valid_pose = (x, y, theta)

    def get_pose(self):
        """Retorna la última pose válida, opcionalmente filtrada por EKF."""
        if self._current_pose is None:
            return None

        x, y, theta = self._current_pose

        if self.ekf is not None:
            self.ekf.predict(self._last_v_cmd, self._last_delta_cmd)
            self.ekf.update(x, y, theta)
            return self.ekf.get_state()

        return x, y, theta

    # ───────── TARGET SEARCH (replica EXACTA de la competencia) ─────────

    def search_target_index(self, x, y):
        """Replica exacta de TargetCourse.search_target_index().
        Usa nearest_ind (nearest) separado de target_ind (lookahead)."""
        n = len(self.cx)

        # 1) Buscar punto mas cercano
        if self.nearest_ind is None:
            # Primera vez: argmin global (igual que la competencia)
            best_d = float('inf')
            best_i = 0
            for i in range(n):
                d = math.hypot(self.cx[i] - x, self.cy[i] - y)
                if d < best_d:
                    best_d = d
                    best_i = i
            self.nearest_ind = best_i
        else:
            # Avanzar desde el ultimo nearest mientras distancia decrece
            ind = self.nearest_ind
            d_this = math.hypot(self.cx[ind] - x, self.cy[ind] - y)

            while ind + 1 < n:
                d_next = math.hypot(self.cx[ind + 1] - x, self.cy[ind + 1] - y)
                if d_this < d_next:
                    break  # ya estamos en el mas cercano
                ind += 1
                d_this = d_next

            self.nearest_ind = ind

        # 2) Lookahead dinamico: Lf = k * |v| + Lfc
        Lf = self.k_gain * abs(self.v_ref) + self.Lfc

        # 3) Desde el nearest, avanzar hasta que distancia >= Lf
        target_ind = self.nearest_ind
        while target_ind < n - 1:
            d = math.hypot(self.cx[target_ind] - x, self.cy[target_ind] - y)
            if d >= Lf:
                break
            target_ind += 1

        return target_ind, Lf

    # ───────── CONTROL LOOP ─────────

    def control_loop(self):
        if not self.route_active or len(self.cx) < 2:
            self.pub.publish(Twist())
            return

        pose = self.get_pose()
        if pose is None:
            self.pub.publish(Twist())
            self.get_logger().warn("NO POSE from TF!", throttle_duration_sec=2.0)
            return

        x, y, theta = pose
        n = len(self.cx)

        # --- DEBUG: al inicio, imprimir distancia a CADA waypoint raw ---
        if self._debug_dense_count == 0 and len(self._raw_waypoints) > 0:
            self._control_start_time = self.get_clock().now().nanoseconds * 1e-9
            self._initial_dist_to_wp0 = math.hypot(
                self._raw_waypoints[0][0] - x, self._raw_waypoints[0][1] - y)
            self._total_distance = 0.0
            self._vel_samples.clear()
            self._cmd_pub_count = 0

            self.get_logger().info(
                f"\n{'='*60}\n"
                f"INICIO CONTROL - Pose actual: ({x:.4f}, {y:.4f}, {math.degrees(theta):.1f}deg)\n"
                f"CMD: v_ref={self.v_ref} throttle  steer_rate={self.steer_rate} rad/s\n"
                f"Distancia a cada waypoint raw:"
            )
            for i, w in enumerate(self._raw_waypoints):
                d = math.hypot(w[0] - x, w[1] - y)
                ang = math.degrees(math.atan2(w[1] - y, w[0] - x))
                self.get_logger().info(
                    f"  WP[{i}] ({w[0]:.4f},{w[1]:.4f}): dist={d:.4f}m  dir={ang:.1f}deg"
                )
            d_start = math.hypot(self.cx[0] - x, self.cy[0] - y)
            d_end_init = math.hypot(self.cx[-1] - x, self.cy[-1] - y)
            self.get_logger().info(
                f"  Spline start: ({self.cx[0]:.4f},{self.cy[0]:.4f}) dist={d_start:.4f}m\n"
                f"  Spline end:   ({self.cx[-1]:.4f},{self.cy[-1]:.4f}) dist={d_end_init:.4f}m\n"
                f"  Pose rx count: {self._pose_rx_count}\n"
                f"{'='*60}"
            )

        # --- Check fin de ruta ---
        dist_end = math.hypot(x - self.cx[-1], y - self.cy[-1])
        nearest = self.nearest_ind if self.nearest_ind is not None else 0
        if dist_end < self.wp_radius or nearest >= n - 2:
            self.route_active = False
            self.pub.publish(Twist())
            self._last_v_cmd = 0.0
            self._last_delta_cmd = 0.0
            self.get_logger().info(
                f"\n*** RUTA COMPLETADA ***\n"
                f"  dist_end={dist_end:.4f}m (threshold={self.wp_radius}m)\n"
                f"  nearest={nearest}  target={self.target_ind}  total={n}\n"
                f"  Pose final: ({x:.4f}, {y:.4f}, {math.degrees(theta):.1f}deg)")
            return

        # --- Buscar target (identico a competencia) ---
        ind, Lf = self.search_target_index(x, y)

        # Monotonic guard (identico a compute_pure_pursuit_delta de la competencia)
        old_ind = ind
        if self.target_ind >= ind:
            ind = self.target_ind
        self.target_ind = ind
        ind = min(ind, n - 1)

        tx = self.cx[ind]
        ty = self.cy[ind]

        # --- Pure Pursuit (identico a competencia) ---
        # alpha = angulo al target relativo al heading del carro
        alpha = math.atan2(ty - y, tx - x) - theta
        alpha = normalize_angle(alpha)

        # delta = atan2(2 * L * sin(alpha), Lf)
        Lf_safe = max(Lf, 1e-3)
        delta_raw = math.atan2(2.0 * self.L * math.sin(alpha), Lf_safe)

        # Clamp steering
        delta_clamped = max(-self.max_steer, min(self.max_steer, delta_raw))

        # --- Rate limiter (simula inercia del servo) ---
        delta = delta_clamped
        delta_change = delta - self._last_delta_cmd
        rate_limited = False
        if abs(delta_change) > self.max_delta_per_step:
            delta = self._last_delta_cmd + self.max_delta_per_step * (
                1.0 if delta_change > 0 else -1.0)
            rate_limited = True

        # --- Cross-track error ---
        d_nearest = math.hypot(self.cx[self.nearest_ind] - x, self.cy[self.nearest_ind] - y)
        d_target = math.hypot(tx - x, ty - y)

        # --- Velocidad ---
        v = self.v_ref

        # Frenado suave al final de la ruta
        if dist_end < self.brake_dist:
            v = self.v_ref * max(0.3, dist_end / self.brake_dist)

        # --- Publicar ---
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(delta)
        self.pub.publish(msg)
        self._cmd_pub_count += 1

        # Guardar para EKF prediction
        self._last_v_cmd = v
        self._last_delta_cmd = delta

        # --- DEBUG LOGGING ---
        self._log_counter += 1
        self._debug_dense_count += 1

        # Calcular velocidad real promedio
        v_real_avg = 0.0
        if self._vel_samples:
            v_real_avg = sum(self._vel_samples) / len(self._vel_samples)

        # Calcular tiempo transcurrido y progreso
        elapsed = 0.0
        if self._control_start_time:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._control_start_time
        
        # Progreso: distancia recorrida vs distancia al WP0 original
        progress_pct = 0.0
        if self._initial_dist_to_wp0 and self._initial_dist_to_wp0 > 0:
            dist_to_wp0_now = math.hypot(
                self._raw_waypoints[0][0] - x, self._raw_waypoints[0][1] - y) if self._raw_waypoints else 0
            progress_pct = (1.0 - dist_to_wp0_now / self._initial_dist_to_wp0) * 100.0

        # Primeros 10 ciclos: log CADA ciclo
        # Despues: cada 1s
        do_log = False
        if self._debug_dense_count <= 10:
            do_log = True
        elif self._log_counter % max(1, int(self.hz * 1.0)) == 0:
            do_log = True

        if do_log:
            heading_to_target = math.degrees(math.atan2(ty - y, tx - x))
            # ETA al primer waypoint basado en velocidad real
            eta_str = "INF"
            if v_real_avg > 0.001:
                dist_to_wp0_now = math.hypot(
                    self._raw_waypoints[0][0] - x, self._raw_waypoints[0][1] - y) if self._raw_waypoints else dist_end
                eta = dist_to_wp0_now / v_real_avg
                eta_str = f"{eta:.1f}s"

            self.get_logger().info(
                f"[{self._debug_dense_count:4d}] t={elapsed:.1f}s "
                f"pos=({x:.4f},{y:.4f}) th={math.degrees(theta):.1f}  "
                f"tgt[{ind}/{n}]=({tx:.4f},{ty:.4f})  "
                f"d_near={d_nearest:.4f} d_tgt={d_target:.4f}  "
                f"alpha={math.degrees(alpha):.1f} delta={math.degrees(delta):.1f}{'*RL' if rate_limited else ''}  "
                f"v_cmd={v:.3f} v_REAL={v_real_avg:.4f}m/s  "
                f"dist_moved={self._total_distance:.3f}m  progress={progress_pct:.1f}%  "
                f"ETA_wp0={eta_str}  near={self.nearest_ind} d_end={dist_end:.3f}  "
                f"cmds={self._cmd_pub_count} poses={self._pose_rx_count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Stop car on exit
    try:
        if rclpy.ok():
            node.pub.publish(Twist())
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
