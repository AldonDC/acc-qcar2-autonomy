import math
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotPoseMarkerNode(Node):
    def __init__(self):
        super().__init__("robot_pose_marker_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("marker_topic", "/qcar_pose_marker")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("circle_radius", 0.1)
        self.declare_parameter("arrow_length", 0.6)
        self.declare_parameter("log_pose_hz", 0.0)
        self.declare_parameter("enable_click_calibration", False)  # Por defecto desactivado para no mover el coche al poner waypoints
        # Calibración persistente
        self.declare_parameter("calibration_file", "")  # Ruta al YAML de calibración
        self.declare_parameter("pose_offset_x", -2.673)
        self.declare_parameter("pose_offset_y", 0.377)
        self.declare_parameter("pose_offset_theta", 0.22)

        self._pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            5,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._circle_radius = self.get_parameter("circle_radius").value
        self._arrow_length = self.get_parameter("arrow_length").value
        
        # Cargar valores iniciales de los parámetros
        self._offset_x = self.get_parameter("pose_offset_x").value
        self._offset_y = self.get_parameter("pose_offset_y").value
        self._offset_theta = self.get_parameter("pose_offset_theta").value
        
        # Intentar cargar desde archivo de calibración si existe
        self._calib_file = self.get_parameter("calibration_file").value
        if not self._calib_file:
            # Intentar ruta por defecto en el paquete
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_path = get_package_share_directory("qcar2_autonomy")
                self._calib_file = os.path.join(pkg_path, "config", "qcar_pose_calibration.yaml")
            except Exception:
                self._calib_file = "/tmp/qcar_pose_calibration.yaml"
        
        self._load_calibration()

        self._last_click = None
        self._last_click_time = None
        self._auto_calibrated = False

        self._sub_click = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self._on_clicked_point,
            5,
        )
        self._srv_calibrate = self.create_service(
            Empty,
            "calibrate_from_click",
            self._on_calibrate_from_click,
        )
        self._srv_save = self.create_service(
            Empty,
            "save_calibration",
            self._on_save_calibration,
        )
        self._srv_reset = self.create_service(
            Empty,
            "reset_calibration",
            self._on_reset_calibration,
        )

        rate = self.get_parameter("rate_hz").value
        self._timer = self.create_timer(1.0 / rate if rate > 0 else 0.1, self._publish_marker)
        log_hz = self.get_parameter("log_pose_hz").value
        if log_hz > 0:
            self._log_timer = self.create_timer(1.0 / log_hz, self._log_pose)
        else:
            self._log_timer = None

        self.get_logger().info(
            f"robot_pose_marker_node: Calibración inicial ({self._offset_x:.3f}, {self._offset_y:.3f}). "
            "Para ajustar: haz UN clic en RViz (Publish Point) donde está el QCar en QLabs. "
            "Para guardar permanentemente: ros2 service call /robot_pose_marker_node/save_calibration std_srvs/srv/Empty"
        )

    def _load_calibration(self):
        if os.path.isfile(self._calib_file):
            try:
                with open(self._calib_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        self._offset_x = data.get('pose_offset_x', self._offset_x)
                        self._offset_y = data.get('pose_offset_y', self._offset_y)
                        self._offset_theta = data.get('pose_offset_theta', self._offset_theta)
                        self.get_logger().info(f"Calibración cargada desde {self._calib_file}: ({self._offset_x:.3f}, {self._offset_y:.3f})")
            except Exception as e:
                self.get_logger().warn(f"No se pudo cargar calibración: {e}")

    def _on_save_calibration(self, _req, resp):
        try:
            os.makedirs(os.path.dirname(self._calib_file), exist_ok=True)
            data = {
                'pose_offset_x': float(self._offset_x),
                'pose_offset_y': float(self._offset_y),
                'pose_offset_theta': float(self._offset_theta)
            }
            with open(self._calib_file, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Calibración guardada en {self._calib_file}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar calibración: {e}")
        return resp

    def _on_reset_calibration(self, _req, resp):
        self._offset_x = self.get_parameter("pose_offset_x").value
        self._offset_y = self.get_parameter("pose_offset_y").value
        self._offset_theta = self.get_parameter("pose_offset_theta").value
        self._auto_calibrated = False
        self.get_logger().info("Calibración reseteada a valores por defecto del launch.")
        return resp

    def _on_clicked_point(self, msg: PointStamped):
        self._last_click = (msg.point.x, msg.point.y)
        self._last_click_time = self.get_clock().now()
        
        # Solo calibramos automáticamente si el parámetro está activo
        if self.get_parameter("enable_click_calibration").value:
            if not self._auto_calibrated:
                self._do_calibrate(msg.point.x, msg.point.y)
                self._auto_calibrated = True
                self.get_logger().info("Calibración automática (primer clic): marcador ajustado. Usa el servicio save_calibration para persistir.")
        # Ya no imprimimos mensajes en cada clic para no contaminar el log durante waypoint placement


    def _do_calibrate(self, click_x: float, click_y: float):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            self.get_logger().warn("No se pudo obtener TF para calibración. Asegúrate de que Cartographer está corriendo.")
            return
        raw_x = t.transform.translation.x
        raw_y = t.transform.translation.y
        self._offset_x = click_x - raw_x
        self._offset_y = click_y - raw_y
        self.get_logger().info(f"Nuevo offset: ({self._offset_x:.3f}, {self._offset_y:.3f})")

    def _on_calibrate_from_click(self, _req, resp):
        if self._last_click is None or self._last_click_time is None:
            self.get_logger().warn("calibrate_from_click: haz primero un clic en RViz donde está el QCar.")
            return resp
        age = (self.get_clock().now() - self._last_click_time).nanoseconds / 1e9
        if age > 15.0:
            self.get_logger().warn("calibrate_from_click: el último clic es antiguo. Haz clic de nuevo.")
            return resp
        click_x, click_y = self._last_click
        self._do_calibrate(click_x, click_y)
        return resp

    def _log_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return
        raw_x = t.transform.translation.x
        raw_y = t.transform.translation.y
        x = raw_x + self._offset_x
        y = raw_y + self._offset_y
        self.get_logger().info(
            f"Pose: raw=({raw_x:.3f}, {raw_y:.3f}) offset=({self._offset_x:.3f}, {self._offset_y:.3f}) -> marcador=({x:.3f}, {y:.3f})"
        )

    def _publish_marker(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return
        raw_x = t.transform.translation.x
        raw_y = t.transform.translation.y
        q = t.transform.rotation
        raw_yaw = quaternion_to_yaw(q)
        
        x = raw_x + self._offset_x
        y = raw_y + self._offset_y
        yaw = raw_yaw + self._offset_theta
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()

        # Círculo (esfera vista desde arriba)
        circle = Marker()
        circle.header.frame_id = self._map_frame
        circle.header.stamp = now
        circle.ns = "qcar"
        circle.id = 0
        circle.type = Marker.SPHERE
        circle.action = Marker.ADD
        circle.pose.position.x = x
        circle.pose.position.y = y
        circle.pose.position.z = 0.05
        circle.pose.orientation.w = 1.0
        circle.scale.x = 2.0 * self._circle_radius
        circle.scale.y = 2.0 * self._circle_radius
        circle.scale.z = 0.15
        circle.color.r = 0.2
        circle.color.g = 0.5
        circle.color.b = 1.0
        circle.color.a = 1.0
        ma.markers.append(circle)

        # Flecha
        arrow = Marker()
        arrow.header.frame_id = self._map_frame
        arrow.header.stamp = now
        arrow.ns = "qcar"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.08
        arrow.pose.orientation.x = 0.0
        arrow.pose.orientation.y = 0.0
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x = self._arrow_length
        arrow.scale.y = 0.15
        arrow.scale.z = 0.15
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        ma.markers.append(arrow)

        self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # El RuntimeError es común en Humble al cerrar con TF2
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

