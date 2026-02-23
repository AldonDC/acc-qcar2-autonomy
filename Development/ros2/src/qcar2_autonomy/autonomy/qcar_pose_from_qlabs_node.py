#!/usr/bin/env python3
"""
Publica la pose del QCar desde QLabs (get_world_transform) en /qcar_pose_gt.
La flecha en RViz puede usar esta fuente (use_pose_gt_topic:=true) para ir
a la par con el coche en QLabs, sin el retraso/desfase del TF de Cartographer.

Requisitos: ejecutar en un entorno con qvl (Quanser) y ROS2 (ej. contenedor con ambos).
QLabs devuelve ubicación en full-scale; convertimos a metros (scale_to_meters, default 0.1).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def _euler_to_quat_yaw(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


class QCarPoseFromQLabsNode(Node):
    def __init__(self):
        super().__init__("qcar_pose_from_qlabs")
        try:
            from qvl.qlabs import QuanserInteractiveLabs
            from qvl.qcar2 import QLabsQCar2
        except ImportError:
            self.get_logger().error(
                "qvl (Quanser) no está instalado. Ejecuta este nodo en un entorno con qvl y ROS2 "
                "(ej. contenedor quanser/virtual-qcar2 con ROS2, o host con ambos)."
            )
            raise RuntimeError("qvl not available")
        self._QuanserInteractiveLabs = QuanserInteractiveLabs
        self._QLabsQCar2 = QLabsQCar2

        self.declare_parameter("qlabs_host", "localhost")
        self.declare_parameter("actor_number", 0)
        self.declare_parameter("topic", "/qcar_pose_gt")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_yaw", 0.0)
        self.declare_parameter("scale_to_meters", 0.1)

        self._pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("topic").value,
            5,
        )
        host = self.get_parameter("qlabs_host").value
        self.get_logger().info("Conectando a QLabs en %s..." % host)
        self._qlabs = self._QuanserInteractiveLabs()
        if not self._qlabs.open(host):
            self.get_logger().error("No se pudo conectar a QLabs. ¿Está abierto con el escenario y el QCar?")
            raise RuntimeError("QLabs connection failed")
        self.get_logger().info("Conectado. Publicando en %s" % self.get_parameter("topic").value)
        self._qcar = self._QLabsQCar2(self._qlabs)
        self._qcar.actorNumber = self.get_parameter("actor_number").value
        rate = self.get_parameter("rate_hz").value
        self._timer = self.create_timer(1.0 / rate if rate > 0 else 0.033, self._publish_pose)

    def _publish_pose(self):
        ok, location, rotation, _ = self._qcar.get_world_transform()
        if not ok:
            return
        scale = self.get_parameter("scale_to_meters").value
        ox = self.get_parameter("offset_x").value
        oy = self.get_parameter("offset_y").value
        oyaw = self.get_parameter("offset_yaw").value
        x = location[0] * scale + ox
        y = location[1] * scale + oy
        yaw = rotation[2] + oyaw
        q = _euler_to_quat_yaw(rotation[0], rotation[1], yaw)
        msg = PoseStamped()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = location[2] * scale
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = QCarPoseFromQLabsNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
