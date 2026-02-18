#!/usr/bin/env python3
"""
Teleop con teclado para el QCar2.
Publica en /cmd_vel_nav (Twist) para usar el converter, o en qcar2_motor_speed_cmd.
Teclas: W adelante, S atrás, A/D girar, X parar, Q salir.
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from qcar2_interfaces.msg import MotorCommands


class SimpleTeleopKeyboard(Node):
    def __init__(self, use_twist=True):
        super().__init__("simple_teleop_keyboard")
        self.use_twist = use_twist
        self.speed = 0.3
        self.steering = 0.5

        if use_twist:
            self.pub_twist = self.create_publisher(Twist, "/cmd_vel_nav", 10)
            self.pub_motor = None
            self.get_logger().info("Publicando en /cmd_vel_nav (usa el converter del launch)")
        else:
            self.pub_twist = None
            self.pub_motor = self.create_publisher(MotorCommands, "qcar2_motor_speed_cmd", 10)
            self.get_logger().info("Publicando en qcar2_motor_speed_cmd")

        self.get_logger().info("W=adelante S=atrás A/D=girar X=parar Q=salir")

    def pub_stop(self):
        if self.use_twist and self.pub_twist:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub_twist.publish(msg)
        elif self.pub_motor:
            msg = MotorCommands()
            msg.motor_names = ["steering_angle", "motor_throttle"]
            msg.values = [0.0, 0.0]
            self.pub_motor.publish(msg)

    def pub_cmd(self, linear: float, angular: float):
        if self.use_twist and self.pub_twist:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.pub_twist.publish(msg)
        elif self.pub_motor:
            msg = MotorCommands()
            msg.motor_names = ["steering_angle", "motor_throttle"]
            msg.values = [float(angular), float(linear)]
            self.pub_motor.publish(msg)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main(args=None):
    rclpy.init(args=args)
    use_twist = "--motor" not in (sys.argv or [])
    node = SimpleTeleopKeyboard(use_twist=use_twist)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key()
            if key == "q" or key == "Q":
                break
            if key == "w" or key == "W":
                node.pub_cmd(0.3, 0.0)
            elif key == "s" or key == "S":
                node.pub_cmd(-0.2, 0.0)
            elif key == "a" or key == "A":
                node.pub_cmd(0.2, 0.4)
            elif key == "d" or key == "D":
                node.pub_cmd(0.2, -0.4)
            elif key == "x" or key == "X":
                node.pub_stop()
    except (KeyboardInterrupt, EOFError):
        pass
    node.pub_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
