#!/usr/bin/env python3
"""
Teleop con teclado para el QCar2.
Publica en /cmd_vel_nav (Twist) a tasa fija con rampa de velocidad para movimiento más fluido.
Teclas: W adelante, S atrás, A/D girar, X parar, Q salir.
"""
import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from qcar2_interfaces.msg import MotorCommands


class SimpleTeleopKeyboard(Node):
    def __init__(self, use_twist=True):
        super().__init__("simple_teleop_keyboard")
        self.use_twist = use_twist
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._ramp = 0.25       # suavizado: 0=instantáneo, 1=muy lento
        self._max_linear = 0.35
        self._max_angular = 0.5
        self._running = True

        if use_twist:
            self.pub_twist = self.create_publisher(Twist, "/cmd_vel_nav", 10)
            self.pub_motor = None
            self.get_logger().info("Publicando en /cmd_vel_nav (control fluido con rampa)")
        else:
            self.pub_twist = None
            self.pub_motor = self.create_publisher(MotorCommands, "qcar2_motor_speed_cmd", 10)
            self.get_logger().info("Publicando en qcar2_motor_speed_cmd")

        self.get_logger().info("W=adelante S=atrás A/D=girar X=parar Q=salir (movimiento suave)")

        # Publicar a tasa fija (25 Hz) con rampa para que se vea fluido
        self._timer = self.create_timer(0.04, self._timer_callback)

    def _timer_callback(self):
        # Rampa: acercar current hacia target
        self._current_linear += (self._target_linear - self._current_linear) * self._ramp
        self._current_angular += (self._target_angular - self._current_angular) * self._ramp
        # Publicar siempre (así el coche recibe comandos continuos)
        if self.use_twist and self.pub_twist:
            msg = Twist()
            msg.linear.x = float(self._current_linear)
            msg.angular.z = float(self._current_angular)
            self.pub_twist.publish(msg)
        elif self.pub_motor:
            msg = MotorCommands()
            msg.motor_names = ["steering_angle", "motor_throttle"]
            msg.values = [float(self._current_angular), float(self._current_linear)]
            self.pub_motor.publish(msg)

    def set_target(self, linear: float, angular: float):
        self._target_linear = max(-self._max_linear, min(self._max_linear, linear))
        self._target_angular = max(-self._max_angular, min(self._max_angular, angular))

    def stop_target(self):
        self._target_linear = 0.0
        self._target_angular = 0.0

    def pub_stop(self):
        self.stop_target()
        self._current_linear = 0.0
        self._current_angular = 0.0
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


def get_key_blocking():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def key_reader_thread(node: SimpleTeleopKeyboard):
    """Hilo que lee teclas y actualiza target; así el timer publica sin bloquear."""
    while node._running and rclpy.ok():
        try:
            ch = get_key_blocking()
        except (EOFError, KeyboardInterrupt):
            break
        if ch == "q" or ch == "Q":
            node._running = False
            break
        if ch == "w" or ch == "W":
            node.set_target(node._max_linear, 0.0)
        elif ch == "s" or ch == "S":
            node.set_target(-node._max_linear * 0.6, 0.0)
        elif ch == "a" or ch == "A":
            node.set_target(node._max_linear * 0.6, node._max_angular)
        elif ch == "d" or ch == "D":
            node.set_target(node._max_linear * 0.6, -node._max_angular)
        elif ch == "x" or ch == "X":
            node.stop_target()


def main(args=None):
    rclpy.init(args=args)
    use_twist = "--motor" not in (sys.argv or [])
    node = SimpleTeleopKeyboard(use_twist=use_twist)

    thread = threading.Thread(target=key_reader_thread, args=(node,))
    thread.daemon = True
    thread.start()

    try:
        while node._running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, RuntimeError):
        pass
    node._running = False
    node.pub_stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
