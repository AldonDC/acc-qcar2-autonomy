#!/usr/bin/env python3
"""
Demo sencillo de navegación: avanza, para, gira un poco, avanza.
Publica en qcar2_motor_speed_cmd (MotorCommands).
"""
import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands


class SimpleDemoNav(Node):
    def __init__(self):
        super().__init__("simple_demo_nav")
        self.pub = self.create_publisher(MotorCommands, "qcar2_motor_speed_cmd", 10)
        self.get_logger().info("Demo nav: avanzar 4s -> parar 1s -> girar 2s -> avanzar 3s -> parar")
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = self.get_clock().now()
        self.state = "forward1"

    def publish_cmd(self, speed_m_s: float, steering_rad: float):
        msg = MotorCommands()
        msg.motor_names = ["steering_angle", "motor_throttle"]
        msg.values = [float(steering_rad), float(speed_m_s)]
        self.pub.publish(msg)

    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if t < 4.0:
            self.publish_cmd(0.25, 0.0)   # adelante 4 s
        elif t < 5.0:
            self.publish_cmd(0.0, 0.0)    # parar 1 s
        elif t < 7.0:
            self.publish_cmd(0.15, 0.4)   # girar ~2 s
        elif t < 10.0:
            self.publish_cmd(0.25, 0.0)   # adelante 3 s
        else:
            self.publish_cmd(0.0, 0.0)    # parar y terminar
            self.get_logger().info("Demo terminado. Ctrl+C para salir.")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_cmd(0.0, 0.0)  # parar al salir
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
