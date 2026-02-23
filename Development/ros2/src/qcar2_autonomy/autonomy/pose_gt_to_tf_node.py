#!/usr/bin/env python3
"""
Publica TF map -> base_link desde /qcar_pose_gt.
Así el Pure Pursuit (y otros nodos que usan TF) pueden seguir la ruta cuando
solo se usa qcar_pose_from_qlabs_node (sin Cartographer).
"""
from __future__ import annotations
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PoseGtToTfNode(Node):
    def __init__(self):
        super().__init__("pose_gt_to_tf")
        self.declare_parameter("pose_gt_topic", "/qcar_pose_gt")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("child_frame", "base_link")

        self._br = TransformBroadcaster(self)
        self._last_pose: Optional[PoseStamped] = None
        self._sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_gt_topic").value,
            self._callback,
            10,
        )
        self._timer = self.create_timer(0.03, self._publish_tf)  # ~33 Hz
        self.get_logger().info(
            "Publicando TF map->base_link desde %s (para Pure Pursuit, etc.)"
            % self.get_parameter("pose_gt_topic").value
        )

    def _callback(self, msg: PoseStamped):
        self._last_pose = msg

    def _publish_tf(self):
        if self._last_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter("map_frame").value
        t.child_frame_id = self.get_parameter("child_frame").value
        t.transform.translation.x = self._last_pose.pose.position.x
        t.transform.translation.y = self._last_pose.pose.position.y
        t.transform.translation.z = self._last_pose.pose.position.z
        t.transform.rotation = self._last_pose.pose.orientation
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseGtToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
