#!/usr/bin/env python3
"""Republish the latest static-map correction as a time-valid TF.

NDT can take seconds on the onboard CPU.  TF must nevertheless be available
at the timestamps of the incoming LaserScan messages, so this small separate
process broadcasts the last accepted map->odom correction at a fixed rate.
"""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class MapToOdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('static_map_tf_broadcaster')
        self.declare_parameter('input_topic', '/static_localization/map_to_odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 30.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        rate = float(self.get_parameter('publish_rate').value)
        self.latest = TransformStamped()
        self.latest.header.frame_id = self.map_frame
        self.latest.child_frame_id = self.odom_frame
        self.latest.transform.rotation.w = 1.0
        self.broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TransformStamped,
            self.get_parameter('input_topic').value,
            self.on_correction,
            qos,
        )
        self.create_timer(1.0 / max(rate, 1.0), self.publish_tf)
        self.get_logger().info(
            f'Publishing {self.map_frame}->{self.odom_frame} at {rate:.1f} Hz from fixed-map localization.'
        )

    def on_correction(self, message):
        if (message.header.frame_id != self.map_frame or
                message.child_frame_id != self.odom_frame):
            self.get_logger().warning(
                f'Ignoring correction {message.header.frame_id}->{message.child_frame_id}; '
                f'expected {self.map_frame}->{self.odom_frame}.'
            )
            return
        self.latest = copy.deepcopy(message)

    def publish_tf(self):
        message = copy.deepcopy(self.latest)
        message.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(message)


def main():
    rclpy.init()
    node = MapToOdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
