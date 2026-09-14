#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTfBridge(Node):
    """Broadcast TF from a nav_msgs/Odometry pose.

    The localization stack publishes /Odometry with frame_id=map and
    child_frame_id=base_link, but Nav2 requires the same relation in /tf.
    This bridge keeps the TF tree connected without changing chassis control.
    """

    def __init__(self):
        super().__init__("odom_to_tf_bridge")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")
        self.declare_parameter("stamp_offset", 0.05)
        self.declare_parameter("publish_rate", 50.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.stamp_offset = float(self.get_parameter("stamp_offset").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.count = 0
        self.latest_odom = None
        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_latest)

        self.get_logger().info(
            f"Odometry TF bridge: {self.odom_topic} -> /tf "
            f"(parent='{self.parent_frame or '<odom.header.frame_id>'}', "
            f"child='{self.child_frame or '<odom.child_frame_id>'}', "
            f"stamp_offset={self.stamp_offset:.3f}s, "
            f"publish_rate={self.publish_rate:.1f}Hz)"
        )

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def publish_latest(self):
        if self.latest_odom is None:
            return

        msg = self.latest_odom
        parent = self.parent_frame or msg.header.frame_id or "map"
        child = self.child_frame or msg.child_frame_id or "base_link"

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.stamp_offset)).to_msg()
        t.child_frame_id = child

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

        self.count += 1
        if self.count == 1 or self.count % 100 == 0:
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.get_logger().info(
                f"broadcast {parent}->{child}: "
                f"x={t.transform.translation.x:+.3f}, "
                f"y={t.transform.translation.y:+.3f}, "
                f"yaw={yaw:+.3f} rad"
            )


def main():
    rclpy.init()
    node = OdomToTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
