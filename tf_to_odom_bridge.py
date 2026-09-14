#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TfToOdomBridge(Node):
    """Publish /odom from the existing odom -> base_link TF.

    The localization stack publishes the standard TF chain:

        map -> odom -> base_link

    but its /Odometry topic is a global pose (frame_id=map,
    child_frame_id=base_link). Nav2's controller odometry input should be the
    continuous local odometry in odom, so this bridge converts the already
    available TF odom -> base_link into nav_msgs/Odometry on /odom without
    publishing any additional TF.
    """

    def __init__(self):
        super().__init__("tf_to_odom_bridge")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("lookup_timeout", 0.05)
        self.declare_parameter("max_translation_norm", 100.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.lookup_timeout = float(self.get_parameter("lookup_timeout").value)
        self.max_translation_norm = float(self.get_parameter("max_translation_norm").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)

        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.count = 0

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.publish_odom)
        self.get_logger().info(
            f"TF -> odom bridge: TF {self.odom_frame}->{self.base_frame} "
            f"publishing {self.odom_topic} @ {self.publish_rate:.1f} Hz"
        )

    def publish_odom(self):
        try:
            tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.lookup_timeout),
            )
        except TransformException as exc:
            if self.count == 0:
                self.get_logger().warn(
                    f"Waiting for TF {self.odom_frame}->{self.base_frame}: {exc}"
                )
            return

        now = self.get_clock().now()
        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        yaw = yaw_from_quat(rot)

        translation_norm = math.hypot(tr.x, tr.y)
        if translation_norm > self.max_translation_norm:
            if self.count == 0 or self.count % 50 == 0:
                self.get_logger().error(
                    f"Rejecting invalid TF {self.odom_frame}->{self.base_frame}: "
                    f"x={tr.x:+.3f}, y={tr.y:+.3f}, norm={translation_norm:.1f} m "
                    f"> {self.max_translation_norm:.1f} m. "
                    "Localization likely diverged; restart localization and reinitialize."
                )
            self.count += 1
            return

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-4:
                dx = tr.x - self.prev_x
                dy = tr.y - self.prev_y
                dyaw = normalize_angle(yaw - self.prev_yaw)

                # Express linear velocity in base_link frame, as typical odom.
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                vx = (cos_yaw * dx + sin_yaw * dy) / dt
                vy = (-sin_yaw * dx + cos_yaw * dy) / dt
                wz = dyaw / dt

        self.prev_time = now
        self.prev_x = tr.x
        self.prev_y = tr.y
        self.prev_yaw = yaw

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = tr.x
        odom.pose.pose.position.y = tr.y
        odom.pose.pose.position.z = tr.z
        odom.pose.pose.orientation = rot
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Conservative covariance: good enough for Nav2 controller velocity
        # feedback, not intended as a fused state estimate.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.10

        self.odom_pub.publish(odom)

        self.count += 1
        if self.count == 1 or self.count % 250 == 0:
            self.get_logger().info(
                f"{self.odom_topic}: x={tr.x:+.3f}, y={tr.y:+.3f}, "
                f"yaw={yaw:+.3f}, vx={vx:+.3f}, vy={vy:+.3f}, wz={wz:+.3f}"
            )


def main():
    rclpy.init()
    node = TfToOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
