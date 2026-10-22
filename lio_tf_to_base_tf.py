#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class LioTfToBaseTf(Node):
    """Convert localization raw odom->LiDAR-body TF into odom->base_link.

    The current localization binary hardcodes its child frame name as
    "base_link", but the pose actually corresponds to the MID360/IMU body
    frame.  The real Nav2 base_link is the chassis centre:

        base_link -> livox_frame = (sensor_x, sensor_y, sensor_z, sensor_yaw)

    Given raw:

        odom -> lio_body

    publish corrected:

        odom -> base_link
    """

    def __init__(self):
        super().__init__("lio_tf_to_base_tf")

        self.declare_parameter("raw_tf_topic", "/tf_lio_raw")
        self.declare_parameter("raw_parent_frame", "odom")
        self.declare_parameter("raw_child_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # User-measured base->sensor mount, 2026-10-16: forward/left, +47.15deg.
        self.declare_parameter("sensor_x", 0.37)
        self.declare_parameter("sensor_y", 0.28)
        self.declare_parameter("sensor_z", 0.15)
        self.declare_parameter("sensor_yaw", 0.8229227423153264)

        self.raw_tf_topic = self.get_parameter("raw_tf_topic").value
        self.raw_parent_frame = self.get_parameter("raw_parent_frame").value
        self.raw_child_frame = self.get_parameter("raw_child_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.sensor_x = float(self.get_parameter("sensor_x").value)
        self.sensor_y = float(self.get_parameter("sensor_y").value)
        self.sensor_z = float(self.get_parameter("sensor_z").value)
        self.sensor_yaw = float(self.get_parameter("sensor_yaw").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.count = 0

        self.create_subscription(TFMessage, self.raw_tf_topic, self.on_tf, 50)
        self.get_logger().info(
            f"LIO TF bridge: {self.raw_tf_topic} {self.raw_parent_frame}->{self.raw_child_frame} "
            f"as sensor, base->sensor=(x={self.sensor_x:+.3f}, y={self.sensor_y:+.3f}, "
            f"z={self.sensor_z:+.3f}, yaw={math.degrees(self.sensor_yaw):+.1f}deg) "
            f"-> /tf {self.odom_frame}->{self.base_frame}"
        )

    def on_tf(self, msg):
        for raw_tf in msg.transforms:
            if raw_tf.header.frame_id != self.raw_parent_frame:
                continue
            if raw_tf.child_frame_id != self.raw_child_frame:
                continue

            tr = raw_tf.transform.translation
            yaw_os = yaw_from_quat(raw_tf.transform.rotation)

            # T_odom_base = T_odom_sensor * inverse(T_base_sensor)
            yaw_ob = math.atan2(
                math.sin(yaw_os - self.sensor_yaw),
                math.cos(yaw_os - self.sensor_yaw),
            )
            cos_ob = math.cos(yaw_ob)
            sin_ob = math.sin(yaw_ob)
            x_ob = tr.x - (cos_ob * self.sensor_x - sin_ob * self.sensor_y)
            y_ob = tr.y - (sin_ob * self.sensor_x + cos_ob * self.sensor_y)

            tf = TransformStamped()
            tf.header.stamp = raw_tf.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x_ob
            tf.transform.translation.y = y_ob
            tf.transform.translation.z = tr.z - self.sensor_z
            qx, qy, qz, qw = quat_from_yaw(yaw_ob)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

            self.count += 1
            if self.count == 1 or self.count % 250 == 0:
                self.get_logger().info(
                    f"TF {self.odom_frame}->{self.base_frame}: "
                    f"x={x_ob:+.3f}, y={y_ob:+.3f}, yaw={math.degrees(yaw_ob):+.1f}deg"
                )


def main():
    rclpy.init()
    node = LioTfToBaseTf()
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
