#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from geometry_msgs.msg import TransformStamped


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def pose_is_valid(pose, max_xy_norm, max_abs_z):
    q = pose.orientation
    values = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        q.x,
        q.y,
        q.z,
        q.w,
    ]
    if not all(math.isfinite(v) for v in values):
        return False, "non-finite pose/quaternion"
    q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not 0.5 <= q_norm <= 1.5:
        return False, f"bad quaternion norm {q_norm:.3f}"
    xy_norm = math.hypot(pose.position.x, pose.position.y)
    if xy_norm > max_xy_norm:
        return False, f"xy norm {xy_norm:.1f}m > {max_xy_norm:.1f}m"
    if abs(pose.position.z) > max_abs_z:
        return False, f"|z| {abs(pose.position.z):.1f}m > {max_abs_z:.1f}m"
    return True, ""


class MapToOdomBridge(Node):
    """Publish the standard map -> odom TF from localization global odometry.

    lio_localization publishes a global LiDAR/IMU body pose as /Odometry:

        header.frame_id = map
        child_frame_id  = base_link  (hardcoded by the binary, but this is
                                      treated here as the sensor body pose)

    A separate lio_tf_to_base_tf.py node converts the raw local TF into:

        odom -> base_link

    Nav2 expects both parts of the standard tree:

        map -> odom -> base_link

    This node computes:

        T_map_base = T_map_sensor * inverse(T_base_sensor)
        T_map_odom = T_map_base   * inverse(T_odom_base)

    and publishes only map -> odom. It includes sanity checks so obviously
    diverged global poses do not pull the TF tree thousands of meters away.
    """

    def __init__(self):
        super().__init__("map_to_odom_bridge")

        self.declare_parameter("global_odom_topic", "/Odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_pose_child_frame", "base_link")
        self.declare_parameter("sensor_x", 0.37)
        self.declare_parameter("sensor_y", 0.28)
        self.declare_parameter("sensor_z", 0.15)
        self.declare_parameter("sensor_yaw", 0.8229227423153264)
        self.declare_parameter("lookup_timeout", 0.05)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("max_global_xy_norm", 100.0)
        self.declare_parameter("max_abs_z", 5.0)

        self.global_odom_topic = self.get_parameter("global_odom_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.global_pose_child_frame = self.get_parameter("global_pose_child_frame").value
        self.sensor_x = float(self.get_parameter("sensor_x").value)
        self.sensor_y = float(self.get_parameter("sensor_y").value)
        self.sensor_z = float(self.get_parameter("sensor_z").value)
        self.sensor_yaw = float(self.get_parameter("sensor_yaw").value)
        self.lookup_timeout = float(self.get_parameter("lookup_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.max_global_xy_norm = float(self.get_parameter("max_global_xy_norm").value)
        self.max_abs_z = float(self.get_parameter("max_abs_z").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_global_odom = None
        self.reject_count = 0
        self.publish_count = 0

        self.create_subscription(Odometry, self.global_odom_topic, self.on_global_odom, 20)
        self.create_timer(1.0 / max(self.publish_rate, 1.0), self.publish_map_to_odom)

        self.get_logger().info(
            f"Map->odom bridge: {self.global_odom_topic} sensor pose + TF "
            f"{self.odom_frame}->{self.base_frame} -> TF {self.map_frame}->{self.odom_frame}; "
            f"base->sensor=(x={self.sensor_x:+.3f}, y={self.sensor_y:+.3f}, "
            f"z={self.sensor_z:+.3f}, yaw={math.degrees(self.sensor_yaw):+.1f}deg)"
        )

    def on_global_odom(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring {self.global_odom_topic}: frame_id={msg.header.frame_id}, "
                f"expected {self.map_frame}"
            )
            return
        if msg.child_frame_id and msg.child_frame_id != self.global_pose_child_frame:
            self.get_logger().warn(
                f"Ignoring {self.global_odom_topic}: child_frame_id={msg.child_frame_id}, "
                f"expected {self.global_pose_child_frame}"
            )
            return
        ok, reason = pose_is_valid(
            msg.pose.pose, self.max_global_xy_norm, self.max_abs_z
        )
        if not ok:
            self.reject_count += 1
            if self.reject_count == 1 or self.reject_count % 50 == 0:
                self.get_logger().error(
                    f"Rejecting invalid global localization pose: {reason}. "
                    "Not updating map->odom."
                )
            return
        self.latest_global_odom = msg

    def publish_map_to_odom(self):
        if self.latest_global_odom is None:
            # Publish identity until localization has a valid global pose so RViz
            # and Nav2 have a connected tree at startup.
            self.publish_transform(0.0, 0.0, 0.0)
            return

        try:
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.lookup_timeout),
            )
        except TransformException as exc:
            if self.publish_count == 0:
                self.get_logger().warn(
                    f"Waiting for TF {self.odom_frame}->{self.base_frame}: {exc}"
                )
            return

        p_ms = self.latest_global_odom.pose.pose.position
        yaw_ms = yaw_from_quat(self.latest_global_odom.pose.pose.orientation)

        # Convert localization sensor/body pose into true chassis base pose:
        # T_map_base = T_map_sensor * inverse(T_base_sensor)
        yaw_mb = math.atan2(
            math.sin(yaw_ms - self.sensor_yaw),
            math.cos(yaw_ms - self.sensor_yaw),
        )
        cos_mb = math.cos(yaw_mb)
        sin_mb = math.sin(yaw_mb)
        x_mb = p_ms.x - (cos_mb * self.sensor_x - sin_mb * self.sensor_y)
        y_mb = p_ms.y - (sin_mb * self.sensor_x + cos_mb * self.sensor_y)

        p_ob = tf_ob.transform.translation
        yaw_ob = yaw_from_quat(tf_ob.transform.rotation)

        yaw_mo = math.atan2(math.sin(yaw_mb - yaw_ob), math.cos(yaw_mb - yaw_ob))
        cos_mo = math.cos(yaw_mo)
        sin_mo = math.sin(yaw_mo)

        x_mo = x_mb - (cos_mo * p_ob.x - sin_mo * p_ob.y)
        y_mo = y_mb - (sin_mo * p_ob.x + cos_mo * p_ob.y)

        if math.hypot(x_mo, y_mo) > self.max_global_xy_norm * 2.0:
            self.reject_count += 1
            if self.reject_count == 1 or self.reject_count % 50 == 0:
                self.get_logger().error(
                    f"Rejecting computed map->odom: x={x_mo:+.3f}, y={y_mo:+.3f}. "
                    "Localization/odom relation is not sane."
                )
            return

        self.publish_transform(x_mo, y_mo, yaw_mo)

    def publish_transform(self, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)
        self.publish_count += 1
        if self.publish_count == 1 or self.publish_count % 250 == 0:
            self.get_logger().info(
                f"TF {self.map_frame}->{self.odom_frame}: "
                f"x={x:+.3f}, y={y:+.3f}, yaw={math.degrees(yaw):+.1f}deg"
            )


def main():
    rclpy.init()
    node = MapToOdomBridge()
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
