#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_is_finite(pose):
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
        return False
    q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return 0.5 <= q_norm <= 1.5


class GuardedInitializationGate(Node):
    """Guard manual localization initialization before it reaches laser_mapping.

    This implements the first practical safety gate inspired by guarded global
    localization:

      1. RViz publishes the operator seed on /initialpose_user.
      2. The seed is forwarded to pose_initializer as /initialpose_checked.
      3. pose_initializer publishes a refined candidate on /lidar_init_pose_candidate.
      4. The candidate is accepted only if it remains close to the operator seed.
      5. Only accepted candidates are published to /lidar_init_pose for laser_mapping.

    pose_initializer's direct /tf output is remapped away in init.launch.py, so a
    bad candidate can no longer bypass this gate and pull map->odom / odom->base_link
    into a nonsense pose.
    """

    def __init__(self):
        super().__init__("guarded_initialization_gate")

        self.declare_parameter("user_initialpose_topic", "/initialpose_user")
        self.declare_parameter("checked_initialpose_topic", "/initialpose_checked")
        self.declare_parameter("candidate_topic", "/lidar_init_pose_candidate")
        self.declare_parameter("accepted_topic", "/lidar_init_pose")
        self.declare_parameter("max_candidate_translation_m", 2.0)
        self.declare_parameter("max_candidate_yaw_rad", 1.2)
        self.declare_parameter("max_abs_z_m", 3.0)

        self.user_initialpose_topic = self.get_parameter("user_initialpose_topic").value
        self.checked_initialpose_topic = self.get_parameter("checked_initialpose_topic").value
        self.candidate_topic = self.get_parameter("candidate_topic").value
        self.accepted_topic = self.get_parameter("accepted_topic").value
        self.max_candidate_translation_m = float(
            self.get_parameter("max_candidate_translation_m").value
        )
        self.max_candidate_yaw_rad = float(self.get_parameter("max_candidate_yaw_rad").value)
        self.max_abs_z_m = float(self.get_parameter("max_abs_z_m").value)

        self.last_seed = None
        self.last_seed_yaw = None

        self.checked_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.checked_initialpose_topic, 10
        )
        self.accepted_pub = self.create_publisher(PoseStamped, self.accepted_topic, 10)

        self.create_subscription(
            PoseWithCovarianceStamped, self.user_initialpose_topic, self.on_user_seed, 10
        )
        self.create_subscription(PoseStamped, self.candidate_topic, self.on_candidate, 10)

        self.get_logger().info(
            "Guarded initialization enabled: "
            f"{self.user_initialpose_topic} -> {self.checked_initialpose_topic} -> "
            f"{self.candidate_topic} -> {self.accepted_topic}, "
            f"max_refine={self.max_candidate_translation_m:.2f}m/"
            f"{math.degrees(self.max_candidate_yaw_rad):.1f}deg"
        )

    def on_user_seed(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"Rejecting user initialpose in frame '{msg.header.frame_id}', expected 'map'"
            )
            return

        if not pose_is_finite(msg.pose.pose):
            self.get_logger().warn("Rejecting user initialpose: non-finite pose/quaternion")
            return

        if abs(msg.pose.pose.position.z) > self.max_abs_z_m:
            self.get_logger().warn(
                f"Rejecting user initialpose: |z|={abs(msg.pose.pose.position.z):.2f}m "
                f"> {self.max_abs_z_m:.2f}m"
            )
            return

        self.last_seed = msg.pose.pose
        self.last_seed_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.checked_pub.publish(msg)
        self.get_logger().info(
            f"Accepted operator seed: x={msg.pose.pose.position.x:+.3f}, "
            f"y={msg.pose.pose.position.y:+.3f}, "
            f"yaw={math.degrees(self.last_seed_yaw):+.1f}deg; waiting for refined candidate"
        )

    def on_candidate(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"Rejecting candidate in frame '{msg.header.frame_id}', expected 'map'"
            )
            return

        if self.last_seed is None:
            self.get_logger().warn("Rejecting candidate: no recent operator seed")
            return

        if not pose_is_finite(msg.pose):
            self.get_logger().warn("Rejecting candidate: non-finite pose/quaternion")
            return

        if abs(msg.pose.position.z) > self.max_abs_z_m:
            self.get_logger().warn(
                f"Rejecting candidate: |z|={abs(msg.pose.position.z):.2f}m "
                f"> {self.max_abs_z_m:.2f}m"
            )
            return

        dx = msg.pose.position.x - self.last_seed.position.x
        dy = msg.pose.position.y - self.last_seed.position.y
        dist = math.hypot(dx, dy)
        yaw = yaw_from_quat(msg.pose.orientation)
        yaw_delta = abs(normalize_angle(yaw - self.last_seed_yaw))

        if dist > self.max_candidate_translation_m:
            self.get_logger().error(
                f"Rejecting candidate: refined pose jumped {dist:.2f}m from operator seed "
                f"(limit {self.max_candidate_translation_m:.2f}m). Bad global match blocked."
            )
            return

        if yaw_delta > self.max_candidate_yaw_rad:
            self.get_logger().error(
                f"Rejecting candidate: refined yaw changed {math.degrees(yaw_delta):.1f}deg "
                f"from operator seed (limit {math.degrees(self.max_candidate_yaw_rad):.1f}deg)."
            )
            return

        self.accepted_pub.publish(msg)
        self.get_logger().info(
            f"Published safe /lidar_init_pose: x={msg.pose.position.x:+.3f}, "
            f"y={msg.pose.position.y:+.3f}, yaw={math.degrees(yaw):+.1f}deg"
        )


def main():
    rclpy.init()
    node = GuardedInitializationGate()
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
