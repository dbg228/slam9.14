#!/usr/bin/env python3
"""Read localization and compute diagnostic proposals, with no motion endpoint.

Inspection tracking intentionally ignores obstacle costs, as requested.
The default tracking-only mode remains a shadow calculation. No function in
this module publishes to /cmd_vel, a velocity smoother, or a robot action.
"""
from __future__ import annotations

from dataclasses import replace
import ast
import hashlib
import json
import math
from pathlib import Path as FilePath
import sys
import time

import numpy as np

sys.path.insert(0, str(FilePath(__file__).resolve().parent))
from controller import Config, GuidedMPPI, Path

PROPOSAL_TOPIC = "/turning_mppi/proposal"
DIAGNOSTIC_TOPIC = "/turning_mppi/shadow"
ODOM_TOPIC = "/odom"
PATH_TOPIC = "/inspection_route/reference_path"


def unit_quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("nonfinite_or_invalid_quaternion")
    norm = np.linalg.norm(q)
    if norm < .99 or norm > 1.01:
        raise ValueError("quaternion_not_unit")
    return q / norm


def quaternion_product(a, b):
    av, aw, bv, bw = a[:3], a[3], b[:3], b[3]
    return np.r_[aw * bv + bw * av + np.cross(av, bv), aw * bw - av @ bv]


def quaternion_yaw(q):
    x, y, z, w = unit_quaternion(q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def transform_state(position, orientation, translation, rotation, vx, wz):
    """Apply the full map<-odom transform; retain body-frame measured speeds."""
    p, t = np.asarray(position, dtype=float), np.asarray(translation, dtype=float)
    if p.shape != (3,) or t.shape != (3,) or not np.isfinite(np.r_[p, t, vx, wz]).all():
        raise ValueError("nonfinite_or_invalid_state")
    q = unit_quaternion(rotation)
    body = unit_quaternion(orientation)
    mapped = p + 2 * np.cross(q[:3], np.cross(q[:3], p) + q[3] * p) + t
    mapped_q = quaternion_product(q, body)
    return np.array([mapped[0], mapped[1], quaternion_yaw(mapped_q), vx, wz])


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def self_test():
    # Full transform: odom (1,0), map<-odom translation (2,3), yaw +90deg.
    qz = [0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)]
    result = transform_state([1, 0, 0], [0, 0, 0, 1], [2, 3, 0], qz, .1, .2)
    assert np.allclose(result, [2, 4, math.pi / 2, .1, .2])
    for bad in ([0, 0, 0, 0], [0, 0, float("nan"), 1]):
        try:
            unit_quaternion(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid quaternion was accepted")
    assert json.dumps(json_safe({"distance": float("inf")}), allow_nan=False)
    # Verify source has exactly the two diagnostic publisher calls.
    tree = ast.parse(FilePath(__file__).read_text())
    publications = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "create_publisher"]
    assert len(publications) == 2
    assert {call.args[1].id for call in publications} == {"PROPOSAL_TOPIC", "DIAGNOSTIC_TOPIC"}
    print("shadow self-test passed: transform, body speeds, invalid inputs, JSON, output whitelist")


def main(argv=None):
    argv = sys.argv if argv is None else argv
    if "--self-test" in argv:
        self_test()
        return

    # Importing/running --self-test never initializes ROS or joins a ROS domain.
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    def xyz(value):
        return [value.x, value.y, value.z]

    def xyzw(value):
        return [value.x, value.y, value.z, value.w]

    class ShadowNode(Node):
        def __init__(self):
            super().__init__("turning_mppi_shadow")
            self.declare_parameter("allow_tracking_only_shadow", True)
            self.declare_parameter("max_odom_age", .25)
            self.declare_parameter("tf_timeout", .02)
            self.declare_parameter("seed", 0)
            self.declare_parameter("samples", 1000)
            self.cfg = replace(Config(), samples=int(self.get_parameter("samples").value))
            self.max_age = float(self.get_parameter("max_odom_age").value)
            self.tf_timeout = float(self.get_parameter("tf_timeout").value)
            if self.max_age <= 0 or self.tf_timeout < 0 or self.tf_timeout > self.cfg.dt:
                raise ValueError("Invalid freshness/TF timeout parameters")
            # Refuse remapping even the diagnostic proposal into any actuator
            # topic. The outputs have exactly two hardcoded allowed names.
            for topic in (PROPOSAL_TOPIC, DIAGNOSTIC_TOPIC):
                if self.resolve_topic_name(topic) != topic:
                    raise ValueError("Shadow output topic remapping is prohibited: " + topic)
            self.proposal = self.create_publisher(TwistStamped, PROPOSAL_TOPIC, 1)
            self.diagnostic = self.create_publisher(String, DIAGNOSTIC_TOPIC, 1)
            self.odom = None
            self.controller = None
            self.path_hash = None
            self.path_error = "waiting_for_map_reference_path"
            self.last_stamp = -1
            self.last_accepted_odom_stamp = -1
            self.odom_error = None
            self.tf = Buffer(cache_time=Duration(seconds=5))
            self.tf_listener = TransformListener(self.tf, self)
            odom_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, odom_qos)
            self.create_subscription(RosPath, PATH_TOPIC, self.on_path, path_qos)
            self.create_timer(self.cfg.dt, self.tick)
            self.get_logger().info(
                "Shadow only. Inputs /odom + map<-odom TF + reference_path. "
                "Outputs ONLY /turning_mppi/proposal and /turning_mppi/shadow. "
                "Inspection tracking only; obstacle costs intentionally disabled.")

        def on_odom(self, message):
            stamp = Time.from_msg(message.header.stamp).nanoseconds
            if stamp <= 0 or stamp <= self.last_accepted_odom_stamp:
                self.odom_error = "zero_or_nonmonotonic_odometry_stamp"
                return
            self.odom = message
            self.last_accepted_odom_stamp = stamp
            self.odom_error = None

        def on_path(self, message):
            try:
                if message.header.frame_id != "map":
                    raise ValueError("reference_path_must_be_in_map")
                points = []
                for pose in message.poses:
                    if pose.header.frame_id not in ("", "map"):
                        raise ValueError("mixed_reference_path_frames")
                    p = pose.pose.position
                    points.append([p.x, p.y, quaternion_yaw(xyzw(pose.pose.orientation))])
                path = Path(points)
                self.controller = GuidedMPPI(path, self.cfg,
                    seed=int(self.get_parameter("seed").value), mode="full")
                self.path_hash = hashlib.sha256(path.points.tobytes()).hexdigest()
                self.path_error = None
            except Exception as exc:
                self.controller = None
                self.path_error = str(exc)

        def emit(self, command, status, details=None):
            proposed = TwistStamped()
            proposed.header.frame_id = "base_link"
            if self.odom is not None:
                proposed.header.stamp = self.odom.header.stamp
            else:
                proposed.header.stamp = self.get_clock().now().to_msg()
            proposed.twist.linear.x = float(command[0])
            proposed.twist.angular.z = float(command[1])
            self.proposal.publish(proposed)
            report = {
                "mode": "shadow_diagnostic_only", "status": status,
                "actuator_connection": False, "obstacle_input_available": False,
                "obstacle_costs": "intentionally_disabled_for_inspection_tracking",
                "source_topics": {"odom": self.resolve_topic_name(ODOM_TOPIC),
                                  "path": self.resolve_topic_name(PATH_TOPIC)},
                "pose_source": "odom_pose_transformed_by_map_from_odom_at_odometry_stamp",
                "smoother_state": "assumed_zero_not_observed_live_state",
                "pending_commands": "assumed_zero_not_observed_live_queue",
                "reference_path_sha256": self.path_hash,
                "proposed_v_w": list(command),
            }
            if details:
                report.update(details)
            self.diagnostic.publish(String(data=json.dumps(json_safe(report), allow_nan=False)))

        def tick(self):
            try:
                if self.odom is None:
                    raise ValueError("waiting_for_odometry")
                if self.odom_error:
                    raise ValueError(self.odom_error)
                if self.path_error or self.controller is None:
                    raise ValueError(self.path_error or "missing_reference_path")
                odom = self.odom
                if odom.header.frame_id != "odom" or odom.child_frame_id != "base_link":
                    raise ValueError("expected_odom_to_base_link_odometry")
                stamp = Time.from_msg(odom.header.stamp)
                age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
                if not -.02 <= age <= self.max_age:
                    raise ValueError("stale_or_future_odometry")
                if stamp.nanoseconds <= self.last_stamp:
                    raise ValueError("waiting_for_new_odometry_sample")
                transform = self.tf.lookup_transform("map", "odom", stamp,
                    timeout=Duration(seconds=self.tf_timeout))
                if transform.header.frame_id != "map" or transform.child_frame_id != "odom":
                    raise ValueError("unexpected_transform_frames")
                tf_stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
                if tf_stamp_ns not in (0, stamp.nanoseconds):
                    raise ValueError("transform_not_at_odometry_stamp")
                state = transform_state(xyz(odom.pose.pose.position),
                    xyzw(odom.pose.pose.orientation), xyz(transform.transform.translation),
                    xyzw(transform.transform.rotation), odom.twist.twist.linear.x,
                    odom.twist.twist.angular.z)
                details = {
                    "odometry_stamp_ns": stamp.nanoseconds,
                    "odometry_age_s": age,
                    "tf_stamp_ns": tf_stamp_ns,
                    "map_base_state_x_y_yaw_v_w": state.tolist(),
                }
                if not bool(self.get_parameter("allow_tracking_only_shadow").value):
                    self.emit([0., 0.], "tracking_shadow_disabled_by_parameter", details)
                    return
                began = time.perf_counter()
                pending = np.zeros((round(self.cfg.delay / self.cfg.dt), 2))
                command = self.controller.command(state, smooth=np.zeros(2),
                                                   pending=pending, obstacles=[])
                elapsed = time.perf_counter() - began
                self.last_stamp = stamp.nanoseconds
                if not np.isfinite(command).all() or command.shape != (2,):
                    raise ValueError("invalid_controller_proposal")
                if elapsed > self.cfg.dt:
                    raise ValueError("shadow_compute_deadline_missed")
                if (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9 > self.max_age:
                    raise ValueError("odometry_became_stale_during_compute")
                details.update({"compute_seconds": elapsed,
                                "controller": self.controller.last_diagnostics})
                self.emit(command, "tracking_only_no_obstacle_information", details)
            except Exception as exc:
                # The zero is also a proposal on the diagnostic topic only.
                self.emit([0., 0.], "blocked", {"reason": str(exc)})

    rclpy.init(args=argv)
    node = None
    try:
        node = ShadowNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
