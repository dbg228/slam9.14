#!/usr/bin/env python3
"""Normalize a raw VRPN rigid-body pose without claiming it is base_link.

The node preserves the raw VRPN topic, publishes a metres-normalized
PoseStamped in the explicitly configured mocap world frame, and emits
time-stamped validity diagnostics.  It never publishes TF or navigation data.
"""
import argparse
import copy
import math
from pathlib import Path

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def receive_clock_minus_header_ns(header_stamp_ns, receive_clock_ns):
    """Return receiver-clock minus message-header time, or None when unknown.

    This is deliberately a clock comparison rather than a claimed camera
    latency.  With a receiver-clock header it only covers ROS delivery and
    callback scheduling.  With a VRPN/server header it also includes any
    Windows--Ubuntu clock offset.
    """
    if header_stamp_ns <= 0 or receive_clock_ns <= 0:
        return None
    return receive_clock_ns - header_stamp_ns


def receive_clock_comparison_note(timestamp_source):
    """State how to interpret receive_clock_minus_header_stamp_* fields."""
    if timestamp_source == 'ubuntu_receiver_clock':
        return ('header_and_receive_clock_are_ubuntu_receiver_clock; '
                'value_is_ros_delivery_and_callback_delay_not_camera_measurement_latency')
    return ('header_clock_is_not_verified_as_ubuntu_receiver_clock; '
            'value_includes_delivery_processing_and_possible_cross_host_clock_offset; '
            'not_a_verified_camera_measurement_latency')


def normalize_pose(message, scale, output_frame):
    """Scale translation, normalize quaternion, retain the input measurement stamp."""
    values = (message.pose.position.x, message.pose.position.y, message.pose.position.z,
              message.pose.orientation.x, message.pose.orientation.y,
              message.pose.orientation.z, message.pose.orientation.w, scale)
    if not all(math.isfinite(value) for value in values) or scale <= 0.0:
        raise ValueError('nonfinite_pose_or_invalid_position_scale')
    norm = math.hypot(message.pose.orientation.x, message.pose.orientation.y,
                      message.pose.orientation.z, message.pose.orientation.w)
    if norm < 1e-9:
        raise ValueError('zero_quaternion')
    result = copy.deepcopy(message)
    result.header.frame_id = output_frame
    result.pose.position.x *= scale
    result.pose.position.y *= scale
    result.pose.position.z *= scale
    result.pose.orientation.x /= norm
    result.pose.orientation.y /= norm
    result.pose.orientation.z /= norm
    result.pose.orientation.w /= norm
    return result


class MocapPoseNormalizer(Node):
    def __init__(self, config_path):
        super().__init__('mocap_tracker0_normalizer')
        document = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
        if not isinstance(document, dict) or not isinstance(document.get('mocap'), dict) or not isinstance(document.get('normalizer'), dict):
            raise ValueError('Expected mocap and normalizer sections in '+str(config_path))
        mocap, normalizer = document['mocap'], document['normalizer']
        defaults = {
            'input_topic': normalizer['input_topic'],
            'output_topic': normalizer['output_topic'],
            'diagnostics_topic': normalizer['diagnostics_topic'],
            'expected_input_frame': normalizer['expected_input_frame'],
            'output_frame': normalizer['output_frame'],
            'position_scale_to_m': float(mocap['position_scale_to_m']),
            'stale_after_seconds': float(normalizer['stale_after_seconds']),
            'timestamp_source': str(mocap['timestamp_source']),
            'raw_position_unit': str(mocap['raw_position_unit']),
        }
        for name, default in defaults.items():
            value = self.declare_parameter(name, default).value
            if isinstance(default, str) and not value:
                raise ValueError(name+' must not be empty')
            if isinstance(default, float) and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(name+' must be finite and positive')
            setattr(self, name, value)
        if self.resolve_topic_name(self.input_topic) == self.resolve_topic_name(self.output_topic):
            raise ValueError('input_topic and output_topic must differ')
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 20)
        self.diagnostics = self.create_publisher(DiagnosticArray, self.diagnostics_topic, 20)
        self.subscription = self.create_subscription(PoseStamped, self.input_topic, self.receive, qos_profile_sensor_data)
        self.last_input_stamp_ns = 0
        self.last_valid_stamp_ns = 0
        self.last_receive_clock_ns = 0
        self.stale_reported = False
        self.timer = self.create_timer(min(self.stale_after_seconds / 2.0, 0.25), self.check_stale)
        self.get_logger().info(
            f'Passive mocap normalizer: {self.input_topic} -> {self.output_topic}; '
            f'frame={self.output_frame}; scale={self.position_scale_to_m:g}; '
            f'raw unit={self.raw_position_unit}; timestamp source={self.timestamp_source}. '
            'No TF, map alignment, rigid-body->base_link transform, NDT, or Nav2 data is published.')

    def publish_status(self, header, valid, reason, receive_clock_ns=None):
        header_stamp_ns = stamp_ns(header.stamp)
        receive_minus_header_ns = receive_clock_minus_header_ns(
            header_stamp_ns, receive_clock_ns)
        if receive_minus_header_ns is None:
            receive_clock_ns_value = 'unknown'
            receive_minus_header_ns_value = 'unknown'
            receive_minus_header_ms_value = 'unknown'
        else:
            receive_clock_ns_value = str(receive_clock_ns)
            receive_minus_header_ns_value = str(receive_minus_header_ns)
            receive_minus_header_ms_value = f'{receive_minus_header_ns / 1e6:.6f}'
        array = DiagnosticArray()
        array.header = header
        status = DiagnosticStatus()
        status.name = 'mocap_tracker0_normalizer'
        status.hardware_id = self.input_topic
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR
        status.message = reason
        status.values = [
            KeyValue(key='valid', value=str(bool(valid)).lower()),
            KeyValue(key='input_topic', value=self.input_topic),
            KeyValue(key='input_stamp_ns', value=str(header_stamp_ns)),
            KeyValue(key='receive_clock_ns', value=receive_clock_ns_value),
            KeyValue(key='receive_clock_minus_header_stamp_ns', value=receive_minus_header_ns_value),
            KeyValue(key='receive_clock_minus_header_stamp_ms', value=receive_minus_header_ms_value),
            KeyValue(key='receive_clock_comparison_note',
                     value=receive_clock_comparison_note(self.timestamp_source)),
            KeyValue(key='timestamp_source', value=self.timestamp_source),
            KeyValue(key='position_scale_to_m', value=str(self.position_scale_to_m)),
            KeyValue(key='raw_position_unit', value=self.raw_position_unit),
        ]
        array.status = [status]
        self.diagnostics.publish(array)

    def receive(self, message):
        input_stamp = stamp_ns(message.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        self.last_receive_clock_ns = now_ns
        if message.header.frame_id != self.expected_input_frame:
            self.publish_status(message.header, False, 'unexpected_input_frame:'+message.header.frame_id,
                                now_ns)
            self.get_logger().warning(
                f'Rejected raw mocap frame {message.header.frame_id!r}; expected {self.expected_input_frame!r}')
            return
        if input_stamp <= 0:
            self.publish_status(message.header, False, 'zero_timestamp', now_ns)
            return
        if input_stamp <= self.last_input_stamp_ns:
            self.publish_status(message.header, False, 'duplicate_or_backward_timestamp', now_ns)
            return
        self.last_input_stamp_ns = input_stamp
        try:
            normalized = normalize_pose(message, self.position_scale_to_m, self.output_frame)
        except ValueError as error:
            self.publish_status(message.header, False, str(error), now_ns)
            return
        self.publisher.publish(normalized)
        self.publish_status(message.header, True, 'valid', now_ns)
        self.last_valid_stamp_ns = input_stamp
        self.stale_reported = False

    def check_stale(self):
        if self.last_receive_clock_ns == 0:
            return
        age = (self.get_clock().now().nanoseconds - self.last_receive_clock_ns) / 1e9
        if age <= self.stale_after_seconds or self.stale_reported:
            return
        header = PoseStamped().header
        header.frame_id = self.output_frame
        self.publish_status(header, False, f'stream_stale_for_{age:.3f}_seconds')
        self.get_logger().error(f'Mocap stream stale for {age:.3f} s; no old pose is republished.')
        self.stale_reported = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('robot_data/config/mocap_tracker0.yaml'))
    args, unknown = parser.parse_known_args()
    rclpy.init(args=unknown)
    node = None
    try:
        node = MocapPoseNormalizer(args.config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
