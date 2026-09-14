#!/usr/bin/env python3
"""Fallback Livox CustomMsg -> LaserScan converter for Nav2 obstacle layers.

The installed robot_navigation livox_time_sync_node/livox_to_scan_node binaries
crash on this machine.  This node intentionally keeps the conversion simple:
it projects Livox MID360 points within a configurable z band into a 2D
LaserScan and publishes /scan.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import LaserScan
from livox_ros_driver2.msg import CustomMsg


class LivoxToScanFallback(Node):
    def __init__(self):
        super().__init__("livox_to_scan_fallback")

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", 0.0058)
        self.declare_parameter("range_min", 0.50)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("z_min", -0.15)
        self.declare_parameter("z_max", 0.20)
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("stamp_offset", 0.0)

        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stamp_offset = float(self.get_parameter("stamp_offset").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.bin_count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        scan_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, output_topic, scan_pub_qos)
        self.sub = self.create_subscription(CustomMsg, input_topic, self.callback, qos)

        self.get_logger().info(
            f"Livox fallback scan: {input_topic} -> {output_topic}, "
            f"frame={self.frame_id}, z=[{self.z_min}, {self.z_max}], "
            f"range=[{self.range_min}, {self.range_max}], bins={self.bin_count}, "
            f"stamp_offset={self.stamp_offset:.3f}s"
        )

    def callback(self, msg: CustomMsg):
        scan = LaserScan()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        ranges = [math.inf] * self.bin_count
        intensities = [0.0] * self.bin_count

        for p in msg.points:
            if p.z < self.z_min or p.z > self.z_max:
                continue
            r = math.hypot(p.x, p.y)
            if r < self.range_min or r > self.range_max:
                continue
            angle = math.atan2(p.y, p.x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.bin_count and r < ranges[idx]:
                ranges[idx] = r
                intensities[idx] = float(p.reflectivity)

        scan.ranges = ranges
        scan.intensities = intensities
        if rclpy.ok():
            try:
                # Nav2 costmaps consume LaserScan through TF message filters.
                # The Livox CustomMsg stamp can lag behind the localization TF
                # buffer on this robot, and Python projection time can vary.
                # Stamp immediately before publishing so scan and TF are in the
                # same time window.
                stamp = self.get_clock().now() + Duration(seconds=self.stamp_offset)
                scan.header.stamp = stamp.to_msg()
                self.pub.publish(scan)
            except RCLError:
                pass


def main():
    rclpy.init()
    node = LivoxToScanFallback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
