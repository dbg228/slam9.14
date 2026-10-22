"""Latest-value transport for private control heartbeats, not chassis commands.

An obsolete heartbeat/request must not be retried ahead of the current one.
Loss still trips the existing freshness watchdogs; this does not extend them.
Events and native chassis topics keep their existing reliable delivery.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def latest_value_qos(depth=1):
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
