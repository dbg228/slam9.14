#!/usr/bin/env bash
set -euo pipefail

LIVOX_IFACE="${LIVOX_IFACE:-enp1s0}"
# The workstation Wi-Fi is also on 192.168.1.0/24.  Use a host route for the
# direct LiDAR link so adding the LiDAR address cannot steal the Wi-Fi's whole
# local subnet route (including its gateway).  The explicit /32 route below is
# all that a single MID360 at LIVOX_LIDAR_IP needs.
LIVOX_HOST_CIDR="${LIVOX_HOST_CIDR:-192.168.1.5/32}"
LIVOX_HOST_IP="${LIVOX_HOST_CIDR%/*}"
LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.154}"

if ! ip -br addr show "${LIVOX_IFACE}" | grep -q "${LIVOX_HOST_CIDR}"; then
    echo "[livox] Configure ${LIVOX_IFACE} -> ${LIVOX_HOST_CIDR}"
    sudo ip link set "${LIVOX_IFACE}" up
    sudo ip addr replace "${LIVOX_HOST_CIDR}" dev "${LIVOX_IFACE}"
fi

sudo ip route replace "${LIVOX_LIDAR_IP}/32" dev "${LIVOX_IFACE}" src "${LIVOX_HOST_IP}"

echo "[livox] Checking lidar ${LIVOX_LIDAR_IP} ..."
lidar_ready=0
for _ in $(seq 1 10); do
    if ping -c 1 -W 1 "${LIVOX_LIDAR_IP}" >/dev/null 2>&1; then
        lidar_ready=1
        break
    fi
    sleep 1
done

if [[ "${lidar_ready}" != "1" ]]; then
    echo "[livox] ERROR: ${LIVOX_LIDAR_IP} is not reachable from ${LIVOX_IFACE}."
    echo "[livox] Check cable/power and make sure ${LIVOX_IFACE} has ${LIVOX_HOST_CIDR}."
    exit 1
fi

cd /home/zkam/test_nav
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
ros2 launch livox_ros_driver2 msg_MID360_launch.py
