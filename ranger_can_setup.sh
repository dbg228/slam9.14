#!/usr/bin/env bash
set -euo pipefail

SERIAL_DEV="${RANGER_CAN_SERIAL:-/dev/ttyACM0}"
CAN_IF="${RANGER_CAN_IF:-can0}"
CAN_SPEED_CODE="${RANGER_CAN_SPEED_CODE:-6}" # slcan: 6 means 500 kbit/s
CAN_UTILS_BIN="/home/zkam/test_nav/tools/can-utils/usr/bin"

if [ ! -e "${SERIAL_DEV}" ]; then
    echo "CAN serial device not found: ${SERIAL_DEV}"
    echo "Check USB-CAN adapter with: lsusb && ls -l /dev/ttyACM* /dev/ttyUSB*"
    exit 1
fi

if ip link show "${CAN_IF}" >/dev/null 2>&1; then
    echo "${CAN_IF} already exists."
    ip -details link show "${CAN_IF}"
    exit 0
fi

sudo modprobe slcan
sudo "${CAN_UTILS_BIN}/slcand" -o -c -f -s"${CAN_SPEED_CODE}" "${SERIAL_DEV}" "${CAN_IF}"
sleep 1
sudo ip link set "${CAN_IF}" up

echo "${CAN_IF} is up from ${SERIAL_DEV}"
ip -details link show "${CAN_IF}"
