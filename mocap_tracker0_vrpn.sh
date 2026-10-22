#!/usr/bin/env bash
# Passive NOKOV/XINGYING Tracker0 VRPN receiver for this Ubuntu host.
# It never broadcasts TF or publishes any Nav2/FAST-LIO/NDT input.
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The VRPN server is the Windows WLAN address.  Its separate 10.1.1.198
# Ethernet address is used only by the motion-capture cameras and is not
# reachable from this robot's Wi-Fi network.
MOCAP_SERVER="${MOCAP_SERVER:-192.168.1.168}"
MOCAP_PORT="${MOCAP_PORT:-3883}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
MOCAP_FRAME_ID="${MOCAP_FRAME_ID:-mocap_world}"
MOCAP_RATE_HZ="${MOCAP_RATE_HZ:-100.0}"
MOCAP_USE_VRPN_TIMESTAMPS="${MOCAP_USE_VRPN_TIMESTAMPS:-false}"
VENDOR_PREFIX="${PROJECT_DIR}/.vendor/ros-humble-vrpn"

if [[ "${MOCAP_SERVER}" != "192.168.1.168" ]]; then
  echo "[mocap-tracker0] ERROR: This verified Tracker0 configuration permits Windows WLAN server 192.168.1.168, not ${MOCAP_SERVER}." >&2
  exit 2
fi
if [[ "${MOCAP_TRACKER}" != "Tracker0" ]]; then
  echo "[mocap-tracker0] ERROR: This dedicated script only permits Tracker0; use a separate explicit script for another rigid body." >&2
  exit 2
fi
if ! [[ "${MOCAP_PORT}" =~ ^[0-9]+$ ]] || (( MOCAP_PORT < 1 || MOCAP_PORT > 65535 )); then
  echo '[mocap-tracker0] ERROR: MOCAP_PORT must be 1..65535.' >&2; exit 2
fi
if ! [[ "${MOCAP_RATE_HZ}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo '[mocap-tracker0] ERROR: MOCAP_RATE_HZ must be positive decimal.' >&2; exit 2
fi
if [[ "${MOCAP_USE_VRPN_TIMESTAMPS}" != true && "${MOCAP_USE_VRPN_TIMESTAMPS}" != false ]]; then
  echo '[mocap-tracker0] ERROR: MOCAP_USE_VRPN_TIMESTAMPS must be true or false.' >&2; exit 2
fi

exec 9>/tmp/test_nav_mocap_tracker0.lock
if ! flock -n 9; then
  echo '[mocap-tracker0] ERROR: Tracker0 receiver is already running.' >&2; exit 1
fi

cd "${PROJECT_DIR}"
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"

if ros2 pkg prefix vrpn_mocap >/dev/null 2>&1; then
  CLIENT_NODE="$(ros2 pkg prefix vrpn_mocap)/lib/vrpn_mocap/client_node"
  CLIENT_SOURCE='installed ROS package'
elif [[ -x "${VENDOR_PREFIX}/opt/ros/humble/lib/vrpn_mocap/client_node" ]]; then
  CLIENT_NODE="${VENDOR_PREFIX}/opt/ros/humble/lib/vrpn_mocap/client_node"
  CLIENT_SOURCE='project-local Humble vrpn_mocap binary'
else
  echo '[mocap-tracker0] ERROR: vrpn_mocap client is unavailable; do not substitute an unverified client.' >&2
  exit 1
fi

# Read-only preflight.  It neither changes NICs nor firewall rules.  A blocked
# socket is a useful failure; continue only when the actual Ubuntu host can
# establish the VRPN TCP control connection.
if ! timeout 3 bash -c "</dev/tcp/${MOCAP_SERVER}/${MOCAP_PORT}"; then
  echo "[mocap-tracker0] ERROR: Cannot open TCP ${MOCAP_SERVER}:${MOCAP_PORT}." >&2
  echo '[mocap-tracker0] Check Windows server NIC/VRPN/port and this Ubuntu route; no network setting was changed.' >&2
  exit 1
fi

echo "[mocap-tracker0] client: ${CLIENT_SOURCE}"
echo "[mocap-tracker0] server: ${MOCAP_SERVER}:${MOCAP_PORT}; rigid body: Tracker0"
echo "[mocap-tracker0] raw: /vrpn_mocap/Tracker0/pose; normalized: /mocap/Tracker0/pose"
echo "[mocap-tracker0] use_vrpn_timestamps=${MOCAP_USE_VRPN_TIMESTAMPS}; no TF or base_link pose is published."
if [[ "${MOCAP_USE_VRPN_TIMESTAMPS}" == true ]]; then
  NORMALIZER_TIMESTAMP_SOURCE='vrpn_server_clock_unverified'
else
  NORMALIZER_TIMESTAMP_SOURCE='ubuntu_receiver_clock'
fi

children=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${children[@]}"; do kill -TERM "${child}" 2>/dev/null || true; done
  for child in "${children[@]}"; do wait "${child}" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"${CLIENT_NODE}" --ros-args \
  -r __ns:=/vrpn_mocap \
  -p server:="${MOCAP_SERVER}" \
  -p port:="${MOCAP_PORT}" \
  -p frame_id:="${MOCAP_FRAME_ID}" \
  -p update_freq:="${MOCAP_RATE_HZ}" \
  -p refresh_freq:=1.0 \
  -p use_vrpn_timestamps:="${MOCAP_USE_VRPN_TIMESTAMPS}" &
children+=("$!")

python3 "${PROJECT_DIR}/mocap_pose_normalizer.py" \
  --config "${PROJECT_DIR}/robot_data/config/mocap_tracker0.yaml" \
  --ros-args -p timestamp_source:="${NORMALIZER_TIMESTAMP_SOURCE}" &
children+=("$!")

wait -n "${children[@]}"
