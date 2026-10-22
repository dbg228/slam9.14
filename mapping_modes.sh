#!/usr/bin/env bash
# Separate profiles; no motion commands, no automatic stopping of existing nodes.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
profile="${1:-}"; action="${2:-}"; shift "$(( $#>=2 ? 2 : $# ))"
if [[ "$profile" == legacy ]]; then
  case "$action" in
    mapping) exec "$task_root/mapping.sh" "$@";;
    localization) exec "$task_root/localization.sh" "$@";;
    navigation) exec "$task_root/navigation.sh" "$@";;
    *) echo 'legacy mapping|localization|navigation' >&2; exit 2;;
  esac
fi
source /opt/ros/humble/setup.bash
source /home/zkam/fastlio_ws/install/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/tests/controller_tracking/.install/setup.bash"
source "$task_root/tests/mocap_anchored_mapping/.install/setup.bash"
if [[ "$profile" != anchored ]]; then
  echo 'Usage: ./mapping_modes.sh legacy mapping|localization|navigation' >&2
  echo '       ./mapping_modes.sh anchored prepare|check|preflight|mapping|save|inspect|project|validate|localization|checkpoints|correct|navigation ...' >&2
  exit 2
fi
case "$action" in
  mapping|localization|checkpoints|navigation)
    task_run="${1:?Provide absolute run directory}"
    if [[ "$action" == localization || "$action" == checkpoints ]]; then
      # Keep native localization within its existing data-age limit on this
      # host without changing estimator or LiDAR parameters.
      export OMP_WAIT_POLICY="${OMP_WAIT_POLICY:-PASSIVE}"
      export OMP_THREAD_LIMIT="${OMP_THREAD_LIMIT:-3}"
    fi
    exec ros2 launch mocap_anchored_mapping "anchored_${action}.launch.py" "run:=$task_run";;
  *) exec ros2 run mocap_anchored_mapping anchored_map "$action" "$@";;
esac
