#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_NAME="stm32_manual.launch"

usage() {
  cat <<EOF
Usage: tools/run_stm32_manual_migels.sh [roslaunch args...]

Launch sensor_data_collection ${LAUNCH_NAME} from this Mi-Gels workspace,
even when other workspaces also contain a sensor_data_collection package.

Environment:
  ROS_DISTRO       ROS distribution name. Default: noetic
  ZLAB_ROBOTS_WS   Dependency workspace. Default: \$HOME/zlab_robots
  MIGELS_RUNTIME_CONFIG
                   Runtime config path. Default: ${WS_ROOT}/migels_runtime.yaml

Examples:
  tools/run_stm32_manual_migels.sh
  tools/run_stm32_manual_migels.sh hardware_config:=ak09973d startup_sensors:=AK_ICM
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_migels_ros_env.sh"

echo "Launching: roslaunch sensor_data_collection ${LAUNCH_NAME} $*"
exec roslaunch sensor_data_collection "${LAUNCH_NAME}" "$@"
