#!/usr/bin/env bash

# Source this file from an SSH shell on zlab so ROS GUI nodes open on zlab's
# physical display instead of trying to render on the SSH client.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced so it can update your current shell."
  echo "Use: source tools/zlab_ros_env.sh"
  exit 1
fi

export DISPLAY=:0
export XAUTHORITY=/home/zhang/.Xauthority

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
# shellcheck source=/dev/null
source "${script_dir}/source_migels_ros_env.sh"
