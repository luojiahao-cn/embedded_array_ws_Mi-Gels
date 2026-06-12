#!/usr/bin/env bash
# Source this file to activate the Mi-Gels ROS environment in the current shell.

_migels_env_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

_migels_env_usage() {
  cat <<EOF
Usage:
  source tools/source_migels_ros_env.sh
  . tools/source_migels_ros_env.sh

Checks every ROS package under this workspace resolves to this source tree.

Environment:
  ROS_DISTRO       ROS distribution name. Default: noetic
  ZLAB_ROBOTS_WS   Dependency workspace. Default: \$HOME/zlab_robots
  MIGELS_RUNTIME_CONFIG
                   Runtime config path. Default: <workspace>/migels_runtime.yaml
EOF
}

_migels_env_finish() {
  local code="$1"
  if _migels_env_is_sourced; then
    return "${code}"
  fi
  exit "${code}"
}

_migels_env_source_setup() {
  local had_nounset=0
  if [[ "$-" == *u* ]]; then
    had_nounset=1
  fi
  set +u
  # shellcheck source=/dev/null
  source "$1"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  else
    set +u
  fi
}

_migels_env_abs_dir() {
  local path="$1"
  (cd "${path}" >/dev/null 2>&1 && pwd -P)
}

_migels_env_package_name() {
  sed -n 's:.*<name>[[:space:]]*\([^<][^<]*\)[[:space:]]*</name>.*:\1:p' "$1" | head -n 1
}

_migels_env_main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    _migels_env_usage
    _migels_env_finish 0
    return $?
  fi

  local script_dir
  local ws_root
  local ros_distro_name
  local ros_setup
  local zlab_robots_ws

  script_dir="$(_migels_env_abs_dir "$(dirname "${BASH_SOURCE[0]}")")"
  ws_root="$(_migels_env_abs_dir "${script_dir}/..")"
  ros_distro_name="${ROS_DISTRO:-noetic}"
  ros_setup="/opt/ros/${ros_distro_name}/setup.bash"

  if [[ ! -d "${ws_root}/src" ]]; then
    echo "ERROR: workspace src directory not found: ${ws_root}/src" >&2
    _migels_env_finish 1
    return $?
  fi

  if [[ ! -f "${ros_setup}" ]]; then
    echo "ERROR: ROS setup not found: ${ros_setup}" >&2
    _migels_env_finish 1
    return $?
  fi
  _migels_env_source_setup "${ros_setup}"

  zlab_robots_ws="${ZLAB_ROBOTS_WS:-${HOME}/zlab_robots}"
  if [[ -f "${zlab_robots_ws}/devel/setup.bash" ]]; then
    _migels_env_source_setup "${zlab_robots_ws}/devel/setup.bash"
  else
    echo "WARN: zlab_robots setup not found: ${zlab_robots_ws}/devel/setup.bash" >&2
  fi

  if [[ ! -f "${ws_root}/devel/setup.bash" ]]; then
    echo "ERROR: Mi-Gels workspace has not been built or sourced yet: ${ws_root}/devel/setup.bash" >&2
    echo "Run: cd ${ws_root} && catkin build" >&2
    _migels_env_finish 2
    return $?
  fi
  _migels_env_source_setup "${ws_root}/devel/setup.bash"

  export ROS_PACKAGE_PATH="${ws_root}/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
  export MIGELS_WS_ROOT="${ws_root}"
  export MIGELS_RUNTIME_CONFIG="${MIGELS_RUNTIME_CONFIG:-${ws_root}/migels_runtime.yaml}"

  if ! command -v rospack >/dev/null 2>&1; then
    echo "ERROR: rospack not found after sourcing ROS." >&2
    _migels_env_finish 1
    return $?
  fi

  rospack profile >/dev/null

  local failed=0
  local package_xml
  local package_name
  local expected_dir
  local resolved_dir

  while IFS= read -r package_xml; do
    package_name="$(_migels_env_package_name "${package_xml}")"
    expected_dir="$(_migels_env_abs_dir "$(dirname "${package_xml}")")"
    if [[ -z "${package_name}" ]]; then
      echo "ERROR: could not parse package name from ${package_xml}" >&2
      failed=1
      continue
    fi
    if ! resolved_dir="$(rospack find "${package_name}" 2>/dev/null)"; then
      echo "ERROR: rospack cannot find package ${package_name}" >&2
      failed=1
      continue
    fi
    resolved_dir="$(_migels_env_abs_dir "${resolved_dir}")"
    if [[ "${resolved_dir}" != "${expected_dir}" ]]; then
      echo "ERROR: package ${package_name} resolves to the wrong path:" >&2
      echo "  resolved: ${resolved_dir}" >&2
      echo "  expected: ${expected_dir}" >&2
      failed=1
    else
      echo "Using ${package_name}: ${resolved_dir}"
    fi
  done < <(find "${ws_root}/src" -mindepth 2 -maxdepth 2 -name package.xml -print | sort)

  if [[ "${failed}" -ne 0 ]]; then
    echo "ROS_PACKAGE_PATH:" >&2
    echo "${ROS_PACKAGE_PATH}" | tr ':' '\n' >&2
    _migels_env_finish 3
    return $?
  fi

  echo "Mi-Gels ROS environment ready: ${ws_root}"
  _migels_env_finish 0
}

_migels_env_main "$@"
