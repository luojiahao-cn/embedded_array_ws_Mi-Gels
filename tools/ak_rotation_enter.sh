#!/usr/bin/env bash
set -euo pipefail
trap 'echo "ERROR: command failed at ${BASH_SOURCE[0]}:${LINENO}: ${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
TRIGGER_TOPIC="${TRIGGER_TOPIC:-/maggrad_continuous_collection/record_trigger}"
METADATA_TOPIC="${METADATA_TOPIC:-/maggrad_continuous_collection/experiment_metadata}"
DEFAULT_EXPERIMENT_ID="${EXPERIMENT_ID:-ak_rotation}"
DEFAULT_PROFILE="${PROFILE:-AUTO}"
DEFAULT_MAG_RATE_HZ="${MAG_RATE_HZ:-400}"
DEFAULT_ICM_RATE_HZ="${ICM_RATE_HZ:-500}"
DEFAULT_CONTROL_MODE="${CONTROL_MODE:-manual}"
AK_COLLECTION_START="${AK_COLLECTION_START:-yes}"
AK_COLLECTION_PORT="${AK_COLLECTION_PORT:-/dev/serial/by-id/usb-MI-GELS_MagGrad_AK_TMAG_V1_315E39623233-if00}"
AK_COLLECTION_OUTPUT_DIR="${AK_COLLECTION_OUTPUT_DIR:-${WS_ROOT}/data/ak_rotation}"
AK_COLLECTION_LOG_DIR="${AK_COLLECTION_LOG_DIR:-${WS_ROOT}/logs/ak_rotation}"
AK_MIN_VALID_ROWS="${AK_MIN_VALID_ROWS:-100}"
AUTO_ROTATE_SCRIPT="${AUTO_ROTATE_SCRIPT:-${WS_ROOT}/tools/diana7_joint7_rotate.py}"
AUTO_NEG_ENDPOINT_DEG="${AUTO_NEG_ENDPOINT_DEG:--174}"
AUTO_POS_ENDPOINT_DEG="${AUTO_POS_ENDPOINT_DEG:-174}"
AUTO_ENDPOINT_TOL_DEG="${AUTO_ENDPOINT_TOL_DEG:-6}"
AUTO_SPEED_PERCENT="${AUTO_SPEED_PERCENT:-2}"
AUTO_ACC_PERCENT="${AUTO_ACC_PERCENT:-2}"
AUTO_START_BRINGUP="${AUTO_START_BRINGUP:-ask}"
SWEEP_START_V="${SWEEP_START_V:-5.0}"
SWEEP_END_V="${SWEEP_END_V:--5.0}"
SWEEP_STEP_V="${SWEEP_STEP_V:--0.5}"
FY8300_START="${FY8300_START:-ask}"
FY8300_CONFIG="${FY8300_CONFIG:-${WS_ROOT}/src/sensor_data_collection/config/signal_params_manual.yaml}"
FY8300_LOG_DIR="${FY8300_LOG_DIR:-${WS_ROOT}/logs/ak_rotation}"
FY8300_SETTLE_S="${FY8300_SETTLE_S:-1.0}"
AK_DATA_TOPIC="${AK_DATA_TOPIC:-/stm_uplink_raw}"
AK_DATA_WAIT_S="${AK_DATA_WAIT_S:-30}"
DIANA7_TOOL="${DIANA7_TOOL:-magnetometer_array}"
DIANA7_ROBOT_IP="${DIANA7_ROBOT_IP:-192.168.31.200}"
DIANA7_USE_RVIZ="${DIANA7_USE_RVIZ:-false}"
DIANA7_BRINGUP_LOG_DIR="${DIANA7_BRINGUP_LOG_DIR:-${WS_ROOT}/logs/ak_rotation}"

usage() {
  cat <<'EOF'
Usage: tools/ak_rotation_enter.sh

Interactive controller for AK09973D rotation captures.

Workflow:
  1. Start ak_sensor_only_collection.launch in another terminal.
  2. Run this script.
  3. Measure +5V current for coil 1, coil 2, and coil 3.
  4. The script sweeps FY8300 DC voltage from +5V to -5V.
  5. Each voltage point records coil1 -> coil2 -> coil3, one joint-7 rotation per coil.

Environment:
  TRIGGER_TOPIC     Default: /maggrad_continuous_collection/record_trigger
  METADATA_TOPIC    Default: /maggrad_continuous_collection/experiment_metadata
  EXPERIMENT_ID     Default: ak_rotation
  PROFILE           Default: AUTO
  MAG_RATE_HZ       Default: 400
  ICM_RATE_HZ       Default: 500
  CONTROL_MODE      manual or auto. Default: manual
  AK_COLLECTION_START  yes, ask, or no. Default: yes
  AK_COLLECTION_PORT   AK firmware serial port.
  AK_COLLECTION_OUTPUT_DIR  Data output dir. Default: workspace data/ak_rotation
  AK_MIN_VALID_ROWS    Delete recordings with fewer data rows. Default: 100
  AUTO_NEG_ENDPOINT_DEG  Negative endpoint angle. Default: -174
  AUTO_POS_ENDPOINT_DEG  Positive endpoint angle. Default: 174
  AUTO_ENDPOINT_TOL_DEG  Skip pre-positioning when already this close to an endpoint. Default: 6
  AUTO_SPEED_PERCENT  MoveIt velocity percent in auto mode. Default: 2
  AUTO_ACC_PERCENT    MoveIt acceleration percent in auto mode. Default: 2
  AUTO_START_BRINGUP  ask, yes, or no. Default: ask
  SWEEP_START_V      FY8300 sweep start voltage. Default: 5.0
  SWEEP_END_V        FY8300 sweep end voltage. Default: -5.0
  SWEEP_STEP_V       FY8300 sweep step voltage. Default: -0.5
  FY8300_START       ask, yes, or no. Default: ask
  FY8300_CONFIG      FY8300 DC config yaml. Default: sensor_data_collection/config/signal_params_manual.yaml
  FY8300_SETTLE_S    Seconds to wait after each voltage change. Default: 1.0
  AK_DATA_TOPIC      AK data topic required before rotation. Default: /stm_uplink_raw
  AK_DATA_WAIT_S     Seconds to wait for one AK frame before sweep. Default: 30
  COIL1_CURRENT_AT_5V_A  Optional default measured current for coil 1.
  COIL2_CURRENT_AT_5V_A  Optional default measured current for coil 2.
  COIL3_CURRENT_AT_5V_A  Optional default measured current for coil 3.
  DIANA7_TOOL         Tool name for diana7_bringup.launch. Default: magnetometer_array
  DIANA7_ROBOT_IP     Diana7 robot IP. Default: 192.168.31.200
  DIANA7_USE_RVIZ     Start RViz with Diana7 bringup. Default: false
  ROS_DISTRO        Default: noetic
  ZLAB_ROBOTS_WS    Default: $HOME/zlab_robots
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
  set +u
  source "${ROS_SETUP}"
  set -u
fi

ZLAB_ROBOTS_WS="${ZLAB_ROBOTS_WS:-${HOME}/zlab_robots}"
if [[ -f "${ZLAB_ROBOTS_WS}/devel/setup.bash" ]]; then
  set +u
  source "${ZLAB_ROBOTS_WS}/devel/setup.bash"
  set -u
fi

if [[ -f "${WS_ROOT}/devel/setup.bash" ]]; then
  set +u
  source "${WS_ROOT}/devel/setup.bash"
  set -u
fi

if ! command -v rostopic >/dev/null 2>&1; then
  echo "ERROR: rostopic not found. Source ROS and this workspace first." >&2
  exit 1
fi

recording=0
diana7_bringup_started=0
diana7_bringup_pid=""
ak_collection_started=0
ak_collection_pid=""
fy8300_started=0
fy8300_pid=""
fy8300_active_channel=""
auto_move_pid=""

publish_trigger() {
  local value="$1"
  rostopic pub "${TRIGGER_TOPIC}" std_msgs/Bool "data: ${value}" -1 >/dev/null
}

publish_metadata() {
  local experiment_id="$1"
  local coil_channel="$2"
  local coil_current_a="$3"
  local rotation_run_id="$4"
  local fy8300_voltage_v="$5"
  local current_at_5v_a="$6"
  local estimated_current_a="$7"
  local payload
  payload="experiment_id=${experiment_id} sensor_type=AK09973D board=MagGrad_AK_TMAG_V1 profile=${DEFAULT_PROFILE} mag_rate_hz=${DEFAULT_MAG_RATE_HZ} icm_rate_hz=${DEFAULT_ICM_RATE_HZ} coil_channel=${coil_channel} coil_current_a=${coil_current_a} fy8300_voltage_v=${fy8300_voltage_v} current_at_5v_a=${current_at_5v_a} estimated_current_a=${estimated_current_a} current_mapping=linear_from_5v signal_generator=FY8300 fy8300_channel=${coil_channel} rotation_run_id=${rotation_run_id} warning=NONE"
  rostopic pub "${METADATA_TOPIC}" std_msgs/String "data: '${payload}'" -1 >/dev/null
}

move_group_running() {
  rosnode list 2>/dev/null | grep -qx "/move_group"
}

ak_collection_running() {
  rosnode list 2>/dev/null | grep -qx "/serial_node_maggrad" || return 1
  rosnode list 2>/dev/null | grep -qx "/maggrad_continuous_collection" || return 1
}

wait_for_ak_collection_nodes() {
  local timeout_s="$1"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if ak_collection_running; then
      return 0
    fi
    if (( $(date +%s) - start_s >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

fy8300_running() {
  pgrep -x "fy8300_node" >/dev/null 2>&1 || return 1
  rosnode list 2>/dev/null | grep -Eq "^/fy8300($|_)|^/fy8300_node$"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    if (( $(date +%s) - start_s >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

publish_float32() {
  local topic="$1"
  local value="$2"
  rostopic pub "${topic}" std_msgs/Float32 "data: ${value}" -1 >/dev/null
}

publish_float64() {
  local topic="$1"
  local value="$2"
  rostopic pub "${topic}" std_msgs/Float64 "data: ${value}" -1 >/dev/null
}

publish_uint8() {
  local topic="$1"
  local value="$2"
  rostopic pub "${topic}" std_msgs/UInt8 "data: ${value}" -1 >/dev/null
}

publish_bool() {
  local topic="$1"
  local value="$2"
  rostopic pub "${topic}" std_msgs/Bool "data: ${value}" -1 >/dev/null
}

start_fy8300_if_needed() {
  if fy8300_running; then
    echo "FY8300 node already running."
    return 0
  fi

  local start_choice="${FY8300_START}"
  if [[ "${start_choice}" == "ask" ]]; then
    read -r -p "FY8300 node is not running. Start it now? yes/no [yes]: " start_choice || exit 0
    start_choice="${start_choice:-yes}"
  fi

  case "${start_choice}" in
    yes|y|true|1) ;;
    no|n|false|0)
      echo "ERROR: FY8300 control needs signal_generator/fy8300_node." >&2
      return 1
      ;;
    *)
      echo "ERROR: FY8300_START must be ask, yes, or no; got: ${start_choice}" >&2
      return 1
      ;;
  esac

  if [[ ! -f "${FY8300_CONFIG}" ]]; then
    echo "ERROR: FY8300 config not found: ${FY8300_CONFIG}" >&2
    return 1
  fi

  mkdir -p "${FY8300_LOG_DIR}"
  local ts log_path
  ts="$(date +%Y%m%d_%H%M%S)"
  log_path="${FY8300_LOG_DIR}/fy8300_${ts}.log"
  echo "Starting FY8300 node in background..."
  echo "Log: ${log_path}"
  rosparam load "${FY8300_CONFIG}" /fy8300
  nohup bash -lc "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; source '${ZLAB_ROBOTS_WS}/devel/setup.bash'; source '${WS_ROOT}/devel/setup.bash'; rosrun signal_generator fy8300_node __name:=fy8300" > "${log_path}" 2>&1 &
  fy8300_pid="$!"
  fy8300_started=1
  echo "${fy8300_pid}" > "${FY8300_LOG_DIR}/fy8300.pid"

  echo "Waiting for FY8300 topics..."
  if ! wait_for_topic "/fy8300/ch1/offset" 20 || ! fy8300_running; then
    echo "ERROR: FY8300 topics did not appear within 20 seconds. Check ${log_path}" >&2
    tail -80 "${log_path}" >&2 || true
    return 1
  fi
  echo "FY8300 node is ready."
}

set_fy8300_dc_defaults() {
  local ch
  publish_bool "/fy8300/sync_output" false || true
  for ch in 1 2 3; do
    publish_uint8 "/fy8300/ch${ch}/waveform" 6
    publish_float64 "/fy8300/ch${ch}/frequency" 0.0
    publish_float32 "/fy8300/ch${ch}/amplitude" 0.0
    publish_float32 "/fy8300/ch${ch}/phase" 0.0
    publish_bool "/fy8300/ch${ch}/output_en" false
  done
}

set_fy8300_voltage() {
  local active_channel="$1"
  local voltage="$2"
  if [[ -n "${fy8300_active_channel}" && "${fy8300_active_channel}" != "${active_channel}" ]]; then
    publish_bool "/fy8300/ch${fy8300_active_channel}/output_en" false
  fi
  publish_float32 "/fy8300/ch${active_channel}/offset" "${voltage}"
  publish_bool "/fy8300/ch${active_channel}/output_en" true
  fy8300_active_channel="${active_channel}"
  sleep "${FY8300_SETTLE_S}"
}

shutdown_fy8300_outputs() {
  local ch
  for ch in 1 2 3; do
    publish_bool "/fy8300/ch${ch}/output_en" false || true
  done
  fy8300_active_channel=""
}

disable_active_fy8300_output() {
  if [[ -n "${fy8300_active_channel}" ]]; then
    publish_bool "/fy8300/ch${fy8300_active_channel}/output_en" false || true
    fy8300_active_channel=""
  fi
}

wait_for_ak_data() {
  local start_s
  echo "Checking AK data stream on ${AK_DATA_TOPIC}..."
  start_s="$(date +%s)"
  while true; do
    if timeout 3 rostopic echo -n 1 "${AK_DATA_TOPIC}" >/dev/null 2>&1; then
      echo "AK data stream is active."
      return 0
    fi
    if (( $(date +%s) - start_s >= AK_DATA_WAIT_S )); then
      break
    fi
    sleep 1
  done
  echo "ERROR: no AK data received from ${AK_DATA_TOPIC} within ${AK_DATA_WAIT_S}s." >&2
  echo "Check the serial firmware stream or the ak_sensor_only_collection.launch log." >&2
  return 1
}

start_ak_collection_if_needed() {
  if ak_collection_running && wait_for_ak_data; then
    echo "AK collection is already running and data stream is active."
    return 0
  fi

  local start_choice="${AK_COLLECTION_START}"
  if [[ "${start_choice}" == "ask" ]]; then
    read -r -p "AK collection is not active. Start ak_sensor_only_collection.launch now? yes/no [yes]: " start_choice || exit 0
    start_choice="${start_choice:-yes}"
  fi

  case "${start_choice}" in
    yes|y|true|1) ;;
    no|n|false|0)
      echo "ERROR: AK collection is required before rotation capture." >&2
      return 1
      ;;
    *)
      echo "ERROR: AK_COLLECTION_START must be yes, ask, or no; got: ${start_choice}" >&2
      return 1
      ;;
  esac

  if [[ ! -e "${AK_COLLECTION_PORT}" ]]; then
    echo "ERROR: AK serial port not found: ${AK_COLLECTION_PORT}" >&2
    return 1
  fi

  mkdir -p "${AK_COLLECTION_LOG_DIR}" "${AK_COLLECTION_OUTPUT_DIR}"
  local ts log_path
  ts="$(date +%Y%m%d_%H%M%S)"
  log_path="${AK_COLLECTION_LOG_DIR}/ak_sensor_only_${ts}.log"
  echo "Starting AK sensor collection in background..."
  echo "Log: ${log_path}"
  nohup bash -lc "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; source '${ZLAB_ROBOTS_WS}/devel/setup.bash'; source '${WS_ROOT}/devel/setup.bash'; roslaunch sensor_data_collection ak_sensor_only_collection.launch port:='${AK_COLLECTION_PORT}' output_dir:='${AK_COLLECTION_OUTPUT_DIR}' trigger_rate_hz:=${DEFAULT_MAG_RATE_HZ} icm_rate_hz:=${DEFAULT_ICM_RATE_HZ} startup_profile:='${DEFAULT_PROFILE}' experiment_id:='${DEFAULT_EXPERIMENT_ID}' coil_channel:=1 rotation_run_id:=script_start min_valid_rows:=${AK_MIN_VALID_ROWS} delete_invalid_recordings:=true" > "${log_path}" 2>&1 &
  ak_collection_pid="$!"
  ak_collection_started=1
  echo "${ak_collection_pid}" > "${AK_COLLECTION_LOG_DIR}/ak_sensor_only.pid"

  echo "Waiting for AK collection nodes..."
  if ! wait_for_ak_collection_nodes 30; then
    echo "ERROR: AK collection nodes did not start within 30 seconds. Check ${log_path}" >&2
    tail -120 "${log_path}" >&2 || true
    return 1
  fi

  echo "Waiting for AK collection data..."
  if ! wait_for_ak_data; then
    echo "ERROR: AK data stream did not become active. Check ${log_path}" >&2
    tail -120 "${log_path}" >&2 || true
    return 1
  fi
  echo "AK collection is ready."
}

generate_voltage_sequence() {
  python3 - "$SWEEP_START_V" "$SWEEP_END_V" "$SWEEP_STEP_V" <<'PY'
import sys
start = float(sys.argv[1])
end = float(sys.argv[2])
step = float(sys.argv[3])
if step == 0:
    raise SystemExit("SWEEP_STEP_V must not be 0")
if (end - start) * step < 0:
    raise SystemExit("SWEEP_STEP_V sign does not move start toward end")
values = []
v = start
eps = abs(step) * 1e-6 + 1e-9
if step > 0:
    while v <= end + eps:
        values.append(v)
        v += step
else:
    while v >= end - eps:
        values.append(v)
        v += step
for item in values:
    print(f"{item:.6g}")
PY
}

estimate_current() {
  local voltage="$1"
  local current_at_5v="$2"
  python3 - "$voltage" "$current_at_5v" <<'PY'
import sys
voltage = float(sys.argv[1])
current_at_5v = float(sys.argv[2])
print(f"{voltage / 5.0 * current_at_5v:.9g}")
PY
}

voltage_label() {
  local voltage="$1"
  python3 - "$voltage" <<'PY'
import sys
v = float(sys.argv[1])
prefix = "p" if v >= 0 else "m"
text = f"{abs(v):.3f}".rstrip("0").rstrip(".")
print(prefix + text.replace(".", "p"))
PY
}

validate_coil_channel() {
  local channel="$1"
  case "${channel}" in
    1|2|3) return 0 ;;
    *) echo "ERROR: coil channel must be 1, 2, or 3; got: ${channel}" >&2; return 1 ;;
  esac
}

wait_for_move_group() {
  local timeout_s="$1"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if move_group_running; then
      return 0
    fi
    if (( $(date +%s) - start_s >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

read_current_joint7_deg() {
  local raw
  raw="$("${AUTO_ROTATE_SCRIPT}" --print-current-deg --connect-wait-s 0.0 2>&1 || true)"
  printf "%s\n" "${raw}" | python3 -c 'import re, sys
text = sys.stdin.read()
match = re.search(r"CURRENT_JOINT7_DEG=([-+]?(?:\d+(?:\.\d*)?|\.\d+))", text)
if match:
    print(match.group(1))
    raise SystemExit(0)
print(text, file=sys.stderr)
raise SystemExit("Could not parse current joint 7 angle from diana7_joint7_rotate output")'
}

start_diana7_bringup_if_needed() {
  if move_group_running; then
    echo "Diana7 move_group already running."
    return 0
  fi

  local start_choice="${AUTO_START_BRINGUP}"
  if [[ "${start_choice}" == "ask" ]]; then
    read -r -p "Diana7 move_group is not running. Start diana7_bringup now? yes/no [yes]: " start_choice || exit 0
    start_choice="${start_choice:-yes}"
  fi

  case "${start_choice}" in
    yes|y|true|1) ;;
    no|n|false|0)
      echo "ERROR: auto mode needs Diana7 bringup/move_group." >&2
      return 1
      ;;
    *)
      echo "ERROR: AUTO_START_BRINGUP must be ask, yes, or no; got: ${start_choice}" >&2
      return 1
      ;;
  esac

  mkdir -p "${DIANA7_BRINGUP_LOG_DIR}"
  local ts log_path
  ts="$(date +%Y%m%d_%H%M%S)"
  log_path="${DIANA7_BRINGUP_LOG_DIR}/diana7_bringup_${ts}.log"
  echo "Starting Diana7 bringup in background..."
  echo "Log: ${log_path}"
  local bringup_launch
  bringup_launch="$(rospack find zlab_robots_bringup)/launch/diana7/diana7_bringup.launch"
  nohup bash -lc "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; source '${ZLAB_ROBOTS_WS}/devel/setup.bash'; source '${WS_ROOT}/devel/setup.bash'; roslaunch '${bringup_launch}' robot_ip:=${DIANA7_ROBOT_IP} tool_name:=${DIANA7_TOOL} use_rviz:=${DIANA7_USE_RVIZ}" > "${log_path}" 2>&1 &
  diana7_bringup_pid="$!"
  diana7_bringup_started=1
  echo "${diana7_bringup_pid}" > "${DIANA7_BRINGUP_LOG_DIR}/diana7_bringup.pid"

  echo "Waiting for /move_group..."
  if ! wait_for_move_group 45; then
    echo "ERROR: /move_group did not start within 45 seconds. Check ${log_path}" >&2
    tail -80 "${log_path}" >&2 || true
    return 1
  fi
  echo "Diana7 move_group is ready."
}

cleanup() {
  if [[ -n "${auto_move_pid}" ]]; then
    if kill -0 "${auto_move_pid}" 2>/dev/null; then
      echo
      echo "Stopping active Diana7 joint move..."
      kill -INT "${auto_move_pid}" 2>/dev/null || true
      sleep 1
      if kill -0 "${auto_move_pid}" 2>/dev/null; then
        kill -TERM "${auto_move_pid}" 2>/dev/null || true
      fi
    fi
    auto_move_pid=""
  fi
  if [[ "${recording}" == "1" ]]; then
    echo
    echo "Stopping active recording..."
    publish_trigger false || true
    recording=0
  fi
  shutdown_fy8300_outputs
  if [[ "${fy8300_started}" == "1" && -n "${fy8300_pid}" ]]; then
    if kill -0 "${fy8300_pid}" 2>/dev/null; then
      echo "Stopping FY8300 node started by this script..."
      kill -INT "${fy8300_pid}" 2>/dev/null || true
      sleep 2
      if kill -0 "${fy8300_pid}" 2>/dev/null; then
        kill -TERM "${fy8300_pid}" 2>/dev/null || true
      fi
    fi
    fy8300_started=0
  fi
  if [[ "${ak_collection_started}" == "1" && -n "${ak_collection_pid}" ]]; then
    if kill -0 "${ak_collection_pid}" 2>/dev/null; then
      echo "Stopping AK collection started by this script..."
      kill -INT "${ak_collection_pid}" 2>/dev/null || true
      sleep 3
      if kill -0 "${ak_collection_pid}" 2>/dev/null; then
        kill -TERM "${ak_collection_pid}" 2>/dev/null || true
      fi
    fi
    pkill -INT -f "roslaunch .*ak_sensor_only_collection.launch" 2>/dev/null || true
    ak_collection_started=0
  fi
  if [[ "${diana7_bringup_started}" == "1" && -n "${diana7_bringup_pid}" ]]; then
    if kill -0 "${diana7_bringup_pid}" 2>/dev/null; then
      echo "Stopping Diana7 bringup started by this script..."
      kill -INT "${diana7_bringup_pid}" 2>/dev/null || true
      sleep 3
      if kill -0 "${diana7_bringup_pid}" 2>/dev/null; then
        kill -TERM "${diana7_bringup_pid}" 2>/dev/null || true
      fi
    fi
    pkill -INT -f "roslaunch .*diana7_bringup.launch" 2>/dev/null || true
    pkill -INT -f "diana7_hardware" 2>/dev/null || true
    pkill -INT -f "move_group" 2>/dev/null || true
    diana7_bringup_started=0
  fi
}

handle_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

echo "AK09973D rotation capture control"
echo "Trigger topic: ${TRIGGER_TOPIC}"
echo "Metadata topic: ${METADATA_TOPIC}"
echo "Default experiment_id: ${DEFAULT_EXPERIMENT_ID}"
echo

read -r -p "Control mode manual/auto [${DEFAULT_CONTROL_MODE}]: " control_mode || exit 0
control_mode="${control_mode:-${DEFAULT_CONTROL_MODE}}"
case "${control_mode}" in
  manual|auto) ;;
  *)
    echo "ERROR: control mode must be manual or auto, got: ${control_mode}" >&2
    exit 1
    ;;
esac

start_ak_collection_if_needed

if [[ "${control_mode}" == "auto" ]]; then
  if [[ ! -x "${AUTO_ROTATE_SCRIPT}" ]]; then
    echo "ERROR: auto rotate script is not executable: ${AUTO_ROTATE_SCRIPT}" >&2
    exit 1
  fi
  start_diana7_bringup_if_needed
  read -r -p "Auto speed percent [${AUTO_SPEED_PERCENT}]: " auto_speed_percent || exit 0
  AUTO_SPEED_PERCENT="${auto_speed_percent:-${AUTO_SPEED_PERCENT}}"
  read -r -p "Auto acceleration percent [${AUTO_ACC_PERCENT}]: " auto_acc_percent || exit 0
  AUTO_ACC_PERCENT="${auto_acc_percent:-${AUTO_ACC_PERCENT}}"
  AUTO_SPEED_SCALING="$(python3 -c 'import sys; print(max(0.001, min(1.0, float(sys.argv[1]) / 100.0)))' "${AUTO_SPEED_PERCENT}")"
  AUTO_ACC_SCALING="$(python3 -c 'import sys; print(max(0.001, min(1.0, float(sys.argv[1]) / 100.0)))' "${AUTO_ACC_PERCENT}")"
  echo "Auto mode: Diana7 joint 7 will record nearest-endpoint sweeps ${AUTO_NEG_ENDPOINT_DEG} deg <-> ${AUTO_POS_ENDPOINT_DEG} deg."
  echo "Auto speed: ${AUTO_SPEED_PERCENT}% velocity, ${AUTO_ACC_PERCENT}% acceleration."
else
  echo "Manual mode: press Enter to start/stop while you rotate the arm."
fi
echo

start_fy8300_if_needed
set_fy8300_dc_defaults
echo "FY8300 DC sweep: ${SWEEP_START_V} V to ${SWEEP_END_V} V, step ${SWEEP_STEP_V} V."
echo "Only the selected coil channel will be enabled during each recording."
echo

wait_for_ak_data

run_index=1
next_endpoint_from=""
next_endpoint_to=""
mapfile -t sweep_voltages < <(generate_voltage_sequence)
declare -A current_at_5v_by_coil

echo "Measure +5V current for all three coils first."
for coil_channel in 1 2 3; do
  default_current_var="COIL${coil_channel}_CURRENT_AT_5V_A"
  default_current="${!default_current_var:-}"
  echo "Setting FY8300 ch${coil_channel} to +5.0 V for current measurement..."
  set_fy8300_voltage "${coil_channel}" "5.0"
  echo "FY8300 ch${coil_channel} is outputting +5.0 V. Measure the coil current now."
  read -r -p "Measured current at +5V for coil ${coil_channel} A [${default_current}]: " current_at_5v || exit 0
  current_at_5v="${current_at_5v:-${default_current}}"
  disable_active_fy8300_output
  echo "FY8300 outputs disabled after +5V current measurement."
  if [[ -z "${current_at_5v}" ]]; then
    echo "ERROR: current_at_5v_a is required for current estimation." >&2
    exit 1
  fi
  python3 - "$current_at_5v" <<'PY'
import sys
float(sys.argv[1])
PY
  current_at_5v_by_coil["${coil_channel}"]="${current_at_5v}"
done

echo "Starting interleaved 3-coil sweep: voltage point -> coil1 -> coil2 -> coil3."
for voltage in "${sweep_voltages[@]}"; do
  for coil_channel in 1 2 3; do
    current_at_5v="${current_at_5v_by_coil[${coil_channel}]}"
    estimated_current="$(estimate_current "${voltage}" "${current_at_5v}")"
    label="$(voltage_label "${voltage}")"
    rotation_run_id="coil${coil_channel}_${label}V_run$(printf "%03d" "${run_index}")"

    echo "[${run_index}] Setting FY8300 ch${coil_channel} DC offset to ${voltage} V; estimated current ${estimated_current} A."
    set_fy8300_voltage "${coil_channel}" "${voltage}"
    publish_metadata \
      "${DEFAULT_EXPERIMENT_ID}" \
      "${coil_channel}" \
      "${estimated_current}" \
      "${rotation_run_id}" \
      "${voltage}" \
      "${current_at_5v}" \
      "${estimated_current}"
    echo "[${run_index}] Metadata set: coil=${coil_channel}, voltage=${voltage}V, estimated_current=${estimated_current}A, run=${rotation_run_id}"

    if [[ "${control_mode}" == "auto" ]]; then
      current_joint7_deg="$(read_current_joint7_deg)"
      endpoint_choice="$(python3 -c 'import sys; cur=float(sys.argv[1]); neg=float(sys.argv[2]); pos=float(sys.argv[3]); print("neg" if abs(cur-neg) <= abs(cur-pos) else "pos")' "${current_joint7_deg}" "${AUTO_NEG_ENDPOINT_DEG}" "${AUTO_POS_ENDPOINT_DEG}")"
      if [[ "${endpoint_choice}" == "pos" ]]; then
        next_endpoint_from="${AUTO_POS_ENDPOINT_DEG}"
        next_endpoint_to="${AUTO_NEG_ENDPOINT_DEG}"
      else
        next_endpoint_from="${AUTO_NEG_ENDPOINT_DEG}"
        next_endpoint_to="${AUTO_POS_ENDPOINT_DEG}"
      fi
      echo "[${run_index}] Current joint 7: ${current_joint7_deg} deg; nearest endpoint is ${next_endpoint_from} deg."
      endpoint_delta_abs="$(python3 -c 'import sys; print(abs(float(sys.argv[1]) - float(sys.argv[2])))' "${current_joint7_deg}" "${next_endpoint_from}")"
      should_preposition="$(python3 -c 'import sys; print("yes" if float(sys.argv[1]) > float(sys.argv[2]) else "no")' "${endpoint_delta_abs}" "${AUTO_ENDPOINT_TOL_DEG}")"
      if [[ "${should_preposition}" == "yes" ]]; then
        echo "[${run_index}] Pre-positioning joint 7 to ${next_endpoint_from} deg without recording..."
        "${AUTO_ROTATE_SCRIPT}" \
          --move-to-deg "${next_endpoint_from}" \
          --fallback-lower-deg -179 \
          --fallback-upper-deg 179 \
          --limit-margin-deg 1 \
          --accept-final-error-deg 6 \
          --max-segment-deg 90 \
          --speed-scaling "${AUTO_SPEED_SCALING}" \
          --acc-scaling "${AUTO_ACC_SCALING}"
        echo "[${run_index}] Ready at ${next_endpoint_from} deg; recording sweep target is ${next_endpoint_to} deg."
      else
        echo "[${run_index}] Already within ${AUTO_ENDPOINT_TOL_DEG} deg of ${next_endpoint_from} deg; skipping pre-positioning."
        echo "[${run_index}] Recording sweep target is ${next_endpoint_to} deg."
      fi
      echo "[${run_index}] Starting recording and auto-rotating Diana7 joint 7."
    else
      read -r -p "[${run_index}] Press Enter to START, then rotate one full turn..." || exit 0
    fi

    publish_trigger true
    recording=1
    echo "[${run_index}] Recording."

    if [[ "${control_mode}" == "auto" ]]; then
      rotate_args=(
        --move-to-deg "${next_endpoint_to}"
        --fallback-lower-deg -179
        --fallback-upper-deg 179
        --limit-margin-deg 1
        --accept-final-error-deg 6
        --max-segment-deg 90
        --speed-scaling "${AUTO_SPEED_SCALING}"
        --acc-scaling "${AUTO_ACC_SCALING}"
      )
      "${AUTO_ROTATE_SCRIPT}" "${rotate_args[@]}" &
      auto_move_pid="$!"
      wait "${auto_move_pid}"
      auto_move_pid=""
    else
      read -r -p "[${run_index}] Press Enter to STOP after one full turn..." || exit 0
    fi
    publish_trigger false
    recording=0
    echo "[${run_index}] Stopped."
    if [[ "${control_mode}" == "auto" ]]; then
      sleep 2
    fi
    echo

    run_index=$((run_index + 1))
  done
done
shutdown_fy8300_outputs
echo "Completed interleaved 3-coil sweep; FY8300 outputs disabled."
