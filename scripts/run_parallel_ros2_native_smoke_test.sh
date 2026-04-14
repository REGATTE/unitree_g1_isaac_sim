#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-${REPO_ROOT}/tmp/parallel_ros2_native_smoke_logs}"
SIM_LOG="${LOG_DIR}/sim.log"
ROS2_LOWSTATE_LOG="${LOG_DIR}/ros2_lowstate_listener.log"
NATIVE_LOWSTATE_LOG="${LOG_DIR}/native_lowstate_listener.log"
NATIVE_LOWCMD_LOG="${LOG_DIR}/native_lowcmd_offset.log"
SUMMARY_LOG="${LOG_DIR}/summary.log"

ISAACSIM_LAUNCHER="${ISAACSIM_PYTHON_EXE:-isaac_sim_python}"
DDS_DOMAIN_ID="${DDS_DOMAIN_ID:-1}"
NATIVE_DOMAIN_ID="${NATIVE_DOMAIN_ID:-${DDS_DOMAIN_ID}}"
SIM_STARTUP_TIMEOUT_SECONDS="${SIM_STARTUP_TIMEOUT_SECONDS:-120}"
ROS2_LOWSTATE_DURATION_SECONDS="${ROS2_LOWSTATE_DURATION_SECONDS:-6}"
NATIVE_LOWSTATE_DURATION_SECONDS="${NATIVE_LOWSTATE_DURATION_SECONDS:-5}"
NATIVE_LOWCMD_DURATION_SECONDS="${NATIVE_LOWCMD_DURATION_SECONDS:-2}"
NATIVE_LOWCMD_RATE_HZ="${NATIVE_LOWCMD_RATE_HZ:-50}"
NATIVE_LOWCMD_JOINT_INDEX="${NATIVE_LOWCMD_JOINT_INDEX:-15}"
NATIVE_LOWCMD_JOINT_NAME="${NATIVE_LOWCMD_JOINT_NAME:-left_shoulder_pitch_joint}"
NATIVE_LOWCMD_OFFSET_RAD="${NATIVE_LOWCMD_OFFSET_RAD:-0.05}"
LISTENER_START_DELAY_SECONDS="${LISTENER_START_DELAY_SECONDS:-1}"
HEADLESS_FLAG="${HEADLESS_FLAG:---headless}"
SIM_LOG_TAIL_LINES="${SIM_LOG_TAIL_LINES:-160}"
NATIVE_BUILD_DIR="${NATIVE_BUILD_DIR:-${REPO_ROOT}/native_sdk_bridge/build}"
NATIVE_BRIDGE_EXE="${NATIVE_BRIDGE_EXE:-${NATIVE_BUILD_DIR}/unitree_g1_native_bridge}"
NATIVE_LOWSTATE_LISTENER="${NATIVE_LOWSTATE_LISTENER:-${NATIVE_BUILD_DIR}/unitree_g1_native_lowstate_listener}"
NATIVE_LOWCMD_OFFSET="${NATIVE_LOWCMD_OFFSET:-${NATIVE_BUILD_DIR}/unitree_g1_native_lowcmd_offset}"

SIM_PID=""
SIM_SESSION_ID=""

log() {
  echo "[parallel-smoke] $*" | tee -a "${SUMMARY_LOG}"
}

fail() {
  log "FAIL: $*"
  dump_sim_tail
  exit 1
}

dump_sim_tail() {
  if [[ -f "${SIM_LOG}" ]]; then
    log "last ${SIM_LOG_TAIL_LINES} lines from sim.log:"
    tail -n "${SIM_LOG_TAIL_LINES}" "${SIM_LOG}" | tee -a "${SUMMARY_LOG}"
  fi
}

assert_log_contains() {
  local file_path="$1"
  local pattern="$2"
  local label="$3"
  if [[ ! -f "${file_path}" ]]; then
    log "check failed: ${label} (missing ${file_path})"
    return 1
  fi
  if grep -F -q "${pattern}" "${file_path}"; then
    log "check passed: ${label}"
    return 0
  fi
  log "check failed: ${label}"
  return 1
}

cleanup() {
  if [[ -n "${SIM_SESSION_ID}" ]]; then
    if pgrep -s "${SIM_SESSION_ID}" >/dev/null 2>&1; then
      log "stopping simulator session=${SIM_SESSION_ID}"
      pkill -TERM -s "${SIM_SESSION_ID}" 2>/dev/null || true
      sleep 2
      pkill -KILL -s "${SIM_SESSION_ID}" 2>/dev/null || true
    fi
  elif [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    log "stopping simulator pid=${SIM_PID}"
    kill "${SIM_PID}" || true
  fi
  if [[ -n "${SIM_PID}" ]]; then
    wait "${SIM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

usage() {
  cat <<EOF
Usage:
  ./scripts/run_parallel_ros2_native_smoke_test.sh

This launches Isaac Sim in mixed mode:
  - ROS 2 lowstate enabled
  - ROS 2 lowcmd disabled
  - native Unitree SDK lowstate enabled
  - native Unitree SDK lowcmd enabled

Optional environment variables:
  ISAACSIM_PYTHON_EXE=isaac_sim_python
  DDS_DOMAIN_ID=1
  NATIVE_DOMAIN_ID=1
  NATIVE_LOWCMD_JOINT_INDEX=15
  NATIVE_LOWCMD_JOINT_NAME=left_shoulder_pitch_joint
  NATIVE_LOWCMD_OFFSET_RAD=0.05
  LOG_DIR=${LOG_DIR}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v "${ISAACSIM_LAUNCHER}" >/dev/null 2>&1; then
  echo "Isaac Sim launcher not found: ${ISAACSIM_LAUNCHER}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
: > "${SIM_LOG}"
: > "${ROS2_LOWSTATE_LOG}"
: > "${NATIVE_LOWSTATE_LOG}"
: > "${NATIVE_LOWCMD_LOG}"
: > "${SUMMARY_LOG}"

log "repo_root=${REPO_ROOT}"
log "logs=${LOG_DIR}"
log "dds_domain_id=${DDS_DOMAIN_ID}"
log "native_domain_id=${NATIVE_DOMAIN_ID}"
log "building native SDK validation tools"

(
  cd "${REPO_ROOT}"
  cmake -S native_sdk_bridge -B "${NATIVE_BUILD_DIR}" >/dev/null
  cmake --build "${NATIVE_BUILD_DIR}" -j4 >/dev/null
)

[[ -x "${NATIVE_BRIDGE_EXE}" ]] || fail "native bridge executable missing: ${NATIVE_BRIDGE_EXE}"
[[ -x "${NATIVE_LOWSTATE_LISTENER}" ]] || fail "native lowstate listener missing: ${NATIVE_LOWSTATE_LISTENER}"
[[ -x "${NATIVE_LOWCMD_OFFSET}" ]] || fail "native lowcmd offset tool missing: ${NATIVE_LOWCMD_OFFSET}"

log "starting simulator in mixed ROS2/native mode"
(
  cd "${REPO_ROOT}"
  exec setsid "${ISAACSIM_LAUNCHER}" src/main.py \
    "${HEADLESS_FLAG}" \
    --enable-dds \
    --enable-ros2-lowstate \
    --no-enable-ros2-lowcmd \
    --enable-native-unitree-lowstate \
    --enable-native-unitree-lowcmd \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --native-unitree-domain-id "${NATIVE_DOMAIN_ID}" \
    --native-unitree-bridge-exe "${NATIVE_BRIDGE_EXE}" \
    > "${SIM_LOG}" 2>&1
) &
SIM_PID=$!
SIM_SESSION_ID="${SIM_PID}"
log "simulator pid=${SIM_PID} session=${SIM_SESSION_ID}"

startup_deadline=$((SECONDS + SIM_STARTUP_TIMEOUT_SECONDS))
startup_ready=0
while (( SECONDS < startup_deadline )); do
  if grep -q "initialized localhost DDS sidecar bridge" "${SIM_LOG}" \
    && grep -q "native lowstate publisher initialized" "${SIM_LOG}" \
    && grep -q "native lowcmd subscriber initialized" "${SIM_LOG}"; then
    startup_ready=1
    break
  fi
  sleep 1
done

if [[ "${startup_ready}" -ne 1 ]]; then
  fail "simulator did not report mixed-mode DDS readiness within ${SIM_STARTUP_TIMEOUT_SECONDS}s"
fi
log "mixed-mode startup confirmed"

log "starting ROS2 lowstate listener"
(
  cd "${REPO_ROOT}"
  export ROS_DOMAIN_ID="${DDS_DOMAIN_ID}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  python3 scripts/lowstate_listener.py \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --duration "${ROS2_LOWSTATE_DURATION_SECONDS}" \
    --joint-name "${NATIVE_LOWCMD_JOINT_NAME}" \
    > "${ROS2_LOWSTATE_LOG}" 2>&1
) &
ROS2_LISTENER_PID=$!

sleep "${LISTENER_START_DELAY_SECONDS}"

log "checking native lowstate visibility"
"${NATIVE_LOWSTATE_LISTENER}" \
  --domain-id "${NATIVE_DOMAIN_ID}" \
  --topic rt/lowstate \
  --duration "${NATIVE_LOWSTATE_DURATION_SECONDS}" \
  --joint-index "${NATIVE_LOWCMD_JOINT_INDEX}" \
  > "${NATIVE_LOWSTATE_LOG}" 2>&1

log "sending native lowcmd offset"
"${NATIVE_LOWCMD_OFFSET}" \
  --domain-id "${NATIVE_DOMAIN_ID}" \
  --lowstate-topic rt/lowstate \
  --lowcmd-topic rt/lowcmd \
  --joint-index "${NATIVE_LOWCMD_JOINT_INDEX}" \
  --offset-rad "${NATIVE_LOWCMD_OFFSET_RAD}" \
  --duration "${NATIVE_LOWCMD_DURATION_SECONDS}" \
  --rate-hz "${NATIVE_LOWCMD_RATE_HZ}" \
  > "${NATIVE_LOWCMD_LOG}" 2>&1

wait "${ROS2_LISTENER_PID}"

log "validating logs"
checks_ok=1
assert_log_contains "${SIM_LOG}" "ROS 2 lowcmd subscriber disabled for this run" "ROS2 lowcmd disabled in target mode" || checks_ok=0
assert_log_contains "${SIM_LOG}" "initialized localhost DDS sidecar bridge" "ROS2 sidecar initialized" || checks_ok=0
assert_log_contains "${SIM_LOG}" "native lowstate publisher initialized" "native lowstate publisher initialized" || checks_ok=0
assert_log_contains "${SIM_LOG}" "native lowcmd subscriber initialized" "native lowcmd subscriber initialized" || checks_ok=0
assert_log_contains "${ROS2_LOWSTATE_LOG}" "valid_messages=" "ROS2 lowstate listener received samples" || checks_ok=0
assert_log_contains "${ROS2_LOWSTATE_LOG}" "transport_rejected=0" "ROS2 lowstate listener saw zero transport rejections" || checks_ok=0
assert_log_contains "${ROS2_LOWSTATE_LOG}" "target_history ${NATIVE_LOWCMD_JOINT_NAME}" "ROS2 lowstate captured target joint history" || checks_ok=0
assert_log_contains "${NATIVE_LOWSTATE_LOG}" "native_lowstate messages=" "native SDK lowstate listener received samples" || checks_ok=0
assert_log_contains "${NATIVE_LOWSTATE_LOG}" "crc_failures=0" "native SDK lowstate CRCs passed" || checks_ok=0
assert_log_contains "${NATIVE_LOWCMD_LOG}" "native_lowcmd published=" "native SDK lowcmd sender published commands" || checks_ok=0

if [[ "${checks_ok}" -ne 1 ]]; then
  log "RESULT: mixed-mode smoke test failed"
  dump_sim_tail
  exit 1
fi

log "RESULT: mixed ROS2/native DDS smoke test passed"
log "collect these logs:"
log "  ${SIM_LOG}"
log "  ${ROS2_LOWSTATE_LOG}"
log "  ${NATIVE_LOWSTATE_LOG}"
log "  ${NATIVE_LOWCMD_LOG}"
log "  ${SUMMARY_LOG}"
