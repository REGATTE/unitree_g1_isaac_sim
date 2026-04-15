#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-${REPO_ROOT}/tmp/sdk2py_policy_smoke_logs}"
SIM_LOG="${LOG_DIR}/sim.log"
ROS2_TOPIC_LOG="${LOG_DIR}/ros2_topic_list.log"
ROS2_LOWSTATE_LOG="${LOG_DIR}/ros2_lowstate_listener.log"
SDK2PY_LOWSTATE_LOG="${LOG_DIR}/sdk2py_lowstate_listener.log"
SDK2PY_LOWCMD_LOG="${LOG_DIR}/sdk2py_lowcmd_offset.log"
HYGIENE_LOG="${LOG_DIR}/startup_hygiene.log"
SUMMARY_LOG="${LOG_DIR}/summary.log"

ISAACSIM_LAUNCHER="${ISAACSIM_PYTHON_EXE:-isaac_sim_python}"
DDS_DOMAIN_ID="${DDS_DOMAIN_ID:-1}"
SDK2PY_DOMAIN_ID="${SDK2PY_DOMAIN_ID:-${DDS_DOMAIN_ID}}"
SDK2PY_NETWORK_INTERFACE="${SDK2PY_NETWORK_INTERFACE:-lo}"
SIM_STARTUP_TIMEOUT_SECONDS="${SIM_STARTUP_TIMEOUT_SECONDS:-120}"
ROS2_LOWSTATE_DURATION_SECONDS="${ROS2_LOWSTATE_DURATION_SECONDS:-6}"
SDK2PY_LOWSTATE_DURATION_SECONDS="${SDK2PY_LOWSTATE_DURATION_SECONDS:-8}"
SDK2PY_LOWCMD_DURATION_SECONDS="${SDK2PY_LOWCMD_DURATION_SECONDS:-1}"
SDK2PY_LOWCMD_RATE_HZ="${SDK2PY_LOWCMD_RATE_HZ:-50}"
SDK2PY_LOWCMD_JOINT_NAME="${SDK2PY_LOWCMD_JOINT_NAME:-left_shoulder_pitch_joint}"
SDK2PY_LOWCMD_OFFSET_RAD="${SDK2PY_LOWCMD_OFFSET_RAD:-0.05}"
LISTENER_START_DELAY_SECONDS="${LISTENER_START_DELAY_SECONDS:-1}"
HEADLESS_FLAG="${HEADLESS_FLAG:---headless}"
SIM_LOG_TAIL_LINES="${SIM_LOG_TAIL_LINES:-160}"
ROS2_TOPIC_TIMEOUT_SECONDS="${ROS2_TOPIC_TIMEOUT_SECONDS:-10}"
HYGIENE_WAIT_SECONDS="${HYGIENE_WAIT_SECONDS:-2}"

SIM_PID=""
SIM_SESSION_ID=""

log() {
  echo "[sdk2py-smoke] $*" | tee -a "${SUMMARY_LOG}"
}

dump_sim_tail() {
  if [[ -f "${SIM_LOG}" ]]; then
    log "last ${SIM_LOG_TAIL_LINES} lines from sim.log:"
    tail -n "${SIM_LOG_TAIL_LINES}" "${SIM_LOG}" | tee -a "${SUMMARY_LOG}"
  fi
}

fail() {
  log "FAIL: $*"
  dump_sim_tail
  exit 1
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

assert_log_not_contains() {
  local file_path="$1"
  local pattern="$2"
  local label="$3"
  if [[ ! -f "${file_path}" ]]; then
    log "check failed: ${label} (missing ${file_path})"
    return 1
  fi
  if grep -F -q "${pattern}" "${file_path}"; then
    log "check failed: ${label}"
    return 1
  fi
  log "check passed: ${label}"
  return 0
}

repo_owned_sidecar_pids() {
  ps -eo pid=,args= | while read -r pid args; do
    [[ -n "${pid}" ]] || continue
    [[ "${pid}" != "$$" ]] || continue
    case "${args}" in
      *"${REPO_ROOT}"*"ros2_cyclonedds_sidecar.py"*|\
      *"${REPO_ROOT}"*"unitree_sdk2py_sidecar.py"*|\
      *"${REPO_ROOT}"*"unitree_g1_native_bridge"*|\
      *"${REPO_ROOT}"*"unitree_g1_native_lowstate_listener"*|\
      *"${REPO_ROOT}"*"unitree_g1_native_lowcmd_offset"*)
        printf '%s\n' "${pid}"
        ;;
    esac
  done
}

run_startup_hygiene() {
  log "running startup hygiene for repo-owned stale sidecars"
  : > "${HYGIENE_LOG}"

  mapfile -t stale_pids < <(repo_owned_sidecar_pids)
  if [[ "${#stale_pids[@]}" -eq 0 ]]; then
    echo "no repo-owned stale sidecars found" | tee -a "${HYGIENE_LOG}" >/dev/null
  else
    echo "terminating repo-owned stale sidecar pids: ${stale_pids[*]}" | tee -a "${HYGIENE_LOG}" >/dev/null
    kill -TERM "${stale_pids[@]}" 2>/dev/null || true
    sleep "${HYGIENE_WAIT_SECONDS}"
  fi

  mapfile -t remaining_pids < <(repo_owned_sidecar_pids)
  if [[ "${#remaining_pids[@]}" -ne 0 ]]; then
    echo "remaining repo-owned sidecar pids after TERM: ${remaining_pids[*]}" | tee -a "${HYGIENE_LOG}" >/dev/null
    return 1
  fi
  echo "startup hygiene complete" | tee -a "${HYGIENE_LOG}" >/dev/null
  return 0
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
  ./scripts/run_sdk2py_policy_smoke_test.sh

This launches Isaac Sim in policy mode:
  - ROS 2 lowstate enabled
  - ROS 2 lowcmd disabled
  - SDK2 Python lowstate enabled
  - SDK2 Python lowcmd enabled
  - native Unitree SDK runtime disabled

Optional environment variables:
  ISAACSIM_PYTHON_EXE=isaac_sim_python
  DDS_DOMAIN_ID=1
  SDK2PY_DOMAIN_ID=1
  SDK2PY_NETWORK_INTERFACE=lo
  SDK2PY_LOWCMD_JOINT_NAME=left_shoulder_pitch_joint
  SDK2PY_LOWCMD_OFFSET_RAD=0.05
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
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found. Source ROS 2 and the unitree_ros2 workspace before running this smoke test." >&2
  exit 1
fi
if ! command -v timeout >/dev/null 2>&1; then
  echo "timeout command not found; install coreutils or run on a standard Linux shell." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
: > "${SIM_LOG}"
: > "${ROS2_TOPIC_LOG}"
: > "${ROS2_LOWSTATE_LOG}"
: > "${SDK2PY_LOWSTATE_LOG}"
: > "${SDK2PY_LOWCMD_LOG}"
: > "${SUMMARY_LOG}"

log "repo_root=${REPO_ROOT}"
log "logs=${LOG_DIR}"
log "dds_domain_id=${DDS_DOMAIN_ID}"
log "sdk2py_domain_id=${SDK2PY_DOMAIN_ID}"
log "sdk2py_network_interface=${SDK2PY_NETWORK_INTERFACE}"

run_startup_hygiene || fail "startup hygiene found repo-owned sidecars that did not exit"

log "resetting ROS 2 daemon for clean graph discovery"
export ROS_DOMAIN_ID="${DDS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ros2 daemon stop >> "${HYGIENE_LOG}" 2>&1 || true
ros2 daemon start >> "${HYGIENE_LOG}" 2>&1 || true

log "starting simulator in ROS2 observation + SDK2 Python policy mode"
(
  cd "${REPO_ROOT}"
  exec setsid "${ISAACSIM_LAUNCHER}" src/main.py \
    "${HEADLESS_FLAG}" \
    --enable-dds \
    --enable-ros2-lowstate \
    --no-enable-ros2-lowcmd \
    --enable-unitree-sdk2py-lowstate \
    --enable-unitree-sdk2py-lowcmd \
    --no-enable-native-unitree-lowstate \
    --no-enable-native-unitree-lowcmd \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --unitree-sdk2py-domain-id "${SDK2PY_DOMAIN_ID}" \
    --unitree-sdk2py-network-interface "${SDK2PY_NETWORK_INTERFACE}" \
    > "${SIM_LOG}" 2>&1
) &
SIM_PID=$!
SIM_SESSION_ID="${SIM_PID}"
log "simulator pid=${SIM_PID} session=${SIM_SESSION_ID}"

startup_deadline=$((SECONDS + SIM_STARTUP_TIMEOUT_SECONDS))
startup_ready=0
while (( SECONDS < startup_deadline )); do
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    fail "simulator exited during startup"
  fi
  if grep -q "DDS runtime mode: ros2=enabled" "${SIM_LOG}" \
    && grep -q "unitree_runtime=sdk2py" "${SIM_LOG}" \
    && grep -q "lowcmd_authority=sdk2py" "${SIM_LOG}" \
    && grep -q "initialized localhost DDS sidecar bridge" "${SIM_LOG}" \
    && grep -q "ROS 2 lowcmd subscriber disabled for this run" "${SIM_LOG}" \
    && grep -q "SDK2 Python lowcmd UDP subscriber ready" "${SIM_LOG}" \
    && grep -q "initialized Unitree SDK2 Python bridge" "${SIM_LOG}" \
    && grep -q "SDK2 Python sidecar initialized" "${SIM_LOG}"; then
    startup_ready=1
    break
  fi
  sleep 1
done

if [[ "${startup_ready}" -ne 1 ]]; then
  fail "simulator did not report SDK2 Python policy-mode readiness within ${SIM_STARTUP_TIMEOUT_SECONDS}s"
fi
log "policy-mode startup confirmed"

log "checking ROS 2 graph visibility"
timeout "${ROS2_TOPIC_TIMEOUT_SECONDS}" ros2 topic list --no-daemon > "${ROS2_TOPIC_LOG}" 2>&1 \
  || fail "ros2 topic list --no-daemon did not complete"

log "starting ROS2 lowstate listener"
(
  cd "${REPO_ROOT}"
  export ROS_DOMAIN_ID="${DDS_DOMAIN_ID}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  python3 scripts/lowstate_listener.py \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --duration "${ROS2_LOWSTATE_DURATION_SECONDS}" \
    --joint-name "${SDK2PY_LOWCMD_JOINT_NAME}" \
    > "${ROS2_LOWSTATE_LOG}" 2>&1
) &
ROS2_LISTENER_PID=$!

log "starting SDK2 Python lowstate listener"
(
  cd "${REPO_ROOT}"
  python3 scripts/sdk2py_lowstate_listener.py \
    --dds-domain-id "${SDK2PY_DOMAIN_ID}" \
    --network-interface "${SDK2PY_NETWORK_INTERFACE}" \
    --duration "${SDK2PY_LOWSTATE_DURATION_SECONDS}" \
    --joint-name "${SDK2PY_LOWCMD_JOINT_NAME}" \
    > "${SDK2PY_LOWSTATE_LOG}" 2>&1
) &
SDK2PY_LISTENER_PID=$!

sleep "${LISTENER_START_DELAY_SECONDS}"

if ! kill -0 "${ROS2_LISTENER_PID}" 2>/dev/null; then
  log "ROS2 lowstate listener exited before lowcmd send"
  cat "${ROS2_LOWSTATE_LOG}" | tee -a "${SUMMARY_LOG}"
  fail "ROS2 lowstate listener failed"
fi
if ! kill -0 "${SDK2PY_LISTENER_PID}" 2>/dev/null; then
  log "SDK2 Python lowstate listener exited before lowcmd send"
  cat "${SDK2PY_LOWSTATE_LOG}" | tee -a "${SUMMARY_LOG}"
  fail "SDK2 Python lowstate listener failed"
fi

log "sending conservative SDK2 Python lowcmd offset"
(
  cd "${REPO_ROOT}"
  python3 scripts/sdk2py_send_lowcmd_offset.py \
    --dds-domain-id "${SDK2PY_DOMAIN_ID}" \
    --network-interface "${SDK2PY_NETWORK_INTERFACE}" \
    --joint-name "${SDK2PY_LOWCMD_JOINT_NAME}" \
    --offset-rad "${SDK2PY_LOWCMD_OFFSET_RAD}" \
    --duration "${SDK2PY_LOWCMD_DURATION_SECONDS}" \
    --rate-hz "${SDK2PY_LOWCMD_RATE_HZ}" \
    > "${SDK2PY_LOWCMD_LOG}" 2>&1
)

wait "${ROS2_LISTENER_PID}"
wait "${SDK2PY_LISTENER_PID}"

log "validating logs"
checks_ok=1
assert_log_contains "${HYGIENE_LOG}" "startup hygiene complete" "startup hygiene completed" || checks_ok=0
assert_log_contains "${SIM_LOG}" "DDS runtime mode: ros2=enabled" "runtime mode logged ROS2 enabled" || checks_ok=0
assert_log_contains "${SIM_LOG}" "unitree_runtime=sdk2py" "runtime mode selected SDK2 Python" || checks_ok=0
assert_log_contains "${SIM_LOG}" "lowcmd_authority=sdk2py" "SDK2 Python is lowcmd authority" || checks_ok=0
assert_log_contains "${SIM_LOG}" "ROS 2 lowcmd subscriber disabled for this run" "ROS2 lowcmd disabled" || checks_ok=0
assert_log_contains "${SIM_LOG}" "initialized localhost DDS sidecar bridge" "ROS2 sidecar initialized" || checks_ok=0
assert_log_contains "${SIM_LOG}" "initialized Unitree SDK2 Python bridge" "SDK2 Python bridge initialized" || checks_ok=0
assert_log_contains "${SIM_LOG}" "cached SDK2 Python \`rt/lowcmd\` sample went stale" "SDK2 Python lowcmd reached simulator and became stale" || checks_ok=0
assert_log_not_contains "${SIM_LOG}" "native lowstate publisher initialized" "native lowstate did not start" || checks_ok=0
assert_log_not_contains "${SIM_LOG}" "native lowcmd subscriber initialized" "native lowcmd did not start" || checks_ok=0
assert_log_contains "${ROS2_TOPIC_LOG}" "/rt/lowstate" "ROS2 /rt/lowstate visible" || checks_ok=0
assert_log_not_contains "${ROS2_TOPIC_LOG}" "/rt/lowcmd" "ROS2 /rt/lowcmd hidden" || checks_ok=0
assert_log_contains "${ROS2_LOWSTATE_LOG}" "valid_messages=" "ROS2 lowstate listener received samples" || checks_ok=0
assert_log_contains "${ROS2_LOWSTATE_LOG}" "transport_rejected=0" "ROS2 lowstate listener saw zero transport rejections" || checks_ok=0
assert_log_contains "${SDK2PY_LOWSTATE_LOG}" "valid_messages=" "SDK2 Python lowstate listener received samples" || checks_ok=0
assert_log_contains "${SDK2PY_LOWSTATE_LOG}" "transport_rejected=0" "SDK2 Python lowstate listener saw zero transport rejections" || checks_ok=0
assert_log_contains "${SDK2PY_LOWSTATE_LOG}" "target_history ${SDK2PY_LOWCMD_JOINT_NAME}" "SDK2 Python lowstate captured target history" || checks_ok=0
assert_log_contains "${SDK2PY_LOWCMD_LOG}" "Published " "SDK2 Python lowcmd sender published commands" || checks_ok=0

if [[ "${checks_ok}" -ne 1 ]]; then
  log "RESULT: SDK2 Python policy smoke test failed"
  dump_sim_tail
  exit 1
fi

log "RESULT: SDK2 Python policy smoke test passed"
log "collect these logs:"
log "  ${SIM_LOG}"
log "  ${ROS2_TOPIC_LOG}"
log "  ${ROS2_LOWSTATE_LOG}"
log "  ${SDK2PY_LOWSTATE_LOG}"
log "  ${SDK2PY_LOWCMD_LOG}"
log "  ${HYGIENE_LOG}"
log "  ${SUMMARY_LOG}"
