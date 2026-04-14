#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-${REPO_ROOT}/tmp/dds_smoke_logs}"
SIM_LOG="${LOG_DIR}/sim.log"
LOWSTATE_LOG="${LOG_DIR}/lowstate_listener.log"
LOWCMD_LOG="${LOG_DIR}/lowcmd_sender.log"
SUMMARY_LOG="${LOG_DIR}/summary.log"

ISAACSIM_LAUNCHER="${ISAACSIM_PYTHON_EXE:-isaac_sim_python}"
DDS_DOMAIN_ID="${DDS_DOMAIN_ID:-1}"
SIM_STARTUP_TIMEOUT_SECONDS="${SIM_STARTUP_TIMEOUT_SECONDS:-90}"
LOWSTATE_DURATION_SECONDS="${LOWSTATE_DURATION_SECONDS:-6}"
LOWCMD_DURATION_SECONDS="${LOWCMD_DURATION_SECONDS:-2}"
LOWCMD_JOINT_NAME="${LOWCMD_JOINT_NAME:-left_shoulder_pitch_joint}"
LOWCMD_OFFSET_RAD="${LOWCMD_OFFSET_RAD:-0.10}"
HEADLESS_FLAG="${HEADLESS_FLAG:---headless}"
LISTENER_START_DELAY_SECONDS="${LISTENER_START_DELAY_SECONDS:-1}"
SIM_LOG_TAIL_LINES="${SIM_LOG_TAIL_LINES:-120}"

log() {
  echo "[smoke] $*" | tee -a "${SUMMARY_LOG}"
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
  if grep -q "${pattern}" "${file_path}"; then
    log "check passed: ${label}"
    return 0
  fi
  log "check failed: ${label}"
  return 1
}

usage() {
  cat <<EOF
Usage:
  ./scripts/run_dds_smoke_test.sh

Optional environment variables:
  ISAACSIM_PYTHON_EXE=/path/to/isaac-sim/python.sh
  DDS_DOMAIN_ID=1
  SIM_STARTUP_TIMEOUT_SECONDS=90
  LOWSTATE_DURATION_SECONDS=6
  LOWCMD_DURATION_SECONDS=2
  LOWCMD_JOINT_NAME=left_shoulder_pitch_joint
  LOWCMD_OFFSET_RAD=0.10
  HEADLESS_FLAG=--headless
  LISTENER_START_DELAY_SECONDS=1
  SIM_LOG_TAIL_LINES=120

Logs are written under:
  ${LOG_DIR}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v "${ISAACSIM_LAUNCHER}" >/dev/null 2>&1; then
  echo "Isaac Sim launcher not found: ${ISAACSIM_LAUNCHER}" >&2
  echo "Make sure \`isaac_sim_python\` is available in your shell," >&2
  echo "or export ISAACSIM_PYTHON_EXE to an absolute path." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
: > "${SIM_LOG}"
: > "${LOWSTATE_LOG}"
: > "${LOWCMD_LOG}"
: > "${SUMMARY_LOG}"

SIM_PID=""

cleanup() {
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    log "stopping simulator pid=${SIM_PID}"
    kill "${SIM_PID}" || true
    wait "${SIM_PID}" || true
  fi
}
trap cleanup EXIT

log "repo_root=${REPO_ROOT}"
log "logs=${LOG_DIR}"
log "isaac_sim_launcher=${ISAACSIM_LAUNCHER}"
log "starting simulator"

(
  cd "${REPO_ROOT}"
  "${ISAACSIM_LAUNCHER}" src/main.py \
    "${HEADLESS_FLAG}" \
    --enable-dds \
    --enable-lowcmd-subscriber \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    > "${SIM_LOG}" 2>&1
) &
SIM_PID=$!

log "simulator pid=${SIM_PID}"

startup_deadline=$((SECONDS + SIM_STARTUP_TIMEOUT_SECONDS))
startup_ready=0
startup_last_reported_second=-1
while (( SECONDS < startup_deadline )); do
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    log "simulator exited during startup"
    dump_sim_tail
    exit 1
  fi
  if grep -q "initialized localhost DDS sidecar bridge" "${SIM_LOG}" \
    && grep -q "lowstate UDP publisher ready" "${SIM_LOG}" \
    && grep -q "lowcmd UDP subscriber ready" "${SIM_LOG}"; then
    startup_ready=1
    break
  fi
  remaining_seconds=$((startup_deadline - SECONDS))
  if (( remaining_seconds != startup_last_reported_second )) && (( remaining_seconds % 10 == 0 )); then
    log "waiting for DDS startup markers; ${remaining_seconds}s remaining"
    startup_last_reported_second=${remaining_seconds}
  fi
  sleep 1
done

if [[ "${startup_ready}" -ne 1 ]]; then
  log "simulator did not report DDS readiness within ${SIM_STARTUP_TIMEOUT_SECONDS}s"
  dump_sim_tail
  exit 1
fi

log "simulator DDS startup confirmed"

log "starting lowstate listener"
(
  cd "${REPO_ROOT}"
  python3 scripts/lowstate_listener.py \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --duration "${LOWSTATE_DURATION_SECONDS}" \
    --joint-name "${LOWCMD_JOINT_NAME}" \
    > "${LOWSTATE_LOG}" 2>&1
) &
LISTENER_PID=$!

sleep "${LISTENER_START_DELAY_SECONDS}"

if ! kill -0 "${LISTENER_PID}" 2>/dev/null; then
  log "lowstate listener exited unexpectedly before lowcmd send"
  cat "${LOWSTATE_LOG}" | tee -a "${SUMMARY_LOG}"
  dump_sim_tail
  exit 1
fi

log "sending conservative lowcmd while listener is active"
(
  cd "${REPO_ROOT}"
  python3 scripts/send_lowcmd_offset.py \
    --dds-domain-id "${DDS_DOMAIN_ID}" \
    --joint-name "${LOWCMD_JOINT_NAME}" \
    --offset-rad "${LOWCMD_OFFSET_RAD}" \
    --duration "${LOWCMD_DURATION_SECONDS}" \
    > "${LOWCMD_LOG}" 2>&1
)

wait "${LISTENER_PID}"

log "smoke test complete"

dds_comm_ok=1
assert_log_contains "${SIM_LOG}" "lowstate UDP publisher ready" "sim reported lowstate bridge readiness" || dds_comm_ok=0
assert_log_contains "${SIM_LOG}" "lowcmd UDP subscriber ready" "sim reported lowcmd bridge readiness" || dds_comm_ok=0
assert_log_contains "${LOWSTATE_LOG}" "valid_messages=" "listener received lowstate samples" || dds_comm_ok=0
assert_log_contains "${LOWSTATE_LOG}" "transport_rejected=0" "listener observed zero transport rejections" || dds_comm_ok=0
assert_log_contains "${LOWCMD_LOG}" "Published " "lowcmd sender published DDS commands" || dds_comm_ok=0

if [[ "${dds_comm_ok}" -eq 1 ]]; then
  log "RESULT: DDS communication working end-to-end."
else
  log "RESULT: DDS smoke test completed, but one or more DDS verification checks failed."
  dump_sim_tail
  exit 1
fi

log "collect these logs:"
log "  ${SIM_LOG}"
log "  ${LOWSTATE_LOG}"
log "  ${LOWCMD_LOG}"
log "  ${SUMMARY_LOG}"
