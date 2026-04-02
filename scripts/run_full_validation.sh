#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-${REPO_ROOT}/tmp/full_validation_logs}"
SUMMARY_LOG="${LOG_DIR}/summary.log"
UNIT_TEST_LOG="${LOG_DIR}/unit_tests.log"
STARTUP_RUN1_LOG="${LOG_DIR}/startup_run1.log"
STARTUP_RUN2_LOG="${LOG_DIR}/startup_run2.log"
STARTUP_SNAPSHOT1_LOG="${LOG_DIR}/startup_snapshot_run1.log"
STARTUP_SNAPSHOT2_LOG="${LOG_DIR}/startup_snapshot_run2.log"
STARTUP_DIFF_LOG="${LOG_DIR}/startup_snapshot.diff"
TRACKING_SIM_LOG="${LOG_DIR}/tracking_sim.log"
TRACKING_LISTENER_LOG="${LOG_DIR}/tracking_listener.log"
TRACKING_LOWCMD_LOG="${LOG_DIR}/tracking_lowcmd.log"
TRACKING_CSV_PATH="${LOG_DIR}/tracking_waist_yaw.csv"
LONG_RUN_LOG="${LOG_DIR}/long_run_sim.log"
SMOKE_LOG_DIR="${LOG_DIR}/smoke"

ISAACSIM_LAUNCHER="${ISAACSIM_PYTHON_EXE:-isaac_sim_python}"
DDS_DOMAIN_ID="${DDS_DOMAIN_ID:-1}"
SIM_STARTUP_TIMEOUT_SECONDS="${SIM_STARTUP_TIMEOUT_SECONDS:-90}"
HEADLESS_FLAG="${HEADLESS_FLAG:---headless}"
STARTUP_MAX_FRAMES="${STARTUP_MAX_FRAMES:-300}"
TRACKING_LISTENER_DURATION_SECONDS="${TRACKING_LISTENER_DURATION_SECONDS:-5}"
TRACKING_LOWCMD_DURATION_SECONDS="${TRACKING_LOWCMD_DURATION_SECONDS:-1.5}"
TRACKING_LOWCMD_JOINT_NAME="${TRACKING_LOWCMD_JOINT_NAME:-waist_yaw_joint}"
TRACKING_LOWCMD_OFFSET_RAD="${TRACKING_LOWCMD_OFFSET_RAD:-0.05}"
TRACKING_LISTENER_START_DELAY_SECONDS="${TRACKING_LISTENER_START_DELAY_SECONDS:-1}"
LONG_RUN_MAX_FRAMES="${LONG_RUN_MAX_FRAMES:-2400}"
LONG_RUN_CADENCE_INTERVAL="${LONG_RUN_CADENCE_INTERVAL:-500}"
SIM_LOG_TAIL_LINES="${SIM_LOG_TAIL_LINES:-120}"

SIM_PID=""

log() {
  echo "[full] $*" | tee -a "${SUMMARY_LOG}"
}

fail() {
  log "FAIL: $*"
  exit 1
}

cleanup() {
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    log "stopping simulator pid=${SIM_PID}"
    kill "${SIM_PID}" || true
    wait "${SIM_PID}" || true
  fi
}
trap cleanup EXIT

dump_sim_tail() {
  local file_path="$1"
  if [[ -f "${file_path}" ]]; then
    log "last ${SIM_LOG_TAIL_LINES} lines from ${file_path}:"
    tail -n "${SIM_LOG_TAIL_LINES}" "${file_path}" | tee -a "${SUMMARY_LOG}"
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

ensure_launcher() {
  if ! command -v "${ISAACSIM_LAUNCHER}" >/dev/null 2>&1; then
    fail "Isaac Sim launcher not found: ${ISAACSIM_LAUNCHER}"
  fi
}

start_simulator() {
  local log_path="$1"
  shift
  : > "${log_path}"
  (
    cd "${REPO_ROOT}"
    "${ISAACSIM_LAUNCHER}" src/main.py "$@" > "${log_path}" 2>&1
  ) &
  SIM_PID=$!
  log "simulator pid=${SIM_PID}"
}

wait_for_dds_startup() {
  local sim_log="$1"
  local startup_deadline=$((SECONDS + SIM_STARTUP_TIMEOUT_SECONDS))
  local startup_last_reported_second=-1

  while (( SECONDS < startup_deadline )); do
    if ! kill -0 "${SIM_PID}" 2>/dev/null; then
      log "simulator exited during startup"
      dump_sim_tail "${sim_log}"
      return 1
    fi
    if grep -q "initialized Unitree DDS channel factory" "${sim_log}" \
      && grep -q "DDS lowstate publisher ready" "${sim_log}" \
      && grep -q "DDS lowcmd subscriber ready" "${sim_log}"; then
      log "simulator DDS startup confirmed"
      return 0
    fi
    local remaining_seconds=$((startup_deadline - SECONDS))
    if (( remaining_seconds != startup_last_reported_second )) && (( remaining_seconds % 10 == 0 )); then
      log "waiting for DDS startup markers; ${remaining_seconds}s remaining"
      startup_last_reported_second=${remaining_seconds}
    fi
    sleep 1
  done

  log "simulator did not report DDS readiness within ${SIM_STARTUP_TIMEOUT_SECONDS}s"
  dump_sim_tail "${sim_log}"
  return 1
}

stop_simulator() {
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    log "stopping simulator pid=${SIM_PID}"
    kill "${SIM_PID}" || true
    wait "${SIM_PID}" || true
  fi
  SIM_PID=""
}

run_unit_tests() {
  log "phase=unit_tests"
  (
    cd "${REPO_ROOT}"
    python3 -m unittest \
      tests/test_g1_lowcmd.py \
      tests/test_dds_manager.py \
      tests/test_robot_control.py \
      tests/test_robot_state_pause_safe.py \
      tests/test_lowstate_listener.py
  ) > "${UNIT_TEST_LOG}" 2>&1 || {
    cat "${UNIT_TEST_LOG}" | tee -a "${SUMMARY_LOG}"
    fail "unit-test phase failed"
  }
  log "unit tests passed"
}

extract_startup_snapshot() {
  local source_log="$1"
  local output_log="$2"
  rg "applied deterministic startup state|base_position_world|base_quaternion_wxyz|sim_joint\\[|dds_joint\\[" "${source_log}" \
    | sed -E 's/^[0-9-]+ [0-9:,]+ \[[A-Z]+\] \[[^]]+\] //' \
    > "${output_log}"
}

run_startup_determinism_phase() {
  log "phase=startup_determinism"

  (
    cd "${REPO_ROOT}"
    "${ISAACSIM_LAUNCHER}" src/main.py \
      "${HEADLESS_FLAG}" \
      --enable-dds \
      --enable-lowcmd-subscriber \
      --max-frames "${STARTUP_MAX_FRAMES}"
  ) > "${STARTUP_RUN1_LOG}" 2>&1 || fail "startup determinism run 1 failed"

  (
    cd "${REPO_ROOT}"
    "${ISAACSIM_LAUNCHER}" src/main.py \
      "${HEADLESS_FLAG}" \
      --enable-dds \
      --enable-lowcmd-subscriber \
      --max-frames "${STARTUP_MAX_FRAMES}"
  ) > "${STARTUP_RUN2_LOG}" 2>&1 || fail "startup determinism run 2 failed"

  extract_startup_snapshot "${STARTUP_RUN1_LOG}" "${STARTUP_SNAPSHOT1_LOG}"
  extract_startup_snapshot "${STARTUP_RUN2_LOG}" "${STARTUP_SNAPSHOT2_LOG}"

  if ! diff -u "${STARTUP_SNAPSHOT1_LOG}" "${STARTUP_SNAPSHOT2_LOG}" > "${STARTUP_DIFF_LOG}"; then
    cat "${STARTUP_DIFF_LOG}" | tee -a "${SUMMARY_LOG}"
    fail "deterministic startup snapshot mismatch across two fresh launches"
  fi

  assert_log_contains "${STARTUP_RUN1_LOG}" "applied deterministic startup state" "run 1 applied deterministic startup state" \
    || fail "startup determinism run 1 did not apply deterministic state"
  assert_log_contains "${STARTUP_RUN2_LOG}" "applied deterministic startup state" "run 2 applied deterministic startup state" \
    || fail "startup determinism run 2 did not apply deterministic state"
  log "startup determinism snapshots matched across two runs"
}

run_smoke_phase() {
  log "phase=dds_smoke"
  rm -rf "${SMOKE_LOG_DIR}"
  mkdir -p "${SMOKE_LOG_DIR}"
  (
    cd "${REPO_ROOT}"
    LOG_DIR="${SMOKE_LOG_DIR}" \
    DDS_DOMAIN_ID="${DDS_DOMAIN_ID}" \
    HEADLESS_FLAG="${HEADLESS_FLAG}" \
    ./scripts/run_dds_smoke_test.sh
  ) >> "${SUMMARY_LOG}" 2>&1 || fail "dds smoke-test phase failed"
  log "dds smoke test passed"
}

run_tracking_phase() {
  log "phase=command_tracking_and_stale_timeout"
  start_simulator "${TRACKING_SIM_LOG}" \
    "${HEADLESS_FLAG}" \
    --enable-dds \
    --enable-lowcmd-subscriber
  wait_for_dds_startup "${TRACKING_SIM_LOG}" || fail "tracking phase simulator startup failed"

  (
    cd "${REPO_ROOT}"
    python3 scripts/lowstate_listener.py \
      --dds-domain-id "${DDS_DOMAIN_ID}" \
      --duration "${TRACKING_LISTENER_DURATION_SECONDS}" \
      --joint-name "${TRACKING_LOWCMD_JOINT_NAME}" \
      --csv-path "${TRACKING_CSV_PATH}" \
      > "${TRACKING_LISTENER_LOG}" 2>&1
  ) &
  local listener_pid=$!

  sleep "${TRACKING_LISTENER_START_DELAY_SECONDS}"

  (
    cd "${REPO_ROOT}"
    python3 scripts/send_lowcmd_offset.py \
      --dds-domain-id "${DDS_DOMAIN_ID}" \
      --joint-name "${TRACKING_LOWCMD_JOINT_NAME}" \
      --offset-rad "${TRACKING_LOWCMD_OFFSET_RAD}" \
      --duration "${TRACKING_LOWCMD_DURATION_SECONDS}" \
      > "${TRACKING_LOWCMD_LOG}" 2>&1
  ) || {
    wait "${listener_pid}" || true
    dump_sim_tail "${TRACKING_SIM_LOG}"
    fail "tracking phase lowcmd send failed"
  }

  wait "${listener_pid}" || {
    dump_sim_tail "${TRACKING_SIM_LOG}"
    fail "tracking phase listener failed"
  }

  stop_simulator

  assert_log_contains "${TRACKING_LOWCMD_LOG}" "Published " "tracking phase published lowcmd samples" \
    || fail "tracking phase did not publish lowcmd samples"
  assert_log_contains "${TRACKING_LISTENER_LOG}" "target_history ${TRACKING_LOWCMD_JOINT_NAME}" "tracking phase captured target history" \
    || fail "tracking phase listener did not capture target history"
  assert_log_contains "${TRACKING_SIM_LOG}" "cached \`rt/lowcmd\` sample went stale" "tracking phase observed stale lowcmd timeout" \
    || fail "tracking phase did not observe stale lowcmd handling"
  [[ -s "${TRACKING_CSV_PATH}" ]] || fail "tracking phase CSV was not written"
  log "tracking and stale-timeout checks passed"
}

run_long_run_phase() {
  log "phase=long_run_cadence"
  (
    cd "${REPO_ROOT}"
    "${ISAACSIM_LAUNCHER}" src/main.py \
      "${HEADLESS_FLAG}" \
      --enable-dds \
      --enable-lowcmd-subscriber \
      --max-frames "${LONG_RUN_MAX_FRAMES}" \
      --lowstate-cadence-report-interval "${LONG_RUN_CADENCE_INTERVAL}"
  ) > "${LONG_RUN_LOG}" 2>&1 || fail "long-run cadence phase failed"

  assert_log_contains "${LONG_RUN_LOG}" "lowstate cadence check" "long-run phase emitted cadence diagnostics" \
    || fail "long-run cadence phase did not emit cadence diagnostics"
  assert_log_contains "${LONG_RUN_LOG}" "DDS lowstate publisher ready" "long-run phase initialized lowstate publisher" \
    || fail "long-run cadence phase did not initialize lowstate publisher"
  log "long-run cadence check passed"
}

usage() {
  cat <<EOF
Usage:
  ./scripts/run_full_validation.sh

Optional environment variables:
  ISAACSIM_PYTHON_EXE=/path/to/isaac-sim/python.sh
  DDS_DOMAIN_ID=1
  LOG_DIR=${LOG_DIR}
  SIM_STARTUP_TIMEOUT_SECONDS=90
  HEADLESS_FLAG=--headless
  STARTUP_MAX_FRAMES=300
  TRACKING_LISTENER_DURATION_SECONDS=5
  TRACKING_LOWCMD_DURATION_SECONDS=1.5
  TRACKING_LOWCMD_JOINT_NAME=waist_yaw_joint
  TRACKING_LOWCMD_OFFSET_RAD=0.05
  TRACKING_LISTENER_START_DELAY_SECONDS=1
  LONG_RUN_MAX_FRAMES=2400
  LONG_RUN_CADENCE_INTERVAL=500
  SIM_LOG_TAIL_LINES=120
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

ensure_launcher
mkdir -p "${LOG_DIR}"
: > "${SUMMARY_LOG}"

log "repo_root=${REPO_ROOT}"
log "logs=${LOG_DIR}"
log "isaac_sim_launcher=${ISAACSIM_LAUNCHER}"
log "starting full validation"

run_unit_tests
run_startup_determinism_phase
run_smoke_phase
run_tracking_phase
run_long_run_phase

log "RESULT: full validation passed."
log "artifacts:"
log "  ${UNIT_TEST_LOG}"
log "  ${STARTUP_RUN1_LOG}"
log "  ${STARTUP_RUN2_LOG}"
log "  ${STARTUP_DIFF_LOG}"
log "  ${SMOKE_LOG_DIR}"
log "  ${TRACKING_SIM_LOG}"
log "  ${TRACKING_LISTENER_LOG}"
log "  ${TRACKING_LOWCMD_LOG}"
log "  ${TRACKING_CSV_PATH}"
log "  ${LONG_RUN_LOG}"
log "  ${SUMMARY_LOG}"
