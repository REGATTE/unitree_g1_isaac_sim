# Full Validation Checklist

This checklist describes the intended "full test" path for the current
baseline simulator. The automated parts are bundled in:

```bash
./scripts/run_full_validation.sh
```

That harness launches Isaac Sim as needed and covers the repeatable repo-side
validation phases below.

## Automated Phases

### 1. Unit / Regression Tests

Purpose:

- catch mapping, DDS, pause-safe, command-application, and listener regressions

Covered by:

```bash
python3 -m unittest \
  tests/test_g1_lowcmd.py \
  tests/test_dds_manager.py \
  tests/test_robot_control.py \
  tests/test_robot_state_pause_safe.py \
  tests/test_lowstate_listener.py
```

Pass criteria:

- all tests pass

### 2. Deterministic Startup Snapshot

Purpose:

- confirm two fresh launches begin from the same canonical simulator-visible state

Covered by:

- two bounded headless simulator launches
- exact diff of the extracted startup snapshot markers from:
  - `applied deterministic startup state`
  - base pose / quaternion
  - startup sim-order joint preview
  - startup DDS-order joint preview

Pass criteria:

- both runs apply deterministic startup state
- extracted startup snapshots are identical

### 3. DDS Smoke Test

Purpose:

- confirm end-to-end DDS transport is working

Covered by:

```bash
./scripts/run_dds_smoke_test.sh
```

The full-validation harness reuses this script inside its own log directory.

Pass criteria:

- lowstate publisher ready
- lowcmd subscriber ready
- simulator receives external `rt/lowcmd`
- listener receives `rt/lowstate`
- listener reports `crc_rejected=0`
- sender publishes commands
- final result is:
  - `DDS communication working end-to-end.`

### 4. Command Tracking and Stale-Timeout Behavior

Purpose:

- confirm a conservative command can be injected while a listener captures the
  same window
- confirm stale-command handling occurs after the publish burst stops

Covered by:

- one headless simulator run
- `scripts/lowstate_listener.py` with `--csv-path`
- `scripts/send_lowcmd_offset.py` against `waist_yaw_joint` by default

Pass criteria:

- lowcmd sender publishes samples
- listener prints `target_history ...`
- CSV is written
- simulator log contains:
  - `cached \`rt/lowcmd\` sample went stale`

### 5. Longer Cadence Run

Purpose:

- confirm cadence diagnostics still appear over a longer bounded run

Covered by:

- one bounded headless DDS run with cadence reporting enabled

Pass criteria:

- simulator log contains `lowstate cadence check`
- no startup/runtime failure occurs

## Manual / Human Review After the Harness

The automated harness proves a lot, but not everything.

### 1. Inspect Tracking CSV / Listener Output

Review:

- joint moves in the expected direction
- no obviously unstable trajectory
- peaks are plausible for the commanded offset

Artifacts:

- `tracking_waist_yaw.csv`
- `tracking_listener.log`
- `tracking_lowcmd.log`

### 2. External SDK Client Compatibility

Purpose:

- prove that a non-repo Unitree SDK-based client can interact with the
  simulator without simulator-specific code changes

Status:

- not currently automated by the full-validation harness
- still recommended before calling a release "fully validated"

### 3. Explicit In-Session Reset Validation

Purpose:

- prove that a runtime reset path returns the simulator to the same canonical
  state, not just a fresh-launch startup path

Status:

- startup determinism is implemented and covered
- a dedicated in-session reset entrypoint is still future work

## Recommended Command

```bash
cd /home/regastation/Workspaces/murali_ws/src/unitree_g1_isaac_sim
./scripts/run_full_validation.sh
```

## Key Environment Overrides

```bash
ISAACSIM_PYTHON_EXE=/path/to/isaac-sim/python.sh
DDS_DOMAIN_ID=1
LOG_DIR=/tmp/full_validation_logs
TRACKING_LOWCMD_JOINT_NAME=waist_yaw_joint
TRACKING_LOWCMD_OFFSET_RAD=0.05
LONG_RUN_MAX_FRAMES=2400
```
