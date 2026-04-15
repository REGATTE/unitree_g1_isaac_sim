# Unitree SDK2 Python Runtime Implementation Plan

## Goal

Add a `unitree_sdk2py` runtime path so external Python policy code can talk to
the simulator through the same Unitree SDK2 Python APIs it expects on the real
robot.

The simulator should keep the ROS 2 observation path running for navigation,
debugging, and tooling, while allowing exactly one Unitree command/runtime path
to control the simulator at a time:

- ROS 2 sidecar:
  - always publishes `/rt/lowstate`
  - does not publish or apply `/rt/lowcmd` unless explicitly enabled with a flag
- native Unitree C++ SDK sidecar:
  - optional Unitree SDK2 DDS path
  - disabled by default
  - cannot run at the same time as the `unitree_sdk2py` path
- Unitree SDK2 Python runtime:
  - optional Unitree SDK2 Python DDS path for policy integration
  - enabled by default as the primary policy runtime
  - cannot run at the same time as the native C++ SDK path

The immediate reason for this branch is policy execution. The policy stack
expects `unitree_sdk2py`, so this branch should provide a simulator runtime mode
that can publish `LowState_` and consume `LowCmd_` through `unitree_sdk2py`
without making ROS 2 lowstate disappear.

## Required Runtime Constraint

The runtime must enforce these command and bridge authority rules:

1. ROS 2 runtime is always running for lowstate observation.
   - `/rt/lowstate` remains available by default.
   - `/clock` and sensor topics remain ROS 2-facing when enabled.
   - ROS 2 `/rt/lowcmd` is disabled by default.
   - ROS 2 `/rt/lowcmd` may only affect the simulator when an explicit flag
     enables it.

2. Only one Unitree SDK runtime may be active at a time.
   - If native C++ Unitree SDK runtime is active, `unitree_sdk2py` runtime must
     be inactive.
   - If `unitree_sdk2py` runtime is active, native C++ Unitree SDK runtime must
     be inactive.
   - Startup must reject configurations that enable both.

3. Only one lowcmd authority may write to the articulation.
   - ROS 2 lowcmd, native C++ SDK lowcmd, and `unitree_sdk2py` lowcmd are
     mutually exclusive.
   - The selected command source must still pass through the existing
     `LowCmdCache` and `RobotCommandApplier` safety checks.

## Desired Runtime Shapes

### Policy Mode: ROS 2 Observation + SDK2 Python Control

```text
Isaac Sim
  -> ROS 2 sidecar -> /rt/lowstate
  -> unitree_sdk2py runtime -> rt/lowstate

unitree_sdk2py policy
  -> rt/lowcmd -> unitree_sdk2py runtime -> RobotCommandApplier -> Isaac Sim
```

Expected launch shape:

```bash
isaac_sim_python src/main.py \
  --headless \
  --enable-dds \
  --enable-ros2-lowstate \
  --no-enable-ros2-lowcmd \
  --enable-unitree-sdk2py-lowstate \
  --enable-unitree-sdk2py-lowcmd
```

Because SDK2 Python is the target default, the explicit SDK2 Python flags above
should be optional once Phase 0 lands. They are shown here to make the active
runtime mode clear.

### Native C++ SDK Mode: ROS 2 Observation + Native SDK Control

This remains the current native runtime mode:

```bash
isaac_sim_python src/main.py \
  --headless \
  --enable-dds \
  --enable-ros2-lowstate \
  --no-enable-ros2-lowcmd \
  --enable-native-unitree-lowstate \
  --enable-native-unitree-lowcmd \
  --no-enable-unitree-sdk2py-lowstate \
  --no-enable-unitree-sdk2py-lowcmd
```

### ROS 2 Command Test Mode

This mode is only for explicit ROS 2 lowcmd testing:

```bash
isaac_sim_python src/main.py \
  --headless \
  --enable-dds \
  --enable-ros2-lowstate \
  --enable-ros2-lowcmd \
  --no-enable-native-unitree-lowcmd \
  --no-enable-unitree-sdk2py-lowcmd
```

## Non-Goals

- Do not implement the locomotion policy in this repository.
- Do not remove the ROS 2 sidecar lowstate path.
- Do not replace the native C++ SDK bridge; keep it available as a separate
  runtime mode.
- Do not allow multiple lowcmd sources to race.
- Do not bypass joint-order validation, stale-command handling, or bounded
  command safety checks.
- Do not make Isaac Sim depend on `unitree_sdk2py` unless the SDK2 Python
  runtime mode is explicitly enabled.

## Starting Point

The current `main` branch already has:

- ROS 2 sidecar lowstate and optional lowcmd:
  - `src/dds/ros2/manager.py`
  - `scripts/ros2_cyclonedds_sidecar.py`
- native C++ Unitree SDK sidecar:
  - `src/dds/native/manager.py`
  - `native_sdk_bridge/`
- shared command representation:
  - `src/dds/common/lowcmd_types.py`
  - `RobotCommandApplier.apply_lowcmd(...)`
- single lowcmd authority validation for ROS 2 versus native C++ SDK:
  - `src/config.py`
- runtime orchestration:
  - `src/main.py`

This branch should add the SDK2 Python runtime beside the existing ROS 2 and
native C++ paths rather than rewiring the whole DDS package.

## Proposed CLI / Config Additions

Add explicit SDK2 Python flags:

```bash
--enable-unitree-sdk2py-lowstate / --no-enable-unitree-sdk2py-lowstate
--enable-unitree-sdk2py-lowcmd / --no-enable-unitree-sdk2py-lowcmd
--unitree-sdk2py-domain-id 1
--unitree-sdk2py-lowstate-topic rt/lowstate
--unitree-sdk2py-lowcmd-topic rt/lowcmd
--unitree-sdk2py-network-interface lo
```

Recommended defaults:

- ROS 2 lowstate: enabled
- ROS 2 lowcmd: disabled
- native C++ Unitree lowstate: disabled by default
- native C++ Unitree lowcmd: disabled by default
- SDK2 Python lowstate: enabled by default
- SDK2 Python lowcmd: enabled by default
- SDK2 Python domain id: default to `--dds-domain-id`
- SDK2 Python topics: `rt/lowstate` and `rt/lowcmd`

The branch default should become:

```bash
isaac_sim_python src/main.py --headless
```

Meaning:

- ROS 2 `/rt/lowstate` is published.
- ROS 2 `/rt/lowcmd` is not applied.
- SDK2 Python `rt/lowstate` is published.
- SDK2 Python `rt/lowcmd` is the active command source.
- native C++ Unitree SDK runtime is not started.

## Phase 0: Configuration and Authority Model

Purpose:

- make the runtime modes explicit before adding transport code
- prevent ambiguous command authority early

Work items:

1. Extend `AppConfig` with SDK2 Python lowstate/lowcmd options.
2. Add CLI flags for SDK2 Python runtime control.
3. Add preflight validation:
   - ROS 2 lowcmd + native C++ lowcmd is invalid
   - ROS 2 lowcmd + SDK2 Python lowcmd is invalid
   - native C++ lowcmd + SDK2 Python lowcmd is invalid
   - native C++ lowstate/lowcmd + SDK2 Python lowstate/lowcmd is invalid
4. Keep ROS 2 lowstate enabled by default and document that it is the always-on
   observation path.
5. Change default Unitree runtime authority:
   - SDK2 Python lowstate enabled by default
   - SDK2 Python lowcmd enabled by default
   - native C++ SDK lowstate disabled by default
   - native C++ SDK lowcmd disabled by default
6. Update tests for valid and invalid runtime combinations.

Deliverables:

- config flags and validation in `src/config.py`
- tests proving mutual exclusion and ROS 2 lowcmd opt-in behavior
- tests proving default config selects SDK2 Python and does not start native
  C++ SDK runtime
- documentation note in README/config docs

## Phase 1: SDK2 Python Lowstate Publisher

Purpose:

- expose simulator state to SDK2 Python policy clients

Work items:

1. Add `src/dds/sdk2py/` package.
2. Implement an SDK2 Python manager skeleton, for example
   `UnitreeSdk2PyDdsManager`.
3. Import `unitree_sdk2py` only inside the SDK2 Python runtime path so normal
   ROS 2/native C++ launches do not fail when the Python SDK is missing.
4. Map `RobotKinematicSnapshot` into SDK2 Python `LowState_`.
5. Reuse existing DDS-order conversion for all body-joint fields.
6. Handle CRC exactly as required by `unitree_sdk2py`.
7. Add lifecycle methods:
   - `initialize()`
   - `step(simulation_time_seconds, snapshot)`
   - `shutdown()`
   - `reset_runtime_state()`
8. Keep publish cadence aligned with the existing `lowstate_publish_hz`
   scheduler.

Deliverables:

- SDK2 Python lowstate publication on `rt/lowstate`
- graceful warning or startup failure policy when `unitree_sdk2py` is missing
- unit tests around message mapping and manager lifecycle, with SDK imports
  mocked where needed

## Phase 2: SDK2 Python Lowcmd Subscriber

Purpose:

- allow Python policy code to command the simulator through SDK2 Python
  `rt/lowcmd`

Work items:

1. Subscribe to SDK2 Python `LowCmd_` on the configured topic.
2. Validate message width and body-joint field availability.
3. Convert incoming command fields into the shared `LowCmdCache`.
4. Apply stale-command timeout through the existing common timing helper.
5. Route SDK2 Python `latest_lowcmd` through `resolve_active_lowcmd(...)`.
6. Preserve bounded motion rejection in `RobotCommandApplier`.
7. Add tests for:
   - fresh command application
   - stale command drop
   - extra motor slots are clamped or rejected consistently with the native
     path
   - short/incomplete command messages are rejected

Deliverables:

- SDK2 Python lowcmd ingress
- one active command source selected at runtime
- policy-compatible command loop without ROS 2 lowcmd participation

## Phase 3: Runtime Orchestration

Purpose:

- make SDK2 Python mode a first-class runtime mode in `src/main.py`

Work items:

1. Instantiate `UnitreeSdk2PyDdsManager` only when SDK2 Python lowstate or
   lowcmd is enabled.
2. Keep `DdsManager` active whenever ROS 2 lowstate or explicit ROS 2 lowcmd is
   enabled.
3. Disable native C++ manager startup whenever SDK2 Python mode is active.
4. Update `resolve_active_lowcmd(...)` to consider:
   - SDK2 Python lowcmd
   - native C++ SDK lowcmd
   - ROS 2 lowcmd
   while relying on config preflight to guarantee only one can be active.
5. Reset and shut down all active managers cleanly.
6. Log active runtime mode clearly at startup.

Deliverables:

- `src/main.py` can launch ROS 2 + SDK2 Python policy mode
- clean startup logs identify the selected Unitree runtime
- no native C++ bridge process starts in SDK2 Python mode

## Phase 4: Policy Smoke Test Tooling

Purpose:

- prove the SDK2 Python path works with the policy-facing API surface

Work items:

1. Add a smoke-test script, for example
   `scripts/run_sdk2py_policy_smoke_test.sh`.
2. Add a startup hygiene step before launching Isaac Sim:
   - detect orphaned repo-owned sidecars from prior interrupted runs
     (`ros2_cyclonedds_sidecar.py`, `unitree_sdk2py_sidecar.py`, and native
     bridge helpers)
   - terminate only processes whose command line points at this repository
   - wait briefly for clean exit, then report any remaining process IDs instead
     of killing unrelated user processes
   - restart or bypass the ROS 2 daemon for the smoke-test shell so stale graph
     discovery does not hide the current run state
3. Launch Isaac Sim in ROS 2 lowstate + SDK2 Python lowstate/lowcmd mode.
4. Start an SDK2 Python lowstate listener.
5. Send a conservative SDK2 Python lowcmd offset.
6. Confirm:
   - ROS 2 `/rt/lowstate` is still visible
   - SDK2 Python `rt/lowstate` is visible
   - SDK2 Python `rt/lowcmd` reaches the simulator
   - native C++ SDK bridge is not running
   - ROS 2 lowcmd remains disabled
   - no stale sidecar process from an earlier run is reused
7. Write logs under `tmp/sdk2py_policy_smoke_logs/`.

Deliverables:

- repeatable SDK2 Python policy smoke test
- conservative startup hygiene for interrupted simulator restarts
- documented expected log markers
- validation summary suitable for PR review

## Phase 5: Documentation and Cleanup

Purpose:

- make the new runtime mode understandable and maintainable

Work items:

1. Update `README.md` runtime overview.
2. Update `config.md` with SDK2 Python flags.
3. Add `docs/context_unitree_sdk2py_runtime.md`.
4. Update `docs/context_native_bridge.md` to state native C++ and SDK2 Python
   runtime modes are mutually exclusive.
5. Update `docs/context_ros2_runtime.md` to state ROS 2 lowstate remains the
   always-on observation path and ROS 2 lowcmd is explicit opt-in only.
6. Add troubleshooting notes for:
   - missing `unitree_sdk2py`
   - missing CRC shared library
   - DDS domain mismatch
   - accidental native C++ bridge activation

Deliverables:

- docs match the runtime behavior
- policy-mode launch instructions are copy-pastable
- known environment failures are documented

## Validation Checklist

Before merging this branch:

- `python3 -m pytest -q`
- config validation tests for all lowcmd authority combinations
- SDK2 Python manager tests with SDK imports mocked
- native C++ mode still starts when SDK2 Python mode is disabled
- SDK2 Python mode starts without launching the native C++ bridge
- ROS 2 `/rt/lowstate` is visible in SDK2 Python policy mode
- ROS 2 `/rt/lowcmd` does not control the simulator unless explicitly enabled
- SDK2 Python `rt/lowstate` is visible to an SDK2 Python listener
- SDK2 Python `rt/lowcmd` moves one bounded joint in the simulator
- stale SDK2 Python lowcmd stops being applied

## Risks

- `unitree_sdk2py` may require native shared libraries that are not installed in
  the Isaac Sim Python environment.
- The SDK2 Python CRC helper may fail if `crc_amd64.so` is missing from the
  package install.
- Running ROS 2 and SDK2 Python in the same process may expose CycloneDDS
  library or environment conflicts. If this happens, convert the SDK2 Python
  runtime to a Python sidecar process instead of importing it into Isaac Sim.
- SDK2 Python and native C++ SDK clients may collide on the same DDS topics if
  both are accidentally launched outside the simulator process. Keep DDS domain
  and runtime-mode checks explicit.

## Sidecar Fallback Decision Point

Start with an in-process SDK2 Python manager only if importing
`unitree_sdk2py` inside Isaac Sim is stable.

If import, DDS initialization, or shared-library behavior is unstable inside
Isaac Sim, switch to a sidecar model:

```text
Isaac Sim <-> localhost UDP <-> SDK2 Python sidecar <-> Unitree SDK2 DDS
```

That fallback mirrors the existing ROS 2 and native C++ bridge designs and
keeps Isaac's Python environment isolated from SDK2 Python dependencies.
