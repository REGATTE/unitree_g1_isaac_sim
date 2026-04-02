# Dev Log

## 2026-03-30 21:14:21 PDT

- Read `Agents/implementation_plan.md`.
- Built the first standalone Isaac Sim launcher in `src/main.py`.
- Moved launcher args/config into `src/config.py`.
- Moved stage construction into `src/scene.py`.
- Added stage setup: `/World`, `/World/PhysicsScene`, `/World/GroundPlane`, `/World/DistantLight`, `/World/G1`.
- Updated `README.md` to use Isaac Sim `python.sh`, not `isaac-sim.sh -p`.
- Verified the intended dev command is:
  - `isaac_sim_python main.py`
- Debugged startup:
  - script now runs under `python.sh`
  - fixed crash caused by adding a duplicate `translate` xform op on the referenced G1 root prim

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- Then continue from articulation discovery and joint-state reading.
- Next file to add is likely `src/robot_state.py`.
- Goal of the next step:
  - find the G1 articulation under `/World/G1`
  - list joint names in simulator order
  - read joint positions/velocities/efforts
  - print them from `main.py` for validation before building DDS

## ===================================================================================================================================

## 2026-03-30 21:25:21 PDT Post-Commit Update

- Added `src/mapping/joints.py`.
- Froze the Isaac Sim 29-joint articulation order from the live `--print-all-joints` run.
- Added the Isaac Lab DDS 29-joint body order from `tasks/common_observations/g1_29dof_state.py`.
- Added explicit bidirectional index maps between simulator order and DDS order.
- Added `src/mapping/__init__.py` for clean imports.

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- Use `src/mapping/joints.py` as the canonical 29-joint mapping layer.
- Next step:
  - reorder simulator state into DDS order
  - then start `dds/g1_lowstate.py` or equivalent DDS-state publication code

## ===================================================================================================================================

## 2026-03-30 22:00:25 PDT Update

- Added `src/robot_state.py`.
- `main.py` now initializes a G1 articulation reader after scene creation.
- `main.py` now reads and prints a bounded joint-state snapshot before entering the main loop.
- Added proper `World` creation/reset before articulation initialization.
- Verified articulation startup now works and detects 29 joints on the `29dof` asset.
- Added `--print-all-joints` to print the full simulator joint order before mapping work.

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- Run `isaac_sim_python main.py --print-all-joints`.
- Freeze the simulator joint order from the live articulation output.
- Commit this state after the joint-order freeze and mapping baseline are in place.

## ===================================================================================================================================

## 2026-03-30 22:13:14 PDT Post-Commit Update

- Added `src/mapping/validator.py` and wired startup validation against the frozen sim order.
- Added `src/mapping/conversion.py` with pure helpers to reorder joint-aligned data between simulator order and DDS order.
- Startup now prints both a simulator-order preview and a short DDS-order preview from the same live snapshot.
- Verified the DDS-order preview matches the expected live 29-joint body order.

## Resume Here

- Commit this state.
- Re-read `Agents/implementation_plan.md`.
- The next step is DDS state publication.
- Use `src/mapping/conversion.py` as the canonical sim-to-DDS reorder layer.
- Wire the DDS-ordered snapshot path into the first `rt/lowstate` publisher.

## ===================================================================================================================================

## 2026-03-31 23:01:09 PDT Post-Merge Update

- Merged `fix/expand-robot-state-reader` into `main`.
- The merged branch now expands `src/robot_state.py` beyond joint-only reads and establishes the first DDS-facing kinematic state boundary.
- Added `RobotKinematicSnapshot` to capture:
  - joint positions/velocities/efforts
  - base position in world frame
  - base quaternion in `wxyz`
  - base linear/angular velocity
  - IMU-like proper acceleration in body frame
  - IMU-like angular velocity in body frame
- Tightened the new snapshot container so it behaves immutably in practice:
  - tuple-backed fields
  - normalization in `__post_init__`
- Refactored the state reader so joint-only reads stay lightweight:
  - `read_snapshot()` uses a direct joint read path
  - kinematic/IMU logic is separated from the joint-only path
- Added `ImuEmulator` to isolate temporal acceleration estimation.
- Removed wall-clock timing from acceleration estimation:
  - dynamic acceleration now depends only on simulation-derived `sample_dt`
  - when `sample_dt` is absent or invalid, the IMU path falls back to gravity-only proper acceleration
- Tightened pose handling:
  - world-pose reads use strict length validation
  - linear/angular world-velocity reads remain tolerant
- Fixed the world-pose quaternion convention bug:
  - `get_world_poses()` is now treated as scalar-first `wxyz`
  - removed the incorrect `xyzw -> wxyz` reorder on that path
  - validated this live from Isaac Sim startup output showing identity orientation as:
    - `base_quaternion_wxyz=(1.000000, 0.000000, 0.000000, 0.000000)`
- Added one-time startup logging in `main.py` for:
  - `base_position_world`
  - `base_quaternion_wxyz`
- Verified the merged branch still:
  - loads the 29-DoF USD correctly
  - validates the frozen 29-joint articulation order
  - prints correct simulator-order and DDS-order joint previews

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- The next implementation target remains the first DDS bridge slice on `main`.
- Use the merged robot-state reader as the DDS state source:
  - `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
- Next files to add:
  - `src/dds/__init__.py`
  - `src/dds/manager.py`
  - `src/dds/g1_lowstate.py`
  - `src/dds/g1_lowcmd.py`
- First goal of the next step:
  - initialize Cyclone/Unitree DDS objects
  - build and publish `LowState_` on `rt/lowstate`
  - use the existing mapping layer for body joint ordering
  - keep quaternion output in the validated `wxyz` convention

## ===================================================================================================================================

## 2026-03-31 23:04:24 PDT Post-Merge Update

- Merged `fix/validate-dds-snapshot-order` into `main`.
- Hardened `src/mapping/conversion.py` so DDS relabeling is no longer width-only validation.
- `to_dds_ordered_snapshot()` now validates `snapshot.joint_names` against the frozen simulator joint order before relabeling to DDS order.
- This closes the silent-corruption path where a future DDS publisher could reuse the conversion helper on a mismatched snapshot and publish mislabeled joint state.
- Added inline documentation in the conversion helper explaining that it is intended to be a safe standalone DDS boundary, not a blind reorder utility.
- `main` now contains both sides of the current DDS state boundary work:
  - validated simulator-order to DDS-order joint conversion
  - validated robot kinematic state extraction with the confirmed `wxyz` quaternion convention

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- The next implementation target is still the first DDS bridge slice on `main`.
- Use the merged state path in this order:
  - `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
  - `to_dds_ordered_snapshot(...)` for body joint-aligned fields
- Next files to add:
  - `src/dds/__init__.py`
  - `src/dds/manager.py`
  - `src/dds/g1_lowstate.py`
  - `src/dds/g1_lowcmd.py`
- First goal of the next step:
  - publish a correct `LowState_` message on `rt/lowstate`
  - keep joint ordering validation at the conversion boundary
  - keep base orientation in the validated Isaac Sim `wxyz` convention

## ===================================================================================================================================

## 2026-03-31 23:06:06 PDT Next Steps

- Re-read `Agents/implementation_plan.md`.
- Create a new branch fo rthe DDS dev `dds_dev`.
- Start the first DDS bridge slice.
- Add the DDS package skeleton:
  - `src/dds/__init__.py`
  - `src/dds/manager.py`
  - `src/dds/g1_lowstate.py`
  - `src/dds/g1_lowcmd.py`
- Build the first `rt/lowstate` publisher using:
  - `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
  - `to_dds_ordered_snapshot(...)` for body joint-aligned fields
  - Unitree SDK2 `LowState_` + CRC
- Keep these constraints in place:
  - body joint ordering must stay validated at the conversion boundary
  - base quaternion must stay in validated Isaac Sim `wxyz` convention
  - IMU acceleration must stay simulation-time derived, never wall-time derived
- After `rt/lowstate` works, add the first `rt/lowcmd` subscriber and map incoming body commands back into simulator order.

## ===================================================================================================================================

## 2026-03-31 DDS Dev Branch Update

- Created the DDS development branch:
  - `dds_dev`
- Started the first DDS bridge slice on `dds_dev`.
- Added the DDS package under `src/dds/`:
  - `src/dds/__init__.py`
  - `src/dds/manager.py`
  - `src/dds/g1_lowstate.py`
  - `src/dds/g1_lowcmd.py`

- Implemented the first `rt/lowstate` publication path.
  - `main.py` now wires DDS stepping into the simulation loop.
  - The DDS manager runs inside the same Isaac Sim process instead of copying the heavier Isaac Lab shared-memory/threaded structure.
  - `rt/lowstate` publication consumes:
    - `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
    - `to_dds_ordered_snapshot(...)` for 29-DoF body joint ordering
    - Unitree SDK2 `LowState_`
    - CRC generation immediately before publish

- Preserved the DDS-boundary constraints during implementation.
  - Body-joint publication still routes through the validated simulator-order to DDS-order conversion boundary.
  - Base orientation remains in validated Isaac Sim `wxyz` internally.
  - The only quaternion reorder introduced is the explicit DDS IMU packing step for the outgoing Unitree message field.
  - IMU acceleration remains simulation-time derived only.

- Added the first `rt/lowcmd` subscriber skeleton.
  - The subscriber validates CRC and caches the latest raw DDS command.
  - DDS-order command vectors can now be translated back into simulator order in one place.
  - Applying those commands into Isaac Sim articulation control is still pending.

- Added DDS-related runtime flags in `src/config.py`.
  - `--enable-dds`
  - `--dds-domain-id`
  - `--lowstate-topic`
  - `--lowcmd-topic`
  - `--lowstate-publish-hz`
  - `--enable-lowcmd-subscriber`

- Added and updated project documentation.
  - Added `Agents/constraints.md` to record the DDS implementation constraints explicitly.
  - Reworked `README.md` to make launch usage clearer.
  - Replaced the old run/args layout with a table-based argument section.
  - Moved the TODO list to the bottom of the README.

- Verified during this round:
  - `python3 -m py_compile src/config.py src/main.py src/dds/__init__.py src/dds/manager.py src/dds/g1_lowstate.py src/dds/g1_lowcmd.py`

- Reference notes captured during review:
  - The Isaac Lab reference DDS manager defaults to a `0.01` second publish interval, which is `100 Hz`.
  - The Isaac Lab G1 observation path also uses a `20 ms` DDS write throttle, which is `50 Hz` on that path.
  - No rate behavior was changed in this repo during the follow-up README clarification round.

## Resume Here

- Test `rt/lowstate` end-to-end with a real external Unitree SDK2 or `unitree_ros2` client.
- Confirm the actual publish cadence and message-field expectations against the external client path.
- Implement the next DDS-control slice:
  - take cached `rt/lowcmd` body commands
  - map them from DDS order back into simulator order
  - apply them into Isaac Sim articulation control
- After body lowcmd works, decide whether optional dex1/dex3/inspire DDS bridges are needed for the intended G1 configuration.

## ===================================================================================================================================

## 2026-04-01 DDS LowCmd Minimum Viable Update

- Implemented the minimum viable `rt/lowcmd` body-command path on `dds_dev`.
- Added `src/robot_control.py`.
  - Introduced `RobotCommandApplier` as the simulator-side owner of cached body commands.
  - This module now takes the latest validated DDS command, remaps it from DDS order back into simulator order, and applies it to the articulation.

- Extended `src/robot_state.py` beyond read-only use.
  - Added articulation write helpers for:
    - joint position targets
    - joint velocity targets
    - joint efforts
  - Kept command-width validation local to the articulation wrapper so bad lowcmd widths fail early.

- Updated `src/main.py` to wire lowcmd application into the runtime loop.
  - The latest cached `rt/lowcmd` sample is now applied before each physics step.
  - DDS initialization is performed eagerly when DDS is enabled, so both sides of the body DDS contract are brought up at startup.

- Current behavior of the minimum viable lowcmd path:
  - `rt/lowstate` publish is still active
  - `rt/lowcmd` subscribe is now active and applied to the live articulation
  - the current control slice should be treated primarily as a position-command path
  - velocity and effort fields are forwarded when matching articulation APIs are exposed by the runtime
  - `kp` and `kd` are preserved from DDS but are not dynamically applied yet

- Added DDS testing documentation.
  - Added `dds_testing.md` at the repo root.
  - Documented:
    - how to launch the simulator with DDS enabled
    - how to test `rt/lowstate`
    - how to test `rt/lowcmd`
    - a safe first validation sequence
    - current limitations and safety notes

- Verified during this round:
  - `python3 -m py_compile src/main.py src/robot_state.py src/robot_control.py src/dds/g1_lowcmd.py src/dds/g1_lowstate.py src/dds/manager.py`

- Git state for this round:
  - created commit:
    - `6d86110` `min viable lowcmd`
  - pushed branch:
    - `origin/dds_dev`

## Resume Here

- Run the documented DDS bring-up sequence in `dds_testing.md`.
- Verify that:
  - `rt/lowstate` is visible to an external Unitree client
  - `rt/lowcmd` produces visible joint motion in Isaac Sim
  - the commanded motion is reflected back on `rt/lowstate`
- After that, decide whether to:
  - implement dynamic `kp` / `kd` application
  - add optional hand DDS topics (`dex1`, `dex3`, `inspire`)
  - add utility DDS topics such as reset or simulator state

## ===================================================================================================================================

## 2026-04-01 External DDS Validation and LowCmd Width Failure

- Validated the first external DDS path against standalone `unitree_sdk2py` scripts.
  - Installed CycloneDDS locally and exported `CYCLONEDDS_HOME`.
  - Installed `unitree_sdk2py` into Isaac Sim's Python so the in-process DDS bridge could start.
  - Launched Isaac Sim with:
    - `--enable-dds`
    - `--enable-lowcmd-subscriber`
  - Confirmed runtime DDS bring-up:
    - Unitree DDS channel factory initialized on domain `1`
    - `rt/lowstate` publisher ready
    - `rt/lowcmd` subscriber ready

- Added standalone DDS validation scripts at the repo root under `scripts/`.
  - `scripts/lowstate_listener.py`
    - passive external `rt/lowstate` subscriber
    - verifies CRC and prints DDS-ordered joint and IMU previews
  - `scripts/send_lowcmd_offset.py`
    - waits for one valid `rt/lowstate` sample
    - reuses the current pose as a hold posture
    - applies a small offset on one selected DDS body joint

- Confirmed the first external `rt/lowstate` success case.
  - A standalone SDK listener received a continuous stream of lowstate samples from outside the Isaac Sim process.
  - CRC validation passed on the external receive path.
  - Joint positions, velocities, torques, and IMU fields were populated and physically reasonable.
  - This validates the direct Isaac Sim DDS architecture for the outgoing state path without the Isaac Lab shared-memory intermediary.

- Confirmed the first external `rt/lowcmd` failure mode.
  - A standalone SDK publisher successfully reached the simulator on `rt/lowcmd`.
  - The simulator crashed while converting the cached DDS command back into simulator order.
  - The runtime failure was:
    - `ValueError: Expected 29 dds-ordered values, got 35`

- Root cause identified from the live crash.
  - The current lowcmd subscriber caches `len(msg.motor_cmd)` entries from the incoming Unitree `LowCmd_`.
  - The external message shape exposed `35` motor-command slots.
  - The current G1 body mapping layer in this repo correctly expects exactly `29` body joints.
  - The remaining bug is therefore DDS-boundary width handling, not CycloneDDS setup, SDK import, topic wiring, or CRC validation.

- Documentation updated during this round.
  - Added DDS test instructions in `dds_testing.md`.
  - Added the standalone validation scripts used for external bring-up.
  - Continued README updates around setup and DDS testing flow.

## Resume Here

- Fix `src/dds/g1_lowcmd.py` so the subscriber does not trust the full incoming `motor_cmd` width.
- Explicitly consume only the 29 supported G1 body-joint command slots.
- Ignore or reject the extra incoming slots without crashing the simulator.
- Re-run the standalone `unitree_sdk2py` lowcmd test after the fix and confirm:
  - the simulator stays alive
  - the commanded body joint moves
  - the motion is reflected back on `rt/lowstate`

## ===================================================================================================================================

## 2026-04-01 DDS Baseline Test Reconciliation

- Re-read `Agents/implementation_plan.md` and reconciled the current DDS branch state against the planned implementation phases.
  - Phase 1 is effectively frozen for the baseline body DDS contract:
    - `rt/lowstate`
    - `rt/lowcmd`
    - Unitree SDK2 message classes
    - CRC handling
    - 29-DoF body joint ordering
  - Phase 4 is externally validated for the outgoing `rt/lowstate` path.
  - Phase 5 is partially validated for the incoming `rt/lowcmd` path:
    - command traffic reaches the simulator
    - the body-command application path exists on `dds_dev`
    - one remaining DDS-boundary width issue was discovered during external testing

- Confirmed the current baseline DDS smoke test on `dds_dev`.
  - Launch Isaac Sim with:
    - `--enable-dds`
    - `--enable-lowcmd-subscriber`
  - Use a standalone `unitree_sdk2py` listener to verify:
    - `rt/lowstate` is externally visible
    - CRC passes on the receive path
    - joint and IMU fields are populated
  - Use a standalone `unitree_sdk2py` publisher to send one conservative body-joint command on `rt/lowcmd`.

- Confirmed what the baseline test now proves for this repo.
  - An external Unitree SDK-based client can subscribe to simulator `rt/lowstate`.
  - An external Unitree SDK-based client can publish simulator `rt/lowcmd`.
  - The direct single-process Isaac Sim DDS design is now externally validated for the basic body DDS path, which is the central contract defined in the implementation plan.

- Clarified the current branch limitation after reconciliation.
  - The external `LowCmd_` path still exposes a 35-slot message shape.
  - The current `dds_dev` mapping layer still expects exactly 29 body-joint slots at the DDS boundary.
  - A focused follow-up branch was created to clamp incoming lowcmd parsing to the supported 29 body joints and avoid this crash.
  - That width-handling fix is still a merge follow-up relative to this `dds_dev` log state.

- Documentation implication from this round.
  - README should now present the standalone listener/publisher sequence as the baseline DDS smoke test for the repo.
  - README wording should reflect that body-command application exists on `dds_dev`, even though wider incoming `LowCmd_` widths still need the follow-up fix branch.

## Resume Here

- Re-run the baseline DDS smoke test after that merge:
  - listener on `rt/lowstate`
  - conservative publisher on `rt/lowcmd`
- Confirm on `dds_dev` that:
  - the simulator stays alive on the 35-slot `LowCmd_` shape
  - the commanded body joint moves in Isaac Sim
  - the motion is reflected back on `rt/lowstate`
- After that, decide whether the next follow-up should be:
  - dynamic `kp` / `kd` application
  - stricter publish-rate validation
  - optional hand DDS topics

## ===================================================================================================================================

## 2026-04-01 DDS LowCmd Width Handling Merged

- Merged the DDS-boundary width fix for incoming `LowCmd_` messages into `dds_dev`.
  - `src/dds/g1_lowcmd.py` no longer trusts the full incoming `motor_cmd` width.
  - The subscriber now:
    - rejects messages shorter than the supported 29 body joints
    - consumes only the first 29 body-joint slots from wider messages
    - ignores extra slots without crashing the simulator
    - logs the unexpected incoming width once per width value

- This closes the externally observed crash path from the standalone DDS test.
  - The previous failure was:
    - `ValueError: Expected 29 dds-ordered values, got 35`
  - The fix keeps the DDS boundary aligned with the frozen 29-DoF body mapping instead of letting extra message slots leak into the simulator-order conversion helpers.

- Project-state reconciliation after the merge:
  - `rt/lowstate` external visibility is already validated.
  - `rt/lowcmd` transport reachability is already validated.
  - The remaining baseline bring-up task is to re-run the external smoke test and confirm visible conservative joint motion with the merged width handling in place.

## Resume Here

- Re-run the baseline DDS smoke test on `dds_dev`:
  - listener on `rt/lowstate`
  - conservative publisher on `rt/lowcmd`
- Confirm that:
  - the simulator stays alive on the 35-slot `LowCmd_` shape
  - the targeted body joint moves in Isaac Sim
  - the motion is reflected back on `rt/lowstate`
- After that, the most likely next implementation choice is one of:
  - dynamic `kp` / `kd` application
  - stricter publish-rate and real-contract validation
  - optional hand DDS topics

## ===================================================================================================================================

## 2026-04-02 Pause-Safe Runtime Hardening

- Created the pause-safe follow-up branch:
  - `feat/pause-safe-runtime`

- Re-ran the baseline external DDS smoke test on the branch.
  - Confirmed again that:
    - `rt/lowstate` is externally visible
    - CRC passes on the external receive path
    - wider `35`-slot incoming `LowCmd_` traffic no longer crashes the simulator
    - conservative body-joint commands produce visible motion and are reflected back on `rt/lowstate`

- Identified a separate runtime edge case during manual testing.
  - If Isaac Sim is paused or is entering shutdown, articulation APIs can temporarily stop exposing a valid physics simulation view.
  - In that state, Isaac Sim starts returning empty joint buffers and warning:
    - `Physics Simulation View is not created yet ...`
  - The previous runtime treated that as fatal because the DDS state path still tried to relabel an empty simulator-order joint vector.

- Hardened the articulation boundary and main loop against that condition.
  - Added a recoverable `PhysicsViewUnavailableError` in `src/robot_state.py`.
  - Joint-state reads now fail explicitly when the articulation physics view drops instead of silently returning malformed empty state.
  - Joint-command application now checks that the articulation physics view is live before applying position, velocity, or effort targets.
  - `src/robot_control.py` now treats that condition as a skipped command application for the affected frame.
  - `src/main.py` now skips DDS publication for frames where the articulation physics view is unavailable instead of aborting the whole runtime.
  - When the physics view returns, the runtime logs that DDS/state updates are resuming.

- Added pure-Python regression coverage.
  - Added `tests/test_g1_lowcmd.py`:
    - covers the merged `35`-slot lowcmd width handling
    - covers rejection of short lowcmd messages
  - Added `tests/test_robot_state_pause_safe.py`:
    - covers recoverable failure on empty articulation joint-state buffers
    - covers recovery logging state reset when valid joint buffers return

- Documentation and repo-state reconciliation carried forward on this branch.
  - `README.md` no longer claims the lowcmd width fix is still pending.
  - The dev log now reflects both:
    - the merged DDS width-handling fix
    - the pause-safe runtime hardening follow-up

- Verified during this round:
  - `python3 -m unittest tests/test_g1_lowcmd.py tests/test_robot_state_pause_safe.py`
  - `python3 -m py_compile src/main.py src/robot_state.py src/robot_control.py tests/test_robot_state_pause_safe.py`

## Resume Here

- Keep this branch as the staging point before the next DDS follow-up.
- The next implementation work should now focus on one of:
  - dynamic `kp` / `kd` application
  - stricter publish-rate and real-contract validation
  - optional hand DDS topics
