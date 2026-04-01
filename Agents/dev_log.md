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
