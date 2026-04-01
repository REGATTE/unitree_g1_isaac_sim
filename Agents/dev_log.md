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
