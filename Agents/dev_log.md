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

## 2026-03-30 21:14:21 PDT Update

- Added `src/robot_state.py`.
- `main.py` now initializes a G1 articulation reader after scene creation.
- `main.py` now reads and prints a bounded joint-state snapshot before entering the main loop.
- Added proper `World` creation/reset before articulation initialization.
- Verified articulation startup now works and detects 29 joints on the `29dof` asset.
- Added `--print-all-joints` to print the full simulator joint order before mapping work.

## Resume Here

- Re-read `Agents/implementation_plan.md`.
- Run `isaac_sim_python main.py --print-all-joints`.
- Freeze the full simulator joint order from `src/robot_state.py`.
- After that, implement joint-order mapping for DDS compatibility.
