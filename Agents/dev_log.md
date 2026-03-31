# Dev Log

## 2026-03-30 21:14:21 PDT

- Read `src/implementation_plan.md`.
- Built the first standalone Isaac Sim launcher in `src/unitree_g1_isaac_sim/src/main.py`.
- Moved launcher args/config into `src/unitree_g1_isaac_sim/src/config.py`.
- Moved stage construction into `src/unitree_g1_isaac_sim/src/scene.py`.
- Added stage setup: `/World`, `/World/PhysicsScene`, `/World/GroundPlane`, `/World/DistantLight`, `/World/G1`.
- Updated `src/unitree_g1_isaac_sim/README.md` to use Isaac Sim `python.sh`, not `isaac-sim.sh -p`.
- Verified the intended dev command is:
  - `isaac_sim_python main.py`
- Debugged startup:
  - script now runs under `python.sh`
  - fixed crash caused by adding a duplicate `translate` xform op on the referenced G1 root prim

## Resume Here

- Re-read `src/implementation_plan.md`.
- Then continue from articulation discovery and joint-state reading.
- Next file to add is likely `src/unitree_g1_isaac_sim/src/robot_state.py`.
- Goal of the next step:
  - find the G1 articulation under `/World/G1`
  - list joint names in simulator order
  - read joint positions/velocities/efforts
  - print them from `main.py` for validation before building DDS
