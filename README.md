# unitree_g1_isaac_sim

This branch, `transition/unitree_ros2`, is focused on shifting the
simulator's low-level integration path from `unitree_sdk2py` to
`unitree_ros2`.

## Current Branch Goal

The immediate implementation target is the pre-Milestone 1 work in
[Agents/implementation_plan_ros2.md](Agents/implementation_plan_ros2.md):

- remove `unitree_sdk2py` runtime dependence from the simulator-facing
  low-level path
- replace message construction / parsing with the ROS 2-facing path
- replace publisher / subscriber transport with the ROS 2 / CycloneDDS path
- replace manual CRC handling if the ROS 2 path no longer requires it

## Current Architecture

The current transition implementation now uses a sidecar bridge:

- Isaac Sim process:
  - extracts robot state
  - applies incoming commands
  - communicates with a localhost bridge over UDP
- ROS 2 sidecar process:
  - runs on system Python
  - publishes `/lowstate`
  - subscribes `/lowcmd`
  - uses `unitree_hg` messages from the built `unitree_ros2` workspace
- CycloneDDS / ROS 2 middleware:
  - carries the external low-level contract visible to `unitree_ros2`
    and downstream ROS 2 applications

This keeps the external target architecture intact:

- Isaac Sim -> CycloneDDS -> `unitree_ros2` -> ROS 2 applications

The sidecar exists only to avoid the Python runtime mismatch between
Isaac Sim and the local ROS 2 Humble Python environment.

## Architecture Diagram

```text
                      State Path

  +-------------------+      UDP localhost       +-------------------------+
  | Isaac Sim Runtime | -----------------------> | ROS 2 Sidecar Bridge    |
  |                   |                          | (system Python)         |
  | - read robot      |                          | - unitree_hg/LowState   |
  |   state           |                          | - rclpy publisher       |
  | - reorder to DDS  |                          | - CycloneDDS via ROS 2  |
  | - build lowstate  |                          +------------+------------+
  +-------------------+                                       |
                                                              v
                                                    +----------------------+
                                                    | CycloneDDS / ROS 2   |
                                                    | middleware layer     |
                                                    +----------+-----------+
                                                               |
                                                               v
                                                    +----------------------+
                                                    | unitree_ros2         |
                                                    | ROS 2 apps           |
                                                    +----------------------+


                     Command Path

  +-------------------+      UDP localhost       +-------------------------+
  | Isaac Sim Runtime | <----------------------- | ROS 2 Sidecar Bridge    |
  |                   |                          | (system Python)         |
  | - receive lowcmd  |                          | - unitree_hg/LowCmd     |
  | - remap DDS order |                          | - rclpy subscriber      |
  |   to sim order    |                          | - CycloneDDS via ROS 2  |
  | - apply commands  |                          +------------+------------+
  +-------------------+                                       ^
                                                              |
                                                    +---------+------------+
                                                    | unitree_ros2         |
                                                    | ROS 2 apps           |
                                                    +----------------------+
```

## Current Status

Current progress against `implementation_plan_ros2.md`:

- `Pre-Milestone 1` is in progress
- the active runtime path no longer depends on `unitree_sdk2py`
- the sidecar now exposes ROS 2 topics:
  - `/lowstate`
  - `/lowcmd`

What is still pending before Milestone 1:

- stabilize sidecar runtime / shutdown behavior fully
- verify live `lowstate` data flow end-to-end
- verify live `lowcmd` ingress end-to-end
- then implement Milestone 1 compatibility work:
  - outward IMU quaternion `wxyz`
  - real 500 Hz `lowstate`

## Documentation Split

- This `README.md` is now the active branch README for the
  `unitree_ros2` transition work.
- The previous SDK2-oriented documentation has been preserved in
  [README_unitree_sdk2py.md](README_unitree_sdk2py.md).

## Next Steps

The next implementation steps on this branch are:

1. keep the ROS 2 sidecar alive cleanly during runtime and shutdown
2. verify live `/lowstate` and `/lowcmd` traffic end-to-end
3. then proceed into Milestone 1 compatibility work:
   - `wxyz` outward quaternion layout
   - real 500 Hz `lowstate`
   - high-frequency `lowstate` and `/lowcmd` only
