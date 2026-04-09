# unitree_g1_isaac_sim

Isaac Sim environment for the Unitree G1 with a ROS 2 / CycloneDDS bridge
path for low-level state and command exchange.

This README is user-facing. Implementation progress and branch notes are
tracked in [Agents/dev_log.md](Agents/dev_log.md).

## Overview

The current low-level integration path uses:

- Isaac Sim for robot state extraction and command application
- a localhost sidecar bridge for ROS 2 message I/O
- `unitree_ros2` message definitions (`unitree_hg`)
- CycloneDDS as the ROS 2 middleware

The intended external flow is:

- simulator state out: Isaac Sim -> ROS 2 / CycloneDDS -> ROS 2 apps
- command ingress: ROS 2 apps -> ROS 2 / CycloneDDS -> Isaac Sim

## Architecture

```text
                      State Path

  +-------------------+      UDP localhost       +-------------------------+
  | Isaac Sim Runtime | -----------------------> | ROS 2 Sidecar Bridge    |
  |                   |                          | (system Python)         |
  | - read robot      |                          | - unitree_hg/LowState   |
  |   state           |                          | - rclpy publisher       |
  | - map to DDS      |                          | - CycloneDDS via ROS 2  |
  | - send lowstate   |                          +------------+------------+
  +-------------------+                                       |
                                                              v
                                                    +----------------------+
                                                    | ROS 2 / CycloneDDS   |
                                                    +----------+-----------+
                                                               |
                                                               v
                                                    +----------------------+
                                                    | unitree_ros2 /       |
                                                    | ROS 2 applications   |
                                                    +----------------------+


                     Command Path

  +-------------------+      UDP localhost       +-------------------------+
  | Isaac Sim Runtime | <----------------------- | ROS 2 Sidecar Bridge    |
  |                   |                          | (system Python)         |
  | - receive lowcmd  |                          | - unitree_hg/LowCmd     |
  | - remap to sim    |                          | - rclpy subscriber      |
  |   joint order     |                          | - CycloneDDS via ROS 2  |
  | - apply commands  |                          +------------+------------+
  +-------------------+                                       ^
                                                              |
                                                    +---------+------------+
                                                    | unitree_ros2 /       |
                                                    | ROS 2 applications   |
                                                    +----------------------+
```

## Requirements

- Isaac Sim 5.x installed and runnable through `isaac_sim_python`
- ROS 2 Humble available on the host
- `unitree_ros2` built locally, with the install prefix available
- CycloneDDS middleware available through ROS 2

Expected local `unitree_ros2` install prefix:

- `~/Workspaces/unitree_ros2/cyclonedds_ws/install`

You can also override that path with:

- `UNITREE_ROS2_INSTALL_PREFIX`
- `--unitree-ros2-install-prefix`

## Runtime Notes

- Isaac Sim and ROS 2 Humble use different Python runtimes on this setup.
  Because of that, ROS 2 runs in a sidecar process rather than in the Isaac
  Sim Python process.
- The default lowstate runtime target is `500 Hz`, with:
  - `--physics-dt 0.002`
  - `--lowstate-publish-hz 500`
- The localhost UDP bridge defaults are:
  - lowstate: `127.0.0.1:35501`
  - lowcmd: `127.0.0.1:35502`
- Startup attempts to clean up stale sidecar bridge processes from prior runs
  before launching a fresh bridge.
- The runtime now logs two separate lowstate cadence diagnostics:
  - `simulation_time`: cadence against the physics schedule
  - `wall_clock`: realized publish cadence against host time

## Launch

Run the simulator:

```bash
isaac_sim_python src/main.py --headless
```

Optional useful flags:

```bash
isaac_sim_python src/main.py --headless --max-frames 300
isaac_sim_python src/main.py --headless --dds-domain-id 1
isaac_sim_python src/main.py --headless --unitree-ros2-install-prefix ~/Workspaces/unitree_ros2/cyclonedds_ws/install
```

## ROS 2 Verification

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Workspaces/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
```

List topics:

```bash
ros2 topic list
```

Expected topics include:

- `/rt/lowstate`
- `/rt/lowcmd`

Inspect lowstate:

```bash
ros2 topic echo /rt/lowstate --once
```

Publish a simple lowcmd smoke-test message:

```bash
ros2 topic pub --once /rt/lowcmd unitree_hg/msg/LowCmd "{mode_pr: 0, mode_machine: 0}"
```

Check the external lowstate rate from ROS 2:

```bash
ros2 topic hz /rt/lowstate
```

Current runtime expectation:

- the configured target is `500 Hz`
- the observed end-to-end ROS 2 rate has been about `467-470 Hz`
- this is currently accepted for this branch as long as the realized rate
  remains above `450 Hz`

The simulator-side cadence log is useful when you want to compare:

- the intended lowstate cadence from the physics loop
- the realized wall-clock cadence inside the Isaac Sim process
- the external observer reported by `ros2 topic hz`

## Configuration

Useful runtime options:

- `--enable-dds` / `--no-enable-dds`
- `--dds-domain-id`
- `--lowstate-topic`
- `--lowcmd-topic`
- `--lowstate-publish-hz`
- `--unitree-ros2-install-prefix`
- `--ros2-python-exe`
- `--bridge-bind-host`
- `--bridge-lowstate-port`
- `--bridge-lowcmd-port`

## Documentation

- active implementation notes: [Agents/dev_log.md](Agents/dev_log.md)
- implementation plan: [Agents/implementation_plan_ros2.md](Agents/implementation_plan_ros2.md)
- previous SDK-oriented documentation: [README_unitree_sdk2py.md](README_unitree_sdk2py.md)
