
# unitree_g1_isaac_sim

Isaac Sim environment for the Unitree G1 with ROS 2, Unitree SDK2 Python, and
native C++ Unitree SDK bridge paths for low-level state and command exchange.

This README is user-facing. Implementation progress and branch notes are
tracked in [Agents/dev_log.md](Agents/dev_log.md).

## Overview

The current low-level integration path uses:

- Isaac Sim for robot state extraction and command application
- a localhost sidecar bridge for ROS 2 `unitree_hg` message I/O
- a Unitree SDK2 Python sidecar bridge for default policy-facing lowstate I/O
- an optional native C++ Unitree SDK sidecar bridge for SDK-level DDS I/O
- Isaac's native ROS 2 bridge for high-bandwidth simulated sensor topics
- `unitree_ros2` message definitions (`unitree_hg`)
- CycloneDDS as the ROS 2 middleware

The current default command-authority mode is:

- ROS 2 lowstate enabled: simulator publishes `/rt/lowstate`
- SDK2 Python lowstate enabled: simulator publishes SDK `rt/lowstate`
- SDK2 Python lowcmd enabled: SDK2 Python `rt/lowcmd` drives the simulator
- native C++ Unitree SDK runtime disabled unless explicitly enabled
- ROS 2 lowcmd disabled: `/rt/lowcmd` is not the active command source

The intended external flow is:

- ROS 2 state out: Isaac Sim -> ROS 2 sidecar -> `/rt/lowstate`
- SDK2 Python state out: Isaac Sim -> SDK2 Python sidecar -> `rt/lowstate`
- SDK2 Python command ingress: `rt/lowcmd` -> SDK2 Python sidecar -> Isaac Sim
- native C++ SDK state/command path is still available as an alternate runtime
  mode, but it is mutually exclusive with SDK2 Python
- simulated LiDAR out: Isaac RTX LiDAR -> Isaac ROS 2 bridge -> ROS 2 apps

## Architecture

```text
                    Low-Level State + Command Bridges

                              +----------------------+
                              | Isaac Sim Runtime    |
                              |                      |
                              | - read robot state   |
                              | - map joint order    |
                              | - send lowstate      |
                              | - receive lowcmd     |
                              | - apply commands     |
                              +----------+-----------+
                                         |
        +--------------------------------+-------------------------------+
        |                                |                               |
        v                                v                               v
+------------------------------+ +------------------------------+ +------------------------------+
| ROS 2 Sidecar Bridge         | | Unitree SDK2 Python Sidecar  | | Native C++ SDK Sidecar       |
| (system Python)              | | (system Python)              | | (system C++)                 |
|                              | |                              | |                              |
| - LowState publisher         | | - LowState_ publisher        | | - LowState_ publisher        |
| - LowCmd subscriber only     | | - LowCmd_ subscriber         | | - LowCmd_ subscriber         |
|   when explicitly enabled    | |                              | |                              |
| - rclpy + CycloneDDS         | | - unitree_sdk2py + DDS       | | - unitree_sdk2 + DDS         |
| - UDP: 127.0.0.1:35501/35502 | | - UDP: 127.0.0.1:35521       | | - UDP: 127.0.0.1:35511/35512 |
+---------------+--------------+ +---------------+--------------+ +---------------+--------------+
                |                                |                                |
                v                                v                                v
+------------------------------+ +------------------------------+ +------------------------------+
| ROS 2 / CycloneDDS           | | Unitree SDK DDS              | | Unitree SDK DDS              |
|                              | |                              | |                              |
| - /rt/lowstate               | | - rt/lowstate                | | - rt/lowstate                |
| - /rt/lowcmd opt-in only     | | - rt/lowcmd                  | | - rt/lowcmd                  |
+---------------+--------------+ +---------------+--------------+ +---------------+--------------+
                |                                |                                |
                v                                v                                v
+------------------------------+ +------------------------------+ +------------------------------+
| ROS 2 applications           | | SDK2 Python policy clients   | | Native Unitree SDK clients   |
| unitree_ros2, bringup, tools | | policy, validation tools     | | validation tools, SDK apps   |
+------------------------------+ +------------------------------+ +------------------------------+

Only one Unitree SDK runtime column may be active at a time:

- default: Unitree SDK2 Python sidecar active, native C++ SDK sidecar inactive
- alternate: native C++ SDK sidecar active, Unitree SDK2 Python sidecar inactive
- ROS 2 lowstate remains available in both modes
- ROS 2 lowcmd is hidden and inactive unless explicitly enabled


                  Simulated MID360 LiDAR Path

  +-------------------+       Isaac ROS 2 bridge     +----------------------+
  | Isaac RTX LiDAR   | ---------------------------> | ROS 2 PointCloud2    |
  |                   |                              | /livox/lidar         |
  | - MID360-like     |                              | frame_id=mid360_link |
  |   scan attributes |                              +----------------------+
  | - URDF-matched    |
  |   mount frame     |
  +-------------------+
```

## Requirements

- Isaac Sim 5.x installed and runnable through `isaac_sim_python`
- ROS 2 Humble available on the host
- `unitree_ros2` built locally, with the install prefix available
- `unitree_sdk2py` installed or available at `~/unitree_sdk2_python`
- CycloneDDS middleware available through ROS 2
- `unitree_sdk2` built locally only when using the optional native C++ bridge

Expected local `unitree_ros2` install prefix:

- `~/Workspaces/unitree_ros2/cyclonedds_ws/install`

You can also override that path with:

- `UNITREE_ROS2_INSTALL_PREFIX`
- `--unitree-ros2-install-prefix`

Expected local `unitree_sdk2` checkout/build for native bridge support:

- `~/unitree_sdk2`

## Install

Follow [Installation](docs/installation.md)

## Native Unitree SDK Bridge Build

The native C++ bridge lives in `native_sdk_bridge/` and links against
`unitree_sdk2`. Build `unitree_sdk2` first, then build the simulator bridge:

```bash
cmake -S native_sdk_bridge -B native_sdk_bridge/build
cmake --build native_sdk_bridge/build -j4
```

The default simulator config expects the bridge executable at:

```text
native_sdk_bridge/build/unitree_g1_native_bridge
```

The same CMake build also produces native SDK validation tools used by the
mixed-mode smoke test:

```text
native_sdk_bridge/build/unitree_g1_native_lowstate_listener
native_sdk_bridge/build/unitree_g1_native_lowcmd_offset
```

If your SDK checkout is not at `~/unitree_sdk2`, pass the root explicitly:

```bash
cmake -S native_sdk_bridge -B native_sdk_bridge/build \
  -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2
cmake --build native_sdk_bridge/build -j4
```

Verify that the bridge resolves both CycloneDDS libraries from the Unitree SDK2
thirdparty directory, not from ROS 2:

```bash
ldd native_sdk_bridge/build/unitree_g1_native_bridge | grep ddsc
```

Expected:

```text
libddsc.so.0 => /home/<user>/unitree_sdk2/thirdparty/lib/<arch>/libddsc.so.0
libddscxx.so.0 => /home/<user>/unitree_sdk2/thirdparty/lib/<arch>/libddscxx.so.0
```

If `libddsc.so.0` resolves from `/opt/ros/humble`, the bridge may crash at
startup with `free(): invalid pointer`. See [docs/issues.md](docs/issues.md)
for the troubleshooting note.

## Runtime Notes

- Isaac Sim and ROS 2 Humble use different Python runtimes on this setup.
  Because of that, ROS 2 runs in a sidecar process rather than in the Isaac
  Sim Python process.
- The native Unitree SDK bridge is a separate C++ sidecar process. Isaac Sim
  exchanges compact localhost UDP packets with it; the sidecar handles
  `unitree_sdk2` DDS publication/subscription.
- Both lowstate paths are enabled by default. ROS 2 publishes `/rt/lowstate`;
  native SDK publishes `rt/lowstate`.
- Only one lowcmd source may be active. By default, native SDK `rt/lowcmd` is
  active and ROS 2 `/rt/lowcmd` command application is disabled. Startup
  rejects configurations that enable both command sources.
- The default lowstate runtime target is `500 Hz`, with:
  - `--physics-dt 0.002`
  - `--lowstate-publish-hz 500`
- The ROS 2 localhost UDP bridge defaults are:
  - lowstate: `127.0.0.1:35501`
  - lowcmd: `127.0.0.1:35502`
- The native SDK localhost UDP bridge defaults are:
  - lowstate: `127.0.0.1:35511`
  - lowcmd: `127.0.0.1:35512`
- Startup attempts to clean up stale ROS 2 and native sidecar bridge processes
  from prior runs before launching fresh bridges.
- A simulated Livox MID360 RTX LiDAR is enabled by default. It mounts at the
  URDF `torso_link -> mid360_link` transform and publishes `PointCloud2` on
  `/livox/lidar` through Isaac's ROS 2 bridge, not through the LowState /
  LowCmd sidecar.
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
isaac_sim_python src/main.py --headless --no-enable-native-unitree-lowcmd --enable-ros2-lowcmd
isaac_sim_python src/main.py --headless --livox-lidar-topic livox/lidar
isaac_sim_python src/main.py --headless --no-enable-livox-lidar
isaac_sim_python src/main.py --headless --unitree-ros2-install-prefix ~/Workspaces/unitree_ros2/cyclonedds_ws/install
```

The active viewport camera follows the robot by default. Disable it when you
want to control the Isaac Sim viewport manually:

```bash
isaac_sim_python src/main.py --no-enable-follow-camera
```

Launch with the default optional world:

```bash
isaac_sim_python src/main.py --use-world true
```

Launch with a specific world USD:

```bash
isaac_sim_python src/main.py --use-world true --world-path /path/to/world.usd
```

If `--use-world` is omitted or set to `false`, the simulator keeps the
lightweight default stage with a ground plane.

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
- `/lowstate`
- `/lowcmd`
- `/livox/lidar`

In mixed mode, `/rt/lowstate` is the ROS 2 sidecar topic. The native Unitree
SDK topics are named `rt/lowstate` and `rt/lowcmd` at the DDS layer; ROS 2 CLI
discovery may display them as `/lowstate` and `/lowcmd` because ROS 2 maps DDS
`rt/<name>` topics back to ROS topic names.

Inspect lowstate:

```bash
ros2 topic echo /rt/lowstate --once
```

ROS 2 `/rt/lowcmd` is not the default command source. To intentionally test
ROS 2 lowcmd instead of native lowcmd, launch with native lowcmd disabled and
ROS 2 lowcmd enabled:

```bash
isaac_sim_python src/main.py --headless \
  --no-enable-native-unitree-lowcmd \
  --enable-ros2-lowcmd
```

Then publish a simple lowcmd smoke-test message:

```bash
ros2 topic pub --once /rt/lowcmd unitree_hg/msg/LowCmd "{mode_pr: 0, mode_machine: 0}"
```

Check the external lowstate rate from ROS 2:

```bash
ros2 topic hz /rt/lowstate
```

Inspect the simulated MID360 point cloud:

```bash
ros2 topic info /livox/lidar
ros2 topic echo /livox/lidar --once
```

The simulated MID360 is an Isaac RTX LiDAR approximation of the non-repetitive
Livox scan pattern. It uses `frame_id=mid360_link`, matching the URDF frame
published by `g1_robot`. Keep ROS 2 sourced before launching Isaac Sim so the
Isaac ROS 2 bridge can publish the topic.

Current runtime expectation:

- the configured target is `500 Hz`
- the observed end-to-end ROS 2 rate has been about `467-470 Hz`

## Mixed ROS 2 + Native SDK Smoke Test

Phase 3 validation is automated by:

```bash
./scripts/run_parallel_ros2_native_smoke_test.sh
```

The script launches Isaac Sim in the target mixed mode:

```bash
isaac_sim_python src/main.py \
  --headless \
  --enable-dds \
  --enable-ros2-lowstate \
  --no-enable-ros2-lowcmd \
  --enable-native-unitree-lowstate \
  --enable-native-unitree-lowcmd \
  --dds-domain-id 1 \
  --native-unitree-domain-id 1
```

It then checks:

- ROS 2 `/rt/lowstate` receives valid samples on `ROS_DOMAIN_ID=1`
- native Unitree SDK `rt/lowstate` is visible and CRC-valid
- native Unitree SDK `rt/lowcmd` publishes a conservative offset command
- ROS 2 lowcmd stays disabled in the target command-authority mode

Useful overrides:

```bash
DDS_DOMAIN_ID=1 \
NATIVE_DOMAIN_ID=1 \
NATIVE_LOWCMD_JOINT_INDEX=15 \
NATIVE_LOWCMD_JOINT_NAME=left_shoulder_pitch_joint \
NATIVE_LOWCMD_OFFSET_RAD=0.05 \
./scripts/run_parallel_ros2_native_smoke_test.sh
```

Logs are written to `tmp/parallel_ros2_native_smoke_logs` by default. Keep the
real robot on domain `0`; use domain `1` for simulator ROS 2 and native SDK
validation unless you intentionally choose another isolated domain.

The simulator-side cadence log is useful when you want to compare:

- the intended lowstate cadence from the physics loop
- the realized wall-clock cadence inside the Isaac Sim process
- the external observer reported by `ros2 topic hz`

## Configuration

Runtime defaults are defined in `src/config.py` and exposed as CLI arguments
to `src/main.py`.

The full configuration reference, including world-loading arguments, is in
[config.md](config.md).

## Documentation

- Installation Instructions: [Installation](docs/installation.md)
- configuration reference: [config.md](config.md)
- ROS 2 sidecar architecture: [context_ros2_runtime.md](docs/context_ros2_runtime.md)
- native Unitree SDK bridge architecture: [context_native_bridge.md](docs/context_native_bridge.md)
- known issues and setup notes: [docs/issues.md](docs/issues.md)
- active implementation notes: [Agents/dev_log.md](Agents/dev_log.md)
- implementation plan: [Agents/implementation_plan_ros2.md](Agents/implementation_plan_ros2.md)
- native SDK implementation plan: [Agents/implementation_plan_native_unitree_sdk.md](Agents/implementation_plan_native_unitree_sdk.md)
