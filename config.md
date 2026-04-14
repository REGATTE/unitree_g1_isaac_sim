# Configuration

Runtime configuration lives in [`src/config.py`](src/config.py). The simulator
does not read a separate config file; defaults are Python constants and
`AppConfig` fields, and launch-time changes are passed as arguments to
`src/main.py`.

Use Isaac Sim's Python entrypoint:

```bash
isaac_sim_python src/main.py [args]
```

Show the current CLI help:

```bash
isaac_sim_python src/main.py --help
```

## World Loading

The default launch keeps the lightweight built-in stage with a ground plane.
To load the configured world USD, enable `--use-world`.

```bash
isaac_sim_python src/main.py --use-world true
```

Use another world USD:

```bash
isaac_sim_python src/main.py --use-world true --world-path /path/to/world.usd
```

`--use-world` also works without an explicit value and is treated as `true`:

```bash
isaac_sim_python src/main.py --use-world
```

## Config Table

| Argument | Default | Description | Example |
| --- | --- | --- | --- |
| `--robot-variant` | `29dof` | Selects the default robot USD variant. Choices are `23dof` and `29dof`. | `isaac_sim_python src/main.py --robot-variant 29dof` |
| `--asset-path` | variant default | Overrides the selected robot USD path. | `isaac_sim_python src/main.py --asset-path Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd` |
| `--robot-prim-path` | `/World/G1` | Prim path where the robot USD is referenced. | `isaac_sim_python src/main.py --robot-prim-path /World/G1` |
| `--robot-height` | `0.8` | Initial robot root height in meters. | `isaac_sim_python src/main.py --robot-height 0.8` |
| `--use-world` | `false` | Enables optional world USD loading. Accepts `true` or `false`; no value means `true`. | `isaac_sim_python src/main.py --use-world true` |
| `--world-path` | `Models/world/Hospital_World.usd` | World USD path used only when `--use-world` is enabled. | `isaac_sim_python src/main.py --use-world true --world-path Models/world/Hospital_World.usd` |
| `--world-prim-path` | `/World/Environment` | Prim path where the optional world USD is referenced. | `isaac_sim_python src/main.py --use-world true --world-prim-path /World/Environment` |
| `--enable-follow-camera` / `--no-enable-follow-camera` | `true` | Enables or disables the active viewport camera that follows the robot base. | `isaac_sim_python src/main.py --no-enable-follow-camera` |
| `--follow-camera-prim-path` | `/World/FollowCamera` | Prim path for the camera that follows the robot. | `isaac_sim_python src/main.py --follow-camera-prim-path /World/FollowCamera` |
| `--follow-camera-distance` | `4.0` | Distance in meters behind the robot for the follow camera. | `isaac_sim_python src/main.py --follow-camera-distance 5.0` |
| `--follow-camera-height` | `0.6` | Height offset in meters above the robot base for the follow camera. | `isaac_sim_python src/main.py --follow-camera-height 1.8` |
| `--follow-camera-target-height` | `0.3` | Height offset in meters above the robot base that the follow camera looks at. | `isaac_sim_python src/main.py --follow-camera-target-height 1.0` |
| `--physics-dt` | `0.002` | Physics timestep in seconds. Must be positive and finite. | `isaac_sim_python src/main.py --physics-dt 0.002` |
| `--headless` | `false` | Runs Isaac Sim without the GUI window. | `isaac_sim_python src/main.py --headless` |
| `--renderer` | `RayTracedLighting` | Renderer passed into `SimulationApp`. | `isaac_sim_python src/main.py --renderer RayTracedLighting` |
| `--width` | `1280` | Isaac Sim window width. | `isaac_sim_python src/main.py --width 1280` |
| `--height` | `720` | Isaac Sim window height. | `isaac_sim_python src/main.py --height 720` |
| `--max-frames` | `0` | Stops after N Kit frames. `0` runs until the app closes. | `isaac_sim_python src/main.py --max-frames 300` |
| `--reset-after-frames` | `0` | Triggers one deterministic runtime reset after N simulation frames. `0` disables it. | `isaac_sim_python src/main.py --reset-after-frames 500` |
| `--print-all-joints` | `false` | Prints the full articulation joint list during startup validation. | `isaac_sim_python src/main.py --print-all-joints` |
| `--enable-dds` / `--no-enable-dds` | `true` | Enables or disables the ROS 2 / CycloneDDS sidecar bridge. | `isaac_sim_python src/main.py --no-enable-dds` |
| `--dds-domain-id` | `1` | DDS domain id used by the sidecar bridge. | `isaac_sim_python src/main.py --dds-domain-id 1` |
| `--lowstate-topic` | `rt/lowstate` | DDS topic used for low-level robot state publication. | `isaac_sim_python src/main.py --lowstate-topic rt/lowstate` |
| `--lowcmd-topic` | `rt/lowcmd` | DDS topic used for low-level robot command subscription. | `isaac_sim_python src/main.py --lowcmd-topic rt/lowcmd` |
| `--lowstate-publish-hz` | `500.0` | Target publish rate for `rt/lowstate`. Cannot exceed the physics rate from `--physics-dt`. | `isaac_sim_python src/main.py --lowstate-publish-hz 500` |
| `--lowcmd-max-position-delta-rad` | `0.25` | Rejects lowcmd position targets farther than this absolute joint-position delta from the current sim pose. `0` disables bounded-motion rejection. | `isaac_sim_python src/main.py --lowcmd-max-position-delta-rad 0.25` |
| `--enable-lowcmd-subscriber` / `--no-enable-lowcmd-subscriber` | `true` | Creates or disables the `rt/lowcmd` subscriber and command application path. | `isaac_sim_python src/main.py --no-enable-lowcmd-subscriber` |
| `--lowcmd-timeout-seconds` | `0.5` | Time a cached `rt/lowcmd` sample stays fresh. `0` disables timeout handling. | `isaac_sim_python src/main.py --lowcmd-timeout-seconds 0.5` |
| `--lowstate-cadence-report-interval` | `500` | Number of lowstate publishes to accumulate before reporting observed cadence. `0` disables cadence reports. | `isaac_sim_python src/main.py --lowstate-cadence-report-interval 500` |
| `--lowstate-cadence-warn-ratio` | `0.05` | Relative cadence tolerance before cadence diagnostics are logged as warnings. | `isaac_sim_python src/main.py --lowstate-cadence-warn-ratio 0.05` |
| `--unitree-ros2-install-prefix` | auto-detected | Path to the built `unitree_ros2/cyclonedds_ws/install` prefix. If omitted, `UNITREE_ROS2_INSTALL_PREFIX` and common workspace paths are checked. | `isaac_sim_python src/main.py --unitree-ros2-install-prefix ~/Workspaces/unitree_ros2/cyclonedds_ws/install` |
| `--ros2-python-exe` | `$ROS2_PYTHON_EXE` or `/usr/bin/python3` | Python executable used for the ROS 2 sidecar process. | `isaac_sim_python src/main.py --ros2-python-exe /usr/bin/python3` |
| `--bridge-bind-host` | `127.0.0.1` | Local interface used for Isaac Sim to sidecar UDP traffic. | `isaac_sim_python src/main.py --bridge-bind-host 127.0.0.1` |
| `--bridge-lowstate-port` | `35501` | UDP port used for Isaac Sim to sidecar lowstate packets. | `isaac_sim_python src/main.py --bridge-lowstate-port 35501` |
| `--bridge-lowcmd-port` | `35502` | UDP port used for sidecar to Isaac Sim lowcmd packets. | `isaac_sim_python src/main.py --bridge-lowcmd-port 35502` |
| `--enable-livox-lidar` / `--no-enable-livox-lidar` | `true` | Creates or disables the Isaac-native Livox MID360 RTX LiDAR approximation. This publishes through Isaac's ROS 2 bridge, not the LowState / LowCmd sidecar. | `isaac_sim_python src/main.py --no-enable-livox-lidar` |
| `--livox-lidar-topic` | `livox/lidar` | ROS 2 `sensor_msgs/msg/PointCloud2` topic for the simulated MID360. | `isaac_sim_python src/main.py --livox-lidar-topic livox/lidar` |
| `--livox-lidar-frame-id` | `mid360_link` | Frame id used in the simulated MID360 point cloud messages. | `isaac_sim_python src/main.py --livox-lidar-frame-id mid360_link` |
| `--livox-lidar-parent-link-name` | `torso_link` | USD prim name searched under the robot prim when mounting the MID360 frame. Mirrors the URDF fixed joint parent. | `isaac_sim_python src/main.py --livox-lidar-parent-link-name torso_link` |
| `--livox-lidar-prim-name` | `mid360_link` | USD Xform created under the parent link for the simulated MID360 frame. Mirrors the URDF fixed joint child. | `isaac_sim_python src/main.py --livox-lidar-prim-name mid360_link` |
| `--livox-lidar-sensor-prim-name` | `mid360_rtx_lidar` | USD prim name for the actual RTX LiDAR sensor under the MID360 frame. | `isaac_sim_python src/main.py --livox-lidar-sensor-prim-name mid360_rtx_lidar` |
| `--livox-lidar-rtx-config` | `OS1` | Base Isaac RTX LiDAR config used for the MID360 approximation. Use `none` to create a generic OmniLidar using only explicit attributes. | `isaac_sim_python src/main.py --livox-lidar-rtx-config OS1` |
| `--livox-lidar-rtx-variant` | `OS1_REV6_32ch20hz1024res` | RTX LiDAR config variant used with `--livox-lidar-rtx-config`. Use `none` to pass no variant. | `isaac_sim_python src/main.py --livox-lidar-rtx-variant none` |

## Notes

- `--asset-path` and `--world-path` are resolved with `Path.expanduser()` and
  `Path.resolve()`.
- Robot asset validation always runs before Isaac Sim startup. World asset
  validation runs only when `--use-world` is enabled.
- When `--use-world` is disabled, `src/scene.py` creates the default ground
  plane. When enabled, the world USD is referenced at `--world-prim-path`.
- The follow camera uses the robot base pose from the same kinematic snapshot
  path used by DDS state publication, so it still follows when DDS is disabled.
- `--lowstate-publish-hz` must not exceed `1 / --physics-dt`.
- The simulated MID360 is mounted at the same `torso_link -> mid360_link`
  transform as the URDF and publishes point clouds with `frame_id=mid360_link`.
  Let the URDF / `robot_state_publisher` own TF for this frame.
- The simulated MID360 uses Isaac's RTX LiDAR ROS 2 writer directly. Source
  ROS 2 before launching Isaac Sim and keep `ROS_DOMAIN_ID` aligned with RViz
  and navigation nodes.
