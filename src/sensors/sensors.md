# Sensors

## Livox Lidar MID 360

The simulated Livox MID360 is mounted at the same `torso_link -> mid360_link` transform used by the G1 URDF and publishes `sensor_msgs/msg/PointCloud2` directly from Isaac Sim.

This sensor model is an approximation. It is intended to provide useful simulated point clouds for bringup, visualization, and navigation development,
but it does not exactly reproduce the real Livox MID360 scan pattern, timing, intensity model, return behavior, or packet-level data format.

### Runtime Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `--enable-livox-lidar` / `--no-enable-livox-lidar` | `true` | Enables or disables creation of the simulated MID360 lidar. |
| `--livox-lidar-topic` | `livox/lidar` | ROS 2 `PointCloud2` topic for the simulated lidar. |
| `--livox-lidar-frame-id` | `mid360_link` | Frame id used in published `PointCloud2` messages. |
| `--livox-lidar-parent-link-name` | `torso_link` | USD prim name searched under the robot prim when mounting the MID360 frame. |
| `--livox-lidar-prim-name` | `mid360_link` | USD Xform name created under the parent link for the simulated MID360 frame. |
| `--livox-lidar-sensor-prim-name` | `mid360_rtx_lidar` | USD prim name for the RTX lidar sensor under the MID360 frame. |
| `--livox-lidar-rtx-config` | `OS1` | Base Isaac RTX lidar config used for the MID360 approximation. Use `none` to create a generic OmniLidar with explicit attributes only. |
| `--livox-lidar-rtx-variant` | `OS1_REV6_32ch20hz1024res` | RTX lidar config variant passed with `--livox-lidar-rtx-config`. Use `none` to pass no variant. |

### Model Parameters

| Parameter | Value | Description |
| --- | --- | --- |
| Mount translation in `torso_link` | `(0.0002835, 0.00003, 0.40618)` m | Translation copied from the G1 URDF `mid360_joint`. |
| Mount RPY in `torso_link` | `(0.0, 0.04014257279586953, 0.0)` rad | Rotation copied from the G1 URDF `mid360_joint`. |
| Channels | `40` | Number of simulated vertical channels. |
| Frame rate | `10.0 Hz` | Point cloud publish rate used by the simulated scanner. |
| Point rate | `200000 points/s` | Target attempted ray rate before missed rays and robot self-hit filtering. |
| Points per frame | `20000` | Attempted rays per published cloud. Actual published points may be lower when rays miss geometry. |
| Vertical FOV | `-7.0 to 52.0 deg` | Simulated vertical field of view. |
| Near range | `0.1 m` | Minimum raycast range. |
| Far range | `40.0 m` | Maximum raycast range. |
| Range resolution | `0.004 m` | RTX lidar attribute used by the approximation. |
| Range accuracy | `0.025 m` | RTX lidar attribute used by the approximation. |
| Wavelength | `905.0 nm` | RTX lidar attribute used by the approximation. |
| Max returns | `2` | RTX lidar attribute. The current direct raycast publisher emits the closest non-robot hit. |
| Scan pattern | Deterministic 40-channel 360-degree approximation | This is not a high-fidelity Livox non-repetitive scan model. |
| ROS timestamp source | Isaac simulation time | Published point clouds use the simulator clock. |
