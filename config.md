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

## Notes

- `--asset-path` and `--world-path` are resolved with `Path.expanduser()` and
  `Path.resolve()`.
- Robot asset validation always runs before Isaac Sim startup. World asset
  validation runs only when `--use-world` is enabled.
- When `--use-world` is disabled, `src/scene.py` creates the default ground
  plane. When enabled, the world USD is referenced at `--world-prim-path`.
- `--lowstate-publish-hz` must not exceed `1 / --physics-dt`.
