# unitree_g1_isaac_sim
A unitree G1 based simulation, to mimic the Cyclone DDS layer, on IsaacSim. 

## Setup

### Setup Isaac Sim Paths

```bash
nano ~/.bashrc

## Copy the following

# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/isaacsim"
# Isaac Sim python executable
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
# (Optional) Setup alias
alias isaac_sim="cd ${ISAACSIM_PATH} && ./isaac-sim.sh"
alias isaac_sim_python="${ISAACSIM_PYTHON_EXE}"
```

## Usage

This package currently uses a standalone Isaac Sim Python entrypoint.
That means it should be launched with Isaac Sim's `python.sh`, because
`src/main.py` creates `SimulationApp(...)` itself.

Do not use `isaac-sim.sh -p main.py`. Use `python.sh` directly.

```bash
/path/to/isaac-sim/python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py
# or, if you set the alias above
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py
```

### Common Commands

```bash
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless --max-frames 300
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --enable-lowcmd-subscriber
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 23dof
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --asset-path ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd
```

### Arguments

| Arg | Brief | Cmd (`python.sh` and alias) |
| --- | --- | --- |
| `--robot-variant` | Select the default robot asset variant. `29dof` is the current validated path. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 29dof`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 29dof` |
| `--asset-path` | Override the default USD path with a specific asset file. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --asset-path ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --asset-path ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd` |
| `--robot-prim-path` | Prim path where the robot is referenced into the stage. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-prim-path /World/G1`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-prim-path /World/G1` |
| `--robot-height` | Initial root height in meters for the referenced robot prim. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-height 0.8`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-height 0.8` |
| `--physics-dt` | Isaac Sim physics timestep in seconds. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --physics-dt 0.0083333333`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --physics-dt 0.0083333333` |
| `--headless` | Run without the Isaac Sim GUI window. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless` |
| `--renderer` | Renderer passed into `SimulationApp`. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --renderer RayTracedLighting`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --renderer RayTracedLighting` |
| `--width` | Isaac Sim window width. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --width 1280`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --width 1280` |
| `--height` | Isaac Sim window height. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --height 720`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --height 720` |
| `--max-frames` | Stop after a bounded number of simulation frames. `0` means run until closed. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --max-frames 300`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --max-frames 300` |
| `--print-all-joints` | Print the full articulation joint order during startup validation. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --print-all-joints`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --print-all-joints` |
| `--enable-dds` | Enable the simulator DDS bridge so external Unitree clients can read simulator state as if it were a real robot. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds` |
| `--dds-domain-id` | DDS domain id passed into the Unitree SDK channel factory. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --dds-domain-id 1`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --dds-domain-id 1` |
| `--lowstate-topic` | DDS topic used for low-level state publication. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-topic rt/lowstate`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-topic rt/lowstate` |
| `--lowcmd-topic` | DDS topic used for low-level command subscription. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowcmd-topic rt/lowcmd`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowcmd-topic rt/lowcmd` |
| `--lowstate-publish-hz` | Target DDS publish rate for `rt/lowstate`. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-publish-hz 100`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-publish-hz 100` |
| `--enable-lowcmd-subscriber` | Create the `rt/lowcmd` subscriber skeleton. Command application into Isaac Sim is still a follow-up step. | `./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --enable-lowcmd-subscriber`<br>`isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --enable-lowcmd-subscriber` |

Important DDS note:

- The intent of this project is to make Isaac Sim look like a real G1 at the DDS boundary.
- The simulator should publish the same low-level topics a real robot publishes so higher-level clients such as `unitree_sdk2` or `unitree_ros2` can run on top without simulator-specific adaptations.
- The current implementation slice focuses on `rt/lowstate` publication first.
- The `rt/lowcmd` subscriber object now exists, but applying incoming commands back into Isaac Sim remains a follow-up step.

## TODO

- [x] Standalone Isaac Sim launcher in `src/main.py`
- [x] Stage/world setup for `/World`, physics scene, ground plane, light, and robot reference
- [x] Articulation discovery for the G1 robot under `/World/G1`
- [x] Live joint-state reads for positions, velocities, and efforts
- [x] Frozen 29-DoF simulator joint order
- [x] Frozen 29-DoF DDS body joint order
- [x] Validated simulator-order to DDS-order joint mapping
- [x] Startup validation that the live articulation still matches the frozen simulator order
- [x] DDS-ordered joint snapshot preview from the live simulator snapshot
- [x] Expanded robot-state reader for base position, base quaternion in validated Isaac Sim `wxyz` convention, base linear/angular velocity, and IMU-like body-frame signals
- [x] Simulation-time based IMU acceleration estimation
- [x] Add the DDS bridge package under `src/dds/`
- [x] Implement `rt/lowstate` publication using Unitree SDK2 `LowState_`
- [x] Implement `rt/lowcmd` subscription using Unitree SDK2 `LowCmd_`
- [ ] Apply incoming low-level body commands back into simulator joint order
- [x] Add CRC handling on live DDS publish/subscribe paths
- [ ] Verify publish cadence and runtime behavior against the real G1 DDS contract
- [ ] Add optional hand/gripper DDS bridges if needed for dex1, dex3, and inspire
- [ ] Add reset/simulator-state utility DDS topics if needed
- [ ] Test the external client path with a real Unitree SDK-based DDS client
- [ ] Properly support the `23dof` variant instead of leaving it as unverified
- [ ] Decide on and implement the final deterministic startup/reset pose strategy
