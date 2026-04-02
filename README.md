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

### Install Cyclone DDS

```bash
# Home Folder
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
mkdir -p build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
cd ..

echo 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.bashrc

source ~/.bashrc
```

### Install Unitree_sdk2_python

```bash
# Home FOlder
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
isaac_sim_python -m pip3 install -e .
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

### Arguments

| Arg | Brief | Default | Cmd (`python.sh` and alias) |
| --- | --- | --- | --- |
| `--robot-variant` | Select the default robot asset variant. `29dof` is the current validated path. | `29dof` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 29dof` |
| `--asset-path` | Override the default USD path with a specific asset file. | Uses the default asset path for the selected `--robot-variant`. | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --asset-path ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd` |
| `--robot-prim-path` | Prim path where the robot is referenced into the stage. | `/World/G1` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-prim-path /World/G1` |
| `--robot-height` | Initial root height in meters for the referenced robot prim. | `0.8` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-height 0.8` |
| `--physics-dt` | Isaac Sim physics timestep in seconds. | `1.0 / 120.0` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --physics-dt 0.0083333333` |
| `--headless` | Run without the Isaac Sim GUI window. | `False` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless` |
| `--renderer` | Renderer passed into `SimulationApp`. | `RayTracedLighting` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --renderer RayTracedLighting` |
| `--width` | Isaac Sim window width. | `1280` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --width 1280` |
| `--height` | Isaac Sim window height. | `720` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --height 720` |
| `--max-frames` | Stop after a bounded number of simulation frames. `0` means run until closed. | `0` | `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --max-frames 300` |
| `--print-all-joints` | Print the full articulation joint order during startup validation. | `False` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --print-all-joints` |
| `--enable-dds` | Enable the simulator DDS bridge so external Unitree clients can read simulator state as if it were a real robot. | `False` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds` |
| `--dds-domain-id` | DDS domain id passed into the Unitree SDK channel factory. | `1` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --dds-domain-id 1` |
| `--lowstate-topic` | DDS topic used for low-level state publication. | `rt/lowstate` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-topic rt/lowstate` |
| `--lowcmd-topic` | DDS topic used for low-level command subscription. | `rt/lowcmd` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowcmd-topic rt/lowcmd` |
| `--lowstate-publish-hz` | Target DDS publish rate for `rt/lowstate`. | `100.0` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --lowstate-publish-hz 100` |
| `--enable-lowcmd-subscriber` | Create the `rt/lowcmd` subscriber and apply cached body commands into the articulation. | `False` |  `isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --enable-lowcmd-subscriber` |

Important DDS note:

- The intent of this project is to make Isaac Sim look like a real G1 at the DDS boundary.
- The simulator should publish the same low-level topics a real robot publishes so higher-level clients such as `unitree_sdk2` or `unitree_ros2` can run on top without simulator-specific adaptations.
- The current baseline DDS path on `dds_dev` is:
  - publish `rt/lowstate`
  - subscribe to `rt/lowcmd`
  - apply cached body commands back into the articulation
- The current control slice should still be treated primarily as a conservative position-command path.
- Dynamic `kp` / `kd` application is not implemented yet.
- The current DDS manager runs inside the same Isaac Sim process and main simulation loop.
- This is intentionally simpler than the Isaac Lab reference, which uses a heavier shared-memory and threaded structure between simulation state production and DDS I/O.
- The direct single-process path keeps the current simulator easier to read and debug while DDS compatibility is still being established.
- A threaded/shared-memory DDS decoupling layer can be added later if tighter runtime isolation or more flexible producer/consumer separation becomes necessary.

## Baseline DDS Smoke Test

This is the current baseline external DDS validation flow for the repo.

Open one terminal and launch Isaac Sim with DDS enabled:

```bash
cd ~/path/to/unitree_g1_isaac_sim
isaac_sim_python src/main.py --enable-dds --enable-lowcmd-subscriber
```

Open a second terminal, activate an environment that has `unitree_sdk2py`,
and confirm `rt/lowstate` is visible externally:

```bash
cd ~/path/to/unitree_g1_isaac_sim
python3 scripts/lowstate_listener.py --dds-domain-id 1 --duration 5
```

Expected result:

- `tick` increases
- `crc_rejected=0`
- joint positions, velocities, torques, and IMU values are populated

Then send one conservative body-joint command:

```bash
cd ~/path/to/unitree_g1_isaac_sim
python3 scripts/send_lowcmd_offset.py \
  --dds-domain-id 1 \
  --joint-name left_shoulder_pitch_joint \
  --offset-rad 0.10 \
  --duration 2.0
```

Safer first-motion alternative:

```bash
python3 scripts/send_lowcmd_offset.py \
  --dds-domain-id 1 \
  --joint-name waist_yaw_joint \
  --offset-rad 0.05 \
  --duration 1.5
```

Expected result:

- Isaac Sim reports that `rt/lowcmd` was received
- the targeted joint moves conservatively in the simulator
- the motion is reflected back on the next `rt/lowstate` samples

Notes:

- Start with small offsets only.
- Keep the DDS domain id matched across the simulator and test scripts.
- If `isaac_sim_python` says `unitree_sdk2py` is unavailable, install it into Isaac Sim's Python, not only into your separate local test environment.
- The current `dds_dev` branch now clamps wider incoming `LowCmd_` shapes to the supported 29 body joints instead of crashing on extra slots.
- The next validation is to re-run this smoke test end to end and confirm the conservative commanded motion is reflected back on `rt/lowstate`.

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
- [x] Apply incoming low-level body commands back into simulator joint order
- [x] Add CRC handling on live DDS publish/subscribe paths
- [ ] Verify publish cadence and runtime behavior against the real G1 DDS contract
- [ ] Add an optional threaded/shared-memory DDS decoupling layer if runtime isolation is needed
- [ ] Add optional hand/gripper DDS bridges if needed for dex1, dex3, and inspire
- [ ] Add reset/simulator-state utility DDS topics if needed
- [x] Test the external client path with a standalone Unitree SDK-based DDS client
- [ ] Properly support the `23dof` variant instead of leaving it as unverified
- [ ] Decide on and implement the final deterministic startup/reset pose strategy
