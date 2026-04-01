# unitree_g1_isaac_sim
A unitree G1 based simulation, to mimic the Cyclone DDS layer, on IsaacSim. 

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
- [ ] Add the DDS bridge package under `src/dds/`
- [ ] Implement `rt/lowstate` publication using Unitree SDK2 `LowState_`
- [ ] Implement `rt/lowcmd` subscription using Unitree SDK2 `LowCmd_`
- [ ] Apply incoming low-level body commands back into simulator joint order
- [ ] Add CRC handling on live DDS publish/subscribe paths
- [ ] Verify publish cadence and runtime behavior against the real G1 DDS contract
- [ ] Add optional hand/gripper DDS bridges if needed for dex1, dex3, and inspire
- [ ] Add reset/simulator-state utility DDS topics if needed
- [ ] Test the external client path with a real Unitree SDK-based DDS client
- [ ] Properly support the `23dof` variant instead of leaving it as unverified
- [ ] Decide on and implement the final deterministic startup/reset pose strategy

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

## Run

This package currently uses a standalone Isaac Sim Python entrypoint.
That means it should be launched with Isaac Sim's `python.sh`, because
`src/main.py` creates `SimulationApp(...)` itself.

Do not use:

```bash
isaac-sim.sh -p main.py
# alias (optional)
isaac_sim -p main.py
```

Use:

```bash
/path/to/isaac-sim/python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py
# alias (optional)
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py
```

Headless example:

```bash
./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless --max-frames 300
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --headless --max-frames 300
```

Load the 23-DoF asset:
**Yet to Test**
```bash
./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 23dof
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --robot-variant 23dof
```

Override the USD directly:

```bash
./python.sh ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --asset-path ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/Models/USD/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd
```


## Supported Args

- `--robot-variant`
- `--asset-path`
- `--robot-prim-path`
- `--robot-height`
- `--physics-dt`
- `--headless`
- `--renderer`
- `--width`
- `--height`
- `--max-frames`
