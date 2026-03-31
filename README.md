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
