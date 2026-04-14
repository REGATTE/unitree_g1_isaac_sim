# Installation

This document lists the host dependencies needed to run the Unitree G1 Isaac
Sim environment with the ROS 2 sidecar bridge and the native Unitree SDK bridge.

Tested environment:

- ROS 2 Humble
- Ubuntu 22.04 LTS
- Isaac Sim 5.1
- NVIDIA driver 580.126.xx

## Clone This Repository

```bash
mkdir -p ~/Workspaces/ros2_ws/src
cd ~/Workspaces/ros2_ws/src
git clone https://github.com/REGATTE/unitree_g1_isaac_sim.git
cd unitree_g1_isaac_sim
git lfs install
git lfs pull
```

This repository is not a ROS 2 package and does not need to be built with
`colcon`. The repo includes `.COLCON_IGNORE` so it is skipped when the parent
ROS 2 workspace is built.

## Cyclone DDS

```bash
sudo apt update && sudo apt install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev
```

```bash
# Add this to the ~/.bashrc
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Install `unitree_ros2`

`unitree_ros2` provides the `unitree_hg` ROS 2 message definitions used by the
ROS 2 sidecar bridge.

```bash
cd ~/Workspaces
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2/cyclonedds_ws/
colcon build
```

Source the workspace and set the ROS 2 middleware/domain for simulator testing:

```bash
source ~/Workspaces/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
```

Add those exports to `~/.bashrc` if you want them applied automatically in new
terminals. Keep the real robot on domain `0`; use domain `1` for simulator
testing unless you intentionally choose another isolated domain.

## Install `unitree_sdk2`

`unitree_sdk2` is the native SDK used by `native_sdk_bridge/`.

```bash
sudo apt-get update
sudo apt-get install -y \
  cmake \
  g++ \
  build-essential \
  libyaml-cpp-dev \
  libeigen3-dev \
  libboost-all-dev \
  libspdlog-dev \
  libfmt-dev
```

Install it under your home directory:

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
sudo make install
```

The simulator bridge build expects the source checkout at `~/unitree_sdk2` by
default. If you put it elsewhere, pass `-DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2`
when building `native_sdk_bridge`.

## Install Isaac Sim v5.1

Follow NVIDIA's Isaac Sim workstation installation guide:

[Isaac Sim 5.1 Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html)

### Setup Alias

```bash
# Add this to ~/.bashrc
export ISAACSIM_PATH="${HOME}/Omniverse/isaac_sim"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

alias isaac_sim="cd ${ISAACSIM_PATH} && ./isaac-sim.sh"
alias isaac_sim_python="${ISAACSIM_PYTHON_EXE}"
```

After installation, confirm the simulator launcher is available:

```bash
isaac_sim_python --help
```
