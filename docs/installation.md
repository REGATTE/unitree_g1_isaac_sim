# Installation

This document is the instruction for installing all the required libraries and dependencies.

**Tested on:**

    ROS2: Humble
    Ubuntu: 22.04 LTS
    Isaac Sim v5.1
    Nvidia Driver: 580.126.xx

## Repo

```bash
mkdir -p ~/Workspaces/ros2_ws/src
cd ~/Workspaces/ros2_ws/src
git clone https://github.com/REGATTE/unitree_g1_isaac_sim.git
cd unitree_g1_isaac_sim
git lfs install
git lfs pull
```

Note: This repo has no ros2 packages to build. A **.COLCON_IGNORE** file has been added, to not build any files from this repo.

## Cyclone DDS

```bash
sudo apt update && sudo apt install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev
```

## Install unitree_ros2

```bash
cd ~/Workspaces
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2/cyclonedds_ws/
colcon build
```

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
```

**Note**: Make sure to source the `cyclonedds_ws` everytime, or place the sourcing in `~/.bashrc`. Same for the the RMW

## Install unitree_sdk2 (this is the native sdk referred in the code-base)

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev
```

Install in the home directory

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
sudo make install
```

## Install Isaac Sim v5.1

Follow this [Isaac Sim Documentation - Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html)