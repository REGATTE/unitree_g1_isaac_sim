# Isaac Sim G1 `unitree_ros2` Compatibility Plan

## Goal

Make the simulator on branch `feat/ros2` interoperate with
`unitree_ros2` for the G1 low-level path without relying on
`unitree_sdk2py` as the primary client integration layer.

Initial scope:

- `unitree_ros2` G1 low-level examples can subscribe to simulator
  `lowstate`
- `unitree_ros2` G1 low-level examples can publish `/lowcmd`
- the simulator responds as expected through the Cyclone DDS / ROS 2
  boundary

Out of scope for the first pass:

- `lf/...` topics
- high-level sport mode APIs
- dex hand topics
- full ROS 2 packaging of the simulator as a native node

## Compatibility Summary

The current simulator is already close to the `unitree_ros2` G1
low-level contract because it uses the same HG low-level DDS family.

The first known compatibility risks are:

- effective publish cadence versus the 500 Hz expectation in the G1
  low-level ROS 2 examples
- final verification that ROS 2 topic naming resolves cleanly to the
  existing `rt/lowstate` and `rt/lowcmd` traffic

Current note:

- the active ROS 2 sidecar path now emits outgoing IMU quaternion in
  `wxyz`

## Pre-Milestone 1: Remove `unitree_sdk2py` Runtime Dependence

Purpose:

- stop relying on `unitree_sdk2py` for the simulator's low-level DDS /
  ROS 2 boundary before the first ROS 2 compatibility milestone

This needs to happen before Milestone 1 because the long-term branch
direction is to validate the simulator against `unitree_ros2`, not to
keep the current `unitree_sdk2py` transport and only test ROS 2 around
it.

Required replacements:

- replace `unitree_sdk2py`-based message construction and parsing
- replace `unitree_sdk2py`-based publisher / subscriber transport
- replace `unitree_sdk2py` CRC handling if the ROS 2 path does not
  require manual CRC management

Constraints:

- keep the existing simulator-to-DDS joint-order mapping layer
- do not mix old and new message semantics implicitly
- keep simulator internal state conventions unchanged unless a verified
  ROS 2 boundary requirement demands otherwise

Deliverables:

- a ROS 2-facing lowstate publish path that does not import
  `unitree_sdk2py`
- a ROS 2-facing lowcmd subscribe path that does not import
  `unitree_sdk2py`
- a clear note on whether manual CRC handling still exists in the ROS 2
  path after the replacement

## Milestone 1: Initial ROS 2 Compatibility Check

Purpose:

- establish the minimum conditions required for a first `unitree_ros2`
  integration check

Required updates:

- make the simulator capable of a real 500 Hz `lowstate` stream for the
  high-frequency G1 low-level path
- keep using the high-frequency topics only:
  - `lowstate`
  - `/lowcmd`
- do not add `lf/...` topics in this milestone

Checks for this milestone:

1. Confirm the runtime stepping / publish scheduling supports an actual
   500 Hz state stream instead of only a configured target above the
   physics rate.
2. Confirm a `unitree_ros2` G1 low-level example can:
   - receive `lowstate`
   - publish `/lowcmd`
   - drive a bounded simulator motion

Deliverables:

- updated runtime configuration and validation for 500 Hz operation
- a documented manual validation flow against `unitree_ros2`

Constraints:

- preserve the existing validated joint-order conversion boundary
- preserve simulator internal quaternion convention as `wxyz`
- preserve simulation-time-derived IMU acceleration logic

## Milestone 2: Validation and Documentation

Purpose:

- make the ROS 2 path repeatable for local development

Deliverables:

- validation steps for sourcing `unitree_ros2` with Cyclone DDS
- example commands for:
  - simulator launch
  - `unitree_ros2` low-level subscriber / publisher tests
- README updates describing ROS 2 expectations, including topic and
  cadence assumptions
