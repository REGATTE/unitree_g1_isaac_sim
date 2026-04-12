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

## Milestone 2: Documentation, Validation, and Tooling Alignment

Purpose:

- make the current ROS 2 sidecar path repeatable, well-documented, and
  internally consistent across docs, helper scripts, and tests
- keep the simulator focused on the interfaces that matter for robot
  simulation and navigation rather than mirroring every real-robot
  hardware topic
- add the locomotion-control layer needed to turn navigation or
  higher-level motion intent into bounded simulator motion

Current scope decision:

- accept the current lowstate cadence as complete for this branch as long
  as the realized external ROS 2 rate stays above `450 Hz`
- keep `LowState` / `LowCmd` as the main `unitree_hg` compatibility
  surface
- treat IMU and lidar as normal ROS 2 sensor publishes, not as extra
  CycloneDDS vendor-topic work
- ignore hardware-oriented topics that are not required for navigation,
  including:
  - `BmsState`
  - `BmsCmd`
  - `MainBoardState`
- treat the remaining `unitree_hg` messages as optional unless a real
  navigation, locomotion, or manipulation workflow proves they are
  needed

Navigation-oriented interface priority:

- keep the low-level robot contract stable:
  - `LowState`
  - `LowCmd`
- add a locomotion controller surface above raw lowcmd so navigation work
  is not forced to command individual joints directly
- support state-estimation work:
  - simulator time alignment
  - IMU availability
  - contact / pressure signals if the estimator needs them
- keep TF and URDF-driven frame publication outside this milestone's DDS
  work

Milestone 2 deliverables:

- README and `Agents/` documentation updated to match the active branch
  behavior and accepted scope
- helper scripts updated to match the current sidecar-era runtime and
  current topic / log expectations
- test coverage updated so the active runtime and helper workflows are
  checked against the latest implementation rather than stale SDK-era
  assumptions
- a first locomotion-controller path that can consume bounded
  high-level motion commands and drive the simulator in a controlled way
- a documented validation flow for:
  - simulator launch
  - ROS 2 lowstate observation
  - bounded lowcmd ingress
  - bounded locomotion-controller motion
  - current accepted cadence expectations

Expected work items:

1. Documentation alignment
   - update planning and README language so it reflects the current
     branch decisions:
     - `>450 Hz` is accepted
     - the active path is the ROS 2 sidecar bridge
     - hardware-only message families are intentionally out of scope
2. Script refresh
   - update repository helper scripts to match current runtime log
     markers, topic names, and sidecar startup behavior
   - remove stale assumptions left over from the pre-sidecar or SDK-era
     path
3. Test refresh
   - update unit tests to cover the current runtime behavior
   - add or revise tests when scripts, startup expectations, or bridge
     semantics change
4. Locomotion controller
   - choose the control surface that the simulator should expose for
     navigation-driven motion
   - keep the interface above raw joint-level commands unless a concrete
     workflow requires direct lowcmd control
   - ensure commanded motion stays bounded and compatible with the
     existing low-level safety checks
   - document how the controller relates to `LowState`, `LowCmd`, state
     estimation, and ROS-native sensor topics
5. Validation flow cleanup
   - keep smoke-test and manual validation flows current with the active
     branch
   - defer larger end-to-end validation harness cleanup until the
     remaining `unitree_ros2` development settles
6. Scope documentation
   - document which interfaces are:
     - implemented and validated
     - intentionally deferred
     - intentionally ignored because they are hardware/platform-specific

Out of scope for this milestone unless a concrete workflow requires them:

- broad expansion to all `unitree_hg` messages
- `unitree_go` message families
- `unitree_api` request / response layers
- battery, board, or other hardware-maintenance telemetry that does not
  materially support navigation or robot simulation goals
