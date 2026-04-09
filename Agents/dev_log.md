# Dev Log

## 2026-04-08 19:50:02 MST

- Branch context:
  - active branch is `transition/unitree_ros2`
- Re-read `Agents/implementation_plan_ros2.md`.
- Started implementing `Pre-Milestone 1: Remove unitree_sdk2py Runtime Dependence`.
- Removed the direct `unitree_sdk2py` runtime dependence from the active
  simulator DDS path.
- Replaced the original in-process DDS transport plan with a sidecar-based
  ROS 2 / CycloneDDS bridge architecture because Isaac Sim is running on
  Python 3.11 while the available ROS 2 Humble Python stack is built for
  Python 3.10.

Current transport shape:

- Isaac Sim runtime:
  - produces lowstate packets
  - receives lowcmd packets
  - stays responsible for mapping, state extraction, and command
    application
- ROS 2 sidecar:
  - runs under system Python
  - publishes `/lowstate`
  - subscribes `/lowcmd`
  - uses the `unitree_hg` message path from the built `unitree_ros2`
    workspace
- localhost UDP between Isaac Sim and the sidecar:
  - lowstate: Isaac Sim -> sidecar
  - lowcmd: sidecar -> Isaac Sim

Files added:

- `src/dds/bridge_protocol.py`
- `scripts/ros2_cyclonedds_sidecar.py`

Files substantially updated:

- `src/dds/g1_lowstate.py`
- `src/dds/g1_lowcmd.py`
- `src/dds/manager.py`
- `src/config.py`
- `README.md`
- `Agents/full_validation.md`
- `scripts/run_full_validation.sh`

Tests updated:

- removed the old SDK-oriented `tests/test_g1_lowcmd.py`
- added `tests/test_dds_lowcmd.py`
- updated config and DDS-manager tests for the sidecar bridge settings

Validation completed so far:

- repo-side unit tests for the new sidecar packet path pass
- Isaac Sim now starts the sidecar bridge successfully
- ROS 2 now exposes:
  - `/lowstate`
  - `/lowcmd`
- this confirms the current external path is visible through ROS 2

Current blocker / active issue:

- the ROS 2 sidecar still shuts down after a runtime publish-context error:
  - `Failed to publish: publisher's context is invalid`
- this looks like a sidecar lifecycle / shutdown handling issue rather
  than a transport-bootstrap failure, because the topics are already
  visible in ROS 2 before shutdown

Status against `implementation_plan_ros2.md`:

- `Pre-Milestone 1` is in progress and materially advanced:
  - `unitree_sdk2py` runtime dependence removed from the active path
  - message / transport handling moved away from the SDK path
  - manual CRC is no longer used in the active runtime bridge
- `Milestone 1` has not started yet:
  - quaternion output still needs the explicit `wxyz` check/update
  - real 500 Hz behavior still needs implementation/validation
  - bounded command-motion validation against `unitree_ros2` still
    remains

Resume here:

- keep the sidecar alive cleanly during runtime and shutdown
- verify `ros2 topic echo /lowstate`
- verify lowcmd ingress from ROS 2 into Isaac Sim
- then move into Milestone 1 compatibility work:
  - `wxyz`
  - 500 Hz
