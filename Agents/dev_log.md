# Dev Log

Historical entries from the earlier implementation phase are preserved in
[Agents/old/dev_log.md](old/dev_log.md).

## 2026-04-08 20:05:00 MST

- Re-read [Agents/implementation_plan_ros2.md](implementation_plan_ros2.md)
  and kept work aligned with `Pre-Milestone 1`.
- Confirmed the ROS 2 sidecar path now exposes:
  - `/lowstate`
  - `/lowcmd`
- Continued hardening the sidecar bridge lifecycle after repeated local
  restart failures.

Bridge/runtime updates completed:

- changed the default localhost UDP bridge ports from `5501/5502` to
  `35501/35502`
- added explicit socket cleanup for both:
  - `src/dds/g1_lowstate.py`
  - `src/dds/g1_lowcmd.py`
- updated the DDS manager to clean up partial initialization state on
  startup failure
- updated the lowcmd bridge path to:
  - bind before sidecar startup
  - expose the actual bound port
  - pass that bound port into the sidecar launch command
- added a fallback path for the Isaac-side lowcmd UDP socket to use an
  ephemeral localhost port if the configured port is still busy
- added stale sidecar cleanup in `src/dds/manager.py`:
  - scans for prior `scripts/ros2_cyclonedds_sidecar.py` processes
  - terminates them before launching a new sidecar

Documentation update:

- rewrote `README.md` into a user-facing document
- removed branch-progress tracking from the README
- kept implementation status and branch details here in `Agents/dev_log.md`

Validation completed:

- `python3 -m unittest tests/test_config.py tests/test_dds_lowcmd.py tests/test_dds_manager.py`
- `python3 -m py_compile src/config.py src/dds/g1_lowcmd.py src/dds/g1_lowstate.py src/dds/manager.py scripts/ros2_cyclonedds_sidecar.py`

Status against `implementation_plan_ros2.md`:

- `Pre-Milestone 1` remains in progress
- the active runtime path is still free of `unitree_sdk2py`
- the current work materially advances the transport replacement and
  runtime robustness needed before Milestone 1 validation

Next technical checks:

- verify long-running `isaac_sim_python src/main.py --headless` startup is
  stable across repeated restarts
- verify live `/lowstate` streaming during runtime, not just topic
  visibility
- verify `/lowcmd` ingress drives bounded simulator motion
- then begin Milestone 1 items:
  - explicit outward quaternion `wxyz` validation
  - real 500 Hz `lowstate` behavior

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
