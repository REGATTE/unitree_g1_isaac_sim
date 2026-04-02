# Isaac Sim G1 Cyclone DDS Simulation Implementation Plan

## Goal

Build an Isaac Sim based Unitree G1 simulator that behaves like the real robot at the DDS interface level.

The simulator should:

- run the G1 robot inside Isaac Sim
- publish robot state on the same Cyclone DDS topics used by the real robot
- subscribe to the same command topics used by the real robot
- translate DDS messages to Isaac Sim control inputs
- translate Isaac Sim state to DDS messages
- allow an external client built on the Unitree SDK to interact with the simulator as if it were a real robot

This plan is simulation oriented, not navigation oriented. Navigation, teleoperation, manipulation, or autonomy stacks are downstream users of the same simulated robot interface.

## Reference Principle

The Isaac Lab package under `src/unitree_sim_isaaclab` is useful as a reference for the DDS compatibility layer, not as a required runtime architecture.

What should be reused conceptually:

- topic names
- message types / IDL usage
- joint ordering conventions
- range and unit conversions
- low-level CRC handling
- separation between state publication and command ingestion

What does not need to be copied directly:

- Isaac Lab observation-manager wiring
- Isaac Lab task structure
- Isaac Lab specific environment configuration
- shared-memory based decoupling, unless it proves useful

## Core Requirement

The simulator must reproduce the external robot contract.

The main success criterion is:

- a Unitree SDK based client can connect to the Isaac Sim instance over Cyclone DDS
- it can read state and send commands using the same code paths used for the real robot
- no simulator-specific client adaptation is required beyond network or channel configuration

## Target Architecture

The recommended first architecture is a single-process Isaac Sim Python application.

Main components:

1. Isaac Sim application bootstrap
2. G1 robot loader and scene setup
3. Robot state reader
4. Robot command applier
5. DDS bridge manager
6. Optional utilities for reset, diagnostics, and replay

High-level flow:

1. Start Isaac Sim app.
2. Load G1 robot USD or URDF-derived asset.
3. Create the DDS publishers and subscribers.
4. On each simulation tick:
   - read latest DDS commands
   - convert them into simulator targets
   - advance physics
   - read robot state from Isaac Sim
   - publish DDS state messages at configured rates

## Recommended Module Layout

Planned code structure under `src/unitree_g1_isaac_sim/src`:

- `main.py`
  Isaac Sim entrypoint, launched with Isaac Sim's Python runtime:
  `isaac_sim_python src/main.py`
- `scene.py`
  stage setup, world creation, robot spawning, reset helpers
- `robot_state.py`
  articulation state extraction and simulator-to-DDS state packaging inputs
- `robot_control.py`
  command cache ownership and DDS-to-simulator command application
- `config.py`
  runtime configuration, topic toggles, rates, asset paths, robot variant selection
- `dds/__init__.py`
  package root for DDS bridge modules
- `dds/manager.py`
  DDS initialization and bridge object lifecycle
- `dds/g1_lowstate.py`
  `rt/lowstate` publisher
- `dds/g1_lowcmd.py`
  `rt/lowcmd` subscriber
- `dds/dex1.py`
  optional dex1 bridge
- `dds/dex3.py`
  optional dex3 bridge
- `dds/inspire.py`
  optional inspire bridge
- `dds/reset_pose.py`
  optional reset command bridge
- `dds/sim_state.py`
  optional simulator state bridge
- `mapping/joints.py`
  joint ordering and index mapping tables
- `mapping/conversion.py`
  unit and range conversions

Existing package assets should stay outside the runtime source tree:

- `src/unitree_g1_isaac_sim/Models/`
  robot USD assets

Validation and DDS helper tooling should stay outside the runtime source tree:

- `scripts/`
  standalone listeners, publishers, and analysis helpers launched with the
  system Python runtime, for example:
  `python scripts/lowstate_listener.py`

## Entry Point Responsibility

`main.py` should stay thin and act as the Isaac Sim orchestration layer only.

Its responsibilities should be:

- start Isaac Sim through the packaged runtime
- resolve project and asset paths
- load configuration
- create the scene and robot
- initialize DDS bridge objects
- own the main simulation loop
- coordinate shutdown

It should not contain:

- large joint mapping tables
- message conversion details
- DDS message class logic
- hand-specific translation logic

Those belong in `mapping/`, `dds/`, `robot_state.py`, and `robot_control.py`.

## Implementation Phases

## Phase 1: Define the External DDS Contract

Before writing simulator code, freeze the exact DDS contract to emulate.

Deliverables:

- list of required DDS topics
- message types used on each topic
- publish rates and expected timing
- required CRC handling
- required command and state fields
- joint ordering for each message type

Minimum baseline from the Isaac Lab reference:

- `rt/lowstate`
- `rt/lowcmd`

Optional depending on robot configuration:

- `rt/dex1/left/state`
- `rt/dex1/right/state`
- `rt/dex1/left/cmd`
- `rt/dex1/right/cmd`
- `rt/dex3/left/state`
- `rt/dex3/right/state`
- `rt/dex3/left/cmd`
- `rt/dex3/right/cmd`
- `rt/inspire/state`
- `rt/inspire/cmd`
- `rt/reset_pose/cmd`
- `rt/sim_state`

Key output of this phase:

- a clear statement of what an external Unitree SDK client should see

## Phase 2: Build the Isaac Sim Robot Runtime

Create the Isaac Sim application that loads and runs G1.

Responsibilities:

- initialize Isaac Sim
- use `main.py` as the Isaac Sim script entrypoint
- load the stage and G1 robot
- configure articulation physics
- reset the robot to a known starting pose
- expose accessors for:
  - joint positions
  - joint velocities
  - applied torques or estimated torques
  - base pose
  - base linear and angular velocity
  - IMU-like signals

Important constraints:

- joint names in the simulator must be mapped explicitly
- never assume simulator joint order already matches Unitree DDS order
- robot startup state should be deterministic
- asset paths should resolve against `src/unitree_g1_isaac_sim/Models`

Expected module ownership:

- `main.py`: app and loop bootstrap
- `scene.py`: robot/stage creation and reset
- `config.py`: asset path and runtime settings

## Phase 3: Build Joint Mapping and Conversion Tables

Extract and formalize the translation rules from the Isaac Lab reference.

This phase should produce:

- G1 body joint DDS order
- Isaac Sim articulation joint order
- mapping from DDS index to simulator joint index
- mapping from simulator joint index to DDS index
- hand and gripper mappings if needed
- any special per-joint scaling rules

Examples from the reference to preserve conceptually:

- body state is reordered before publishing
- lowcmd arm positions are read from specific slices of the incoming DDS layout
- gripper and hand commands use custom range conversion
- inspire hand uses per-joint normalization and denormalization

This mapping layer is the most important compatibility asset in the whole system.

## Phase 4: Implement Low-Level State Publisher

Create the DDS publisher that makes the Isaac Sim robot appear alive to the external client.

Responsibilities:

- read robot state from Isaac Sim
- obtain simulator values through `robot_state.py`
- build `LowState_` messages
- populate:
  - joint position
  - joint velocity
  - estimated torque
  - IMU fields
  - tick counter
  - CRC
- publish on `rt/lowstate`

Important behaviors to match:

- stable publish cadence
- correct quaternion ordering
- correct IMU frame convention
- valid CRC for each published message

Expected module ownership:

- `robot_state.py`: raw state extraction
- `dds/g1_lowstate.py`: message construction and publication
- `dds/manager.py`: publisher lifecycle

The published message should be derived from simulator state each cycle, not cached from commands.

## Phase 5: Implement Low-Level Command Subscriber

Create the DDS subscriber that accepts Unitree SDK commands and converts them into simulator inputs.

Responsibilities:

- subscribe to `rt/lowcmd`
- validate CRC
- extract position, velocity, torque, kp, kd arrays
- map DDS motor indices to simulator joints
- apply those commands into Isaac Sim control targets

Design requirement:

- command ingestion should be decoupled from physics stepping
- the subscriber should update a command cache
- the simulator tick should read from that cache and apply the latest valid command

This avoids applying control directly inside DDS callback threads.

Expected module ownership:

- `dds/g1_lowcmd.py`: subscriber and command decoding
- `robot_control.py`: command cache and simulator application
- `mapping/joints.py`: index translation
- `mapping/conversion.py`: command value conversion if needed

## Phase 6: Implement the Simulator Control Loop

The control loop should own authoritative state transitions.

Recommended loop structure:

1. poll or read latest cached DDS commands
2. convert into articulation targets
3. apply control to the robot
4. step physics
5. read resulting robot state
6. publish DDS outputs according to rate policy

Recommended timing design:

- fixed simulation timestep
- separate publish rate for DDS state
- command cache updated asynchronously by DDS subscribers

The state publisher should not depend on an observation framework. It should read directly from the articulation state extractor.

Expected ownership:

- `main.py`: top-level tick loop
- `robot_control.py`: apply cached commands
- `robot_state.py`: read current articulation state
- `dds/manager.py`: drive publish/update hooks if needed

## Phase 7: Add Optional Hand / Gripper Bridges

Only implement the hand type you actually need first.

Options:

- dex1 gripper
- dex3 hand
- inspire hand

For each optional hand bridge:

- define simulator joint subset
- define DDS topics
- define state conversion rules
- define command conversion rules

This should follow the same pattern as body control:

- simulator state -> DDS state message
- DDS command message -> simulator command cache

Expected module ownership:

- `dds/dex1.py`
- `dds/dex3.py`
- `dds/inspire.py`
- `mapping/joints.py`
- `mapping/conversion.py`
- `robot_control.py`
- `robot_state.py`

## Phase 8: Add Simulator Utility Topics

These are not required for a minimal real-robot mimic, but may be useful.

Optional utilities:

- reset pose command
- sim state JSON topic
- reward topic if later needed for research tooling
- diagnostics or performance topic

These should be treated as simulator-specific convenience features, not part of the core G1 hardware contract unless another downstream system depends on them.

## Phase 9: Validation Against the Real Interface

Validation should focus on interface compatibility, not just whether the sim moves.

Tests to perform:

1. Topic validation
   - confirm expected topics exist
   - confirm message types match expected IDL

2. SDK connectivity
   - run a Unitree SDK example against the simulator
   - confirm it can subscribe to state and send commands

3. Joint mapping validation
   - command one joint at a time
   - verify the correct simulator joint moves

4. State fidelity validation
   - compare published joint values with simulator articulation values
   - confirm quaternion, angular velocity, and acceleration conventions

5. CRC validation
   - confirm outgoing messages pass the same checks expected by the SDK
   - confirm invalid incoming CRC is rejected

6. Timing validation
   - measure effective publish rate
   - measure control loop latency
   - confirm no unstable callback-to-physics coupling

## Shared Memory Decision

Shared memory is not a requirement.

It is only one way to separate:

- DDS I/O layer
- simulation state/control layer

Recommended initial choice:

- do not use shared memory in version 1
- keep everything in one Isaac Sim process
- store latest command/state in normal Python objects managed by the app

When shared memory becomes useful:

- if DDS code must run in a separate process
- if callback threading causes timing instability
- if you want to reuse the Isaac Lab bridge classes with minimal redesign
- if external tooling also needs direct access to normalized state snapshots

For the first implementation, an in-process cache is simpler and lower risk.

## Minimum Viable Version

The minimum viable simulator should include only the pieces needed to let Unitree SDK code believe it is talking to a G1.

Minimum scope:

- Isaac Sim app that loads G1
- `rt/lowstate` publisher
- `rt/lowcmd` subscriber
- correct body joint mapping
- IMU publication
- fixed stepping loop
- reset to known pose

Not required for version 1:

- dex hands
- grippers
- teleoperation tools
- task logic
- replay
- camera transport
- dataset logging

## Recommended Build Order

1. Freeze DDS contract and joint order.
2. Finish `config.py` so `main.py` can resolve robot USD assets and runtime settings.
3. Build `scene.py` and verify G1 loads cleanly in Isaac Sim.
4. Build `robot_state.py` and verify raw articulation state extraction.
5. Implement body joint mapping tables in `mapping/joints.py`.
6. Implement conversions in `mapping/conversion.py`.
7. Implement `dds/manager.py`.
8. Publish `rt/lowstate` from `dds/g1_lowstate.py`.
9. Subscribe to `rt/lowcmd` in `dds/g1_lowcmd.py` and cache commands in `robot_control.py`.
10. Apply commands to Isaac Sim joints each physics step.
11. Validate with Unitree SDK test code.
12. Add hand/gripper bridges only if needed.
13. Add simulator utilities only after base compatibility is stable.

## Main Risks

## Risk 1: Wrong Joint Ordering

If DDS motor indices do not match simulator joint indices, the simulator will appear functional but incompatible.

Mitigation:

- define explicit mapping tables
- unit test one-joint-at-a-time behavior

## Risk 2: Wrong IMU Convention

Quaternion ordering, sign convention, frame convention, and acceleration derivation may differ from what external code expects.

Mitigation:

- verify against real robot topic semantics
- compare with the Isaac Lab reference and Unitree SDK assumptions

## Risk 3: Callback Threading Issues

Applying simulator state mutations directly inside DDS callbacks can cause nondeterministic behavior.

Mitigation:

- callbacks only update a command cache
- simulation loop remains the only place that mutates robot state

## Risk 4: Overbuilding Around Isaac Lab Patterns

Copying the Isaac Lab implementation too literally may add complexity that is not needed in Isaac Sim.

Mitigation:

- preserve the DDS contract
- simplify the internal runtime

## Definition of Done

This project should be considered successful when all of the following are true:

- Isaac Sim runs a G1 robot reliably
- the app publishes the expected Cyclone DDS robot state topics
- the app accepts the expected Cyclone DDS command topics
- a Unitree SDK client can control the simulated robot without client-side simulator-specific changes
- the robot state seen by the SDK is consistent with the Isaac Sim articulation state

## Immediate Next Step

Start by writing a DDS contract document from the existing Isaac Lab reference:

- exact topics
- exact message classes
- exact joint ordering
- exact conversion rules

That should be treated as the source specification for the Isaac Sim implementation.
