# DDS Testing

This document describes the current bring-up path for testing the G1 DDS
interface exposed by `unitree_g1_isaac_sim`.

The current DDS surface under active development is:

- `rt/lowstate` publish
- `rt/lowcmd` subscribe

The intent is for the simulator to look like a real G1 at the DDS boundary so
external clients such as `unitree_sdk2` or `unitree_ros2` can run on top of it
without simulator-specific changes.

## Current Scope

What is currently expected to work:

- the simulator publishes `LowState_` on `rt/lowstate`
- the simulator subscribes to `LowCmd_` on `rt/lowcmd`
- incoming body commands are remapped from DDS joint order back into simulator
  joint order
- the minimum viable command path applies body joint targets back into the live
  articulation

Important current limitation:

- the `lowcmd` path should be treated primarily as a position-command path
- velocity and effort fields are forwarded when the articulation runtime
  exposes matching setters
- dynamic `kp` and `kd` application is implemented, but validation should still
  begin with conservative gain values and a small set of safe joints

## Launch the Simulator

Run the simulator with DDS enabled and the lowcmd subscriber enabled:

```bash
isaac_sim_python ~/Workspaces/ros2_ws/src/unitree_g1_isaac_sim/src/main.py --enable-dds --enable-lowcmd-subscriber
```

This should:

- initialize the Unitree DDS channel factory
- publish `rt/lowstate`
- subscribe to `rt/lowcmd`

## Test `rt/lowstate`

Start with passive state observation before sending commands.

Use either:

- a Unitree SDK2 G1 low-state subscriber example
- `scripts/lowstate_listener.py` in this repo
- a small `unitree_sdk2py` subscriber script
- a `unitree_ros2` node that already expects the real low-level DDS state topic

Example:

```bash
python3 scripts/lowstate_listener.py --dds-domain-id 1 --duration 5
```

Check the following:

- messages arrive continuously on `rt/lowstate`
- `tick` increases over time
- `motor_state[i].q`, `motor_state[i].dq`, and `motor_state[i].tau_est` look
  physically reasonable
- IMU quaternion, accelerometer, and gyroscope fields are populated
- CRC is valid on the received message path

## Test `rt/lowcmd`

After `rt/lowstate` is confirmed, test closed-loop command reception.

Use either:

- a Unitree SDK2 low-level command example
- `scripts/send_lowcmd_offset.py` in this repo
- a small `unitree_sdk2py` publisher that writes `LowCmd_` to `rt/lowcmd`

Recommended first command strategy:

- keep the command small and conservative
- command only a safe subset of joints first
- prefer a single waist or arm joint rather than a whole-body posture jump

Good first joints to try:

- `waist_yaw_joint`
- `left_shoulder_pitch_joint`
- `right_shoulder_pitch_joint`
- `left_elbow_joint`
- `right_elbow_joint`

Example:

```bash
python3 scripts/send_lowcmd_offset.py \
  --dds-domain-id 1 \
  --joint-name left_shoulder_pitch_joint \
  --offset-rad 0.10 \
  --duration 2.0
```

What to verify:

- the simulator receives the command without CRC rejection
- the targeted joint moves in Isaac Sim
- the next `rt/lowstate` samples reflect the commanded motion

## Recommended Minimal Test Sequence

1. Launch the simulator with:
   - `--enable-dds`
   - `--enable-lowcmd-subscriber`
2. Subscribe to `rt/lowstate`.
3. Confirm that state messages are arriving and that `tick` is increasing.
4. Publish one static `LowCmd_` with a very small offset on a single safe arm
   or waist joint.
5. Observe the joint motion in Isaac Sim.
6. Confirm that the changed joint position is reflected back on `rt/lowstate`.

## Safety Notes

- Do not begin with aggressive full-body commands.
- Do not begin with aggressive `kp` / `kd` changes across the whole body.
- Start with small position offsets and inspect the response before sending
  broader body commands.
- If a real robot is present on the same DDS network, keep the topic/domain
  setup isolated so the simulator and real robot are not confused with each
  other.

## Next Validation Step

The highest-value next validation is:

- use a real external Unitree SDK2 or `unitree_ros2` client against the
  simulator
- confirm that the client can read `rt/lowstate`
- confirm that the client can send `rt/lowcmd`
- verify that the simulator responds without requiring simulator-specific
  client modifications
