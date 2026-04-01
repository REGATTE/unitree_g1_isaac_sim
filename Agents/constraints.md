# Project Constraints

These constraints should remain explicit during the implementation work.

## DDS Boundary Constraints

- Keep body joint-order validation at the DDS boundary.
  - Any simulator joint-aligned data that is published to DDS must pass through the existing validated conversion layer.
  - Do not bypass `to_dds_ordered_snapshot(...)` or weaken the frozen simulator-order checks.

- Preserve Isaac Sim base orientation in `wxyz`.
  - Treat the Isaac Sim world-pose quaternion as scalar-first `wxyz`.
  - Do not introduce a silent `xyzw <-> wxyz` reorder in DDS packaging unless there is an explicitly verified external requirement for a different field layout.

- Keep IMU acceleration simulation-time derived only.
  - IMU-like linear acceleration must be computed from simulation stepping time, not host wall time.
  - If a valid simulation `sample_dt` is unavailable, fall back to the existing gravity-only proper-acceleration behavior rather than estimating from wall-clock timing.

## Current DDS Implementation Order

- Implement `rt/lowstate` first.
- Use:
  - `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
  - `to_dds_ordered_snapshot(...)` for body joint-aligned fields
  - Unitree SDK2 `LowState_`
  - CRC handling
- Only after `rt/lowstate` works, add `rt/lowcmd` and map incoming body commands back into simulator order.
