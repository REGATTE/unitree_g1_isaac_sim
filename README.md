# unitree_g1_isaac_sim

This branch, `transition/unitree_ros2`, is focused on shifting the
simulator's low-level integration path from `unitree_sdk2py` to
`unitree_ros2`.

## Current Branch Goal

The immediate implementation target is the pre-Milestone 1 work in
[Agents/implementation_plan_ros2.md](Agents/implementation_plan_ros2.md):

- remove `unitree_sdk2py` runtime dependence from the simulator-facing
  low-level path
- replace message construction / parsing with the ROS 2 path
- replace publisher / subscriber transport with the ROS 2 path
- replace manual CRC handling if the ROS 2 path no longer requires it

## Documentation Split

- This `README.md` is now the active branch README for the
  `unitree_ros2` transition work.
- The previous SDK2-oriented documentation has been preserved in
  [README_unitree_sdk2py.md](README_unitree_sdk2py.md).

## Next Steps

The next implementation steps on this branch are:

1. identify the exact `unitree_sdk2py` runtime seams to replace
2. introduce the `unitree_ros2` transport/message path
3. then proceed into Milestone 1 compatibility work:
   - `wxyz` outward quaternion layout
   - real 500 Hz `lowstate`
   - high-frequency `lowstate` and `/lowcmd` only
