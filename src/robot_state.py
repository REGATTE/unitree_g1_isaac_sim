"""Robot articulation discovery and state access helpers.

This module isolates the first articulation-facing code path for the project.
It exists to answer one question before any DDS work starts:
can the G1 robot be discovered reliably and can its joint state be read?
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class JointStateSnapshot:
    """Small immutable container for one articulation state sample."""

    joint_names: list[str]
    joint_positions: list[float]
    joint_velocities: list[float]
    joint_efforts: list[float] | None


@dataclass(frozen=True)
class RobotKinematicSnapshot:
    """Expanded state sample for DDS-facing robot-state publication.

    This is intentionally the state boundary that future `rt/lowstate`
    publication should consume. Keeping the convention explicit here avoids
    duplicating quaternion/frame assumptions inside DDS code later.
    """

    joint_names: list[str]
    joint_positions: list[float]
    joint_velocities: list[float]
    joint_efforts: list[float] | None
    base_position_world: list[float]
    base_quaternion_wxyz: list[float]
    base_linear_velocity_world: list[float]
    base_angular_velocity_world: list[float]
    imu_linear_acceleration_body: list[float]
    imu_angular_velocity_body: list[float]
    quaternion_convention: str = "wxyz"


def import_articulation():
    """Import the articulation wrapper with Isaac Sim 5.x/4.x compatibility."""
    try:
        from isaacsim.core.prims import Articulation
    except ImportError:
        from omni.isaac.core.prims import Articulation
    return Articulation


class RobotStateReader:
    """Thin articulation wrapper for reading simulator joint state."""

    def __init__(self, robot_prim_path: str) -> None:
        Articulation = import_articulation()
        self._robot_prim_path = robot_prim_path
        self._articulation = Articulation(prim_paths_expr=robot_prim_path, name="unitree_g1")
        self._initialized = False
        self._last_linear_velocity_world: list[float] | None = None
        self._last_read_time: float | None = None

    def initialize(self) -> None:
        if self._initialized:
            return
        self._articulation.initialize()
        self._initialized = True

    @property
    def joint_names(self) -> list[str]:
        names = getattr(self._articulation, "dof_names", None)
        if names is None:
            names = []
        return list(names)

    def read_snapshot(self) -> JointStateSnapshot:
        """Read the current joint-level state from the simulator."""
        state = self.read_kinematic_snapshot()
        return JointStateSnapshot(
            joint_names=state.joint_names,
            joint_positions=state.joint_positions,
            joint_velocities=state.joint_velocities,
            joint_efforts=state.joint_efforts,
        )

    def read_kinematic_snapshot(self, sample_dt: float | None = None) -> RobotKinematicSnapshot:
        """Read joint state plus base and IMU-like signals for DDS publication.

        Quaternion output is normalized to `wxyz`. IMU-like signals are exposed
        in the body frame so downstream DDS packaging does not need to infer
        frame conventions from raw simulator APIs.
        """
        if not self._initialized:
            raise RuntimeError("RobotStateReader must be initialized before reading state.")

        joint_positions = self._articulation.get_joint_positions()
        joint_velocities = self._articulation.get_joint_velocities()
        joint_efforts = None
        if hasattr(self._articulation, "get_measured_joint_efforts"):
            joint_efforts = self._articulation.get_measured_joint_efforts()

        base_position_world, base_quaternion_xyzw = self._read_world_pose()
        # Isaac Sim world-pose APIs commonly report quaternions as xyzw.
        # Convert once here so downstream code can treat wxyz as canonical.
        base_quaternion_wxyz = _quat_xyzw_to_wxyz(base_quaternion_xyzw)
        base_linear_velocity_world = self._read_world_vector("get_linear_velocities")
        base_angular_velocity_world = self._read_world_vector("get_angular_velocities")
        imu_linear_acceleration_body = self._compute_body_linear_acceleration(
            base_linear_velocity_world,
            base_quaternion_wxyz,
            sample_dt,
        )
        imu_angular_velocity_body = _rotate_world_vector_to_body(
            base_angular_velocity_world,
            base_quaternion_wxyz,
        )

        return RobotKinematicSnapshot(
            joint_names=self.joint_names,
            joint_positions=_to_float_list(joint_positions),
            joint_velocities=_to_float_list(joint_velocities),
            joint_efforts=_to_float_list(joint_efforts) if joint_efforts is not None else None,
            base_position_world=base_position_world,
            base_quaternion_wxyz=base_quaternion_wxyz,
            base_linear_velocity_world=base_linear_velocity_world,
            base_angular_velocity_world=base_angular_velocity_world,
            imu_linear_acceleration_body=imu_linear_acceleration_body,
            imu_angular_velocity_body=imu_angular_velocity_body,
        )

    def _read_world_pose(self) -> tuple[list[float], list[float]]:
        if not hasattr(self._articulation, "get_world_poses"):
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        positions, orientations = self._articulation.get_world_poses()
        return _to_fixed_float_list(positions, 3), _to_fixed_float_list(orientations, 4)

    def _read_world_vector(self, method_name: str) -> list[float]:
        if not hasattr(self._articulation, method_name):
            return [0.0, 0.0, 0.0]
        values = getattr(self._articulation, method_name)()
        return _to_fixed_float_list(values, 3)

    def _compute_body_linear_acceleration(
        self,
        linear_velocity_world: list[float],
        quaternion_wxyz: list[float],
        sample_dt: float | None,
    ) -> list[float]:
        now = time.perf_counter()
        dt = sample_dt
        if dt is None and self._last_read_time is not None:
            dt = now - self._last_read_time

        if dt is None or dt <= 0.0 or self._last_linear_velocity_world is None:
            gravity_world = [0.0, 0.0, 9.81]
            acceleration_body = _rotate_world_vector_to_body(gravity_world, quaternion_wxyz)
        else:
            acceleration_world = [
                (linear_velocity_world[index] - self._last_linear_velocity_world[index]) / dt
                for index in range(3)
            ]
            proper_acceleration_world = [
                acceleration_world[0],
                acceleration_world[1],
                acceleration_world[2] + 9.81,
            ]
            acceleration_body = _rotate_world_vector_to_body(
                proper_acceleration_world,
                quaternion_wxyz,
            )

        self._last_linear_velocity_world = list(linear_velocity_world)
        self._last_read_time = now
        return acceleration_body


def _to_float_list(values) -> list[float]:
    """Normalize Isaac Sim array-like outputs into a plain Python float list."""
    if values is None:
        return []
    if hasattr(values, "tolist"):
        values = values.tolist()
    # Some Isaac Sim APIs return ``[[...]]`` for a single articulation view.
    if isinstance(values, (list, tuple)) and values and isinstance(values[0], (list, tuple)):
        values = values[0]
    return [float(value) for value in values]


def _to_fixed_float_list(values, expected_length: int) -> list[float]:
    normalized = _to_float_list(values)
    if len(normalized) < expected_length:
        normalized = normalized + [0.0] * (expected_length - len(normalized))
    return normalized[:expected_length]


def _quat_xyzw_to_wxyz(quaternion_xyzw: list[float]) -> list[float]:
    """Convert Isaac Sim-style xyzw quaternions into the DDS-facing wxyz form."""
    return [
        quaternion_xyzw[3],
        quaternion_xyzw[0],
        quaternion_xyzw[1],
        quaternion_xyzw[2],
    ]


def _rotate_world_vector_to_body(vector_world: list[float], quaternion_wxyz: list[float]) -> list[float]:
    """Rotate a world-frame vector into the body frame using a wxyz quaternion."""
    rotation_matrix = _quat_wxyz_to_rotation_matrix(quaternion_wxyz)
    return [
        rotation_matrix[0][0] * vector_world[0]
        + rotation_matrix[1][0] * vector_world[1]
        + rotation_matrix[2][0] * vector_world[2],
        rotation_matrix[0][1] * vector_world[0]
        + rotation_matrix[1][1] * vector_world[1]
        + rotation_matrix[2][1] * vector_world[2],
        rotation_matrix[0][2] * vector_world[0]
        + rotation_matrix[1][2] * vector_world[1]
        + rotation_matrix[2][2] * vector_world[2],
    ]


def _quat_wxyz_to_rotation_matrix(quaternion_wxyz: list[float]) -> list[list[float]]:
    """Build a body-to-world rotation matrix from a normalized wxyz quaternion."""
    w, x, y, z = quaternion_wxyz
    norm = math.sqrt((w * w) + (x * x) + (y * y) + (z * z))
    if norm == 0.0:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return [
        [1.0 - (2.0 * y * y) - (2.0 * z * z), (2.0 * x * y) - (2.0 * z * w), (2.0 * x * z) + (2.0 * y * w)],
        [(2.0 * x * y) + (2.0 * z * w), 1.0 - (2.0 * x * x) - (2.0 * z * z), (2.0 * y * z) - (2.0 * x * w)],
        [(2.0 * x * z) - (2.0 * y * w), (2.0 * y * z) + (2.0 * x * w), 1.0 - (2.0 * x * x) - (2.0 * y * y)],
    ]


def log_joint_state(
    snapshot: JointStateSnapshot,
    limit: int | None = 12,
    order_label: str | None = None,
) -> None:
    """Print a startup joint-state preview.

    Set ``limit`` to ``None`` to print the full articulation order.
    Use ``order_label`` to distinguish simulator-order and DDS-order logs.
    """
    label_prefix = f"{order_label}_" if order_label else ""
    total = len(snapshot.joint_names)
    print(f"[unitree_g1_isaac_sim] detected {total} {label_prefix}joints")
    visible_joint_count = total if limit is None else min(total, limit)
    for index, name in enumerate(snapshot.joint_names[:visible_joint_count]):
        position = snapshot.joint_positions[index] if index < len(snapshot.joint_positions) else float("nan")
        velocity = snapshot.joint_velocities[index] if index < len(snapshot.joint_velocities) else float("nan")
        if snapshot.joint_efforts is not None and index < len(snapshot.joint_efforts):
            effort = snapshot.joint_efforts[index]
            print(
                f"[unitree_g1_isaac_sim] {label_prefix}joint[{index:02d}] {name}: "
                f"pos={position:.6f} vel={velocity:.6f} effort={effort:.6f}"
            )
        else:
            print(
                f"[unitree_g1_isaac_sim] {label_prefix}joint[{index:02d}] {name}: "
                f"pos={position:.6f} vel={velocity:.6f}"
            )
    if limit is not None and total > limit:
        print(f"[unitree_g1_isaac_sim] ... truncated {total - limit} additional {label_prefix}joints")
