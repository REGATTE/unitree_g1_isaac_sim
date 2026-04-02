"""Robot articulation discovery and state access helpers.

This module isolates the first articulation-facing code path for the project.
It exists to answer one question before any DDS work starts:
can the G1 robot be discovered reliably and can its joint state be read?
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


_GRAVITY_MAGNITUDE_MPS2 = 9.81
_WORLD_GRAVITY_VECTOR = (0.0, 0.0, -_GRAVITY_MAGNITUDE_MPS2)
_WORLD_PROPER_ACCELERATION_AT_REST = (0.0, 0.0, _GRAVITY_MAGNITUDE_MPS2)


class PhysicsViewUnavailableError(RuntimeError):
    """Raised when Isaac Sim temporarily stops exposing articulation physics data.

    This can happen while the simulator is paused or shutting down. Callers
    should treat it as a recoverable runtime condition and skip the affected
    frame instead of terminating the application.
    """


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

    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_efforts: tuple[float, ...] | None
    base_position_world: tuple[float, ...]
    base_quaternion_wxyz: tuple[float, ...]
    base_linear_velocity_world: tuple[float, ...]
    base_angular_velocity_world: tuple[float, ...]
    imu_linear_acceleration_body: tuple[float, ...]
    imu_angular_velocity_body: tuple[float, ...]
    quaternion_convention: str = "wxyz"

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint_names", tuple(self.joint_names))
        object.__setattr__(self, "joint_positions", tuple(self.joint_positions))
        object.__setattr__(self, "joint_velocities", tuple(self.joint_velocities))
        if self.joint_efforts is not None:
            object.__setattr__(self, "joint_efforts", tuple(self.joint_efforts))
        object.__setattr__(self, "base_position_world", tuple(self.base_position_world))
        object.__setattr__(self, "base_quaternion_wxyz", tuple(self.base_quaternion_wxyz))
        object.__setattr__(self, "base_linear_velocity_world", tuple(self.base_linear_velocity_world))
        object.__setattr__(self, "base_angular_velocity_world", tuple(self.base_angular_velocity_world))
        object.__setattr__(self, "imu_linear_acceleration_body", tuple(self.imu_linear_acceleration_body))
        object.__setattr__(self, "imu_angular_velocity_body", tuple(self.imu_angular_velocity_body))


class ImuEmulator:
    """Small helper for IMU-like proper acceleration estimation.

    The simulator world is assumed to use a z-up convention with gravity
    pointing along [0, 0, -9.81] in world coordinates. The returned linear
    acceleration matches the usual IMU proper-acceleration convention, so a
    stationary upright robot reports approximately [0, 0, +9.81] in body frame.
    """

    def __init__(self) -> None:
        self._last_linear_velocity_world: list[float] | None = None

    def compute_body_linear_acceleration(
        self,
        linear_velocity_world: list[float],
        quaternion_wxyz: list[float],
        sample_dt: float | None,
    ) -> list[float]:
        """Estimate body-frame proper acceleration using simulation time only.

        `sample_dt` must come from simulator stepping, not host wall time.
        When no simulation dt is supplied, or there is no previous sample yet,
        fall back to the stationary gravity-only proper-acceleration reading.
        """
        if sample_dt is None or sample_dt <= 0.0 or self._last_linear_velocity_world is None:
            acceleration_body = _gravity_only_acceleration_body(quaternion_wxyz)
        else:
            acceleration_world = _finite_difference_acceleration(
                self._last_linear_velocity_world,
                linear_velocity_world,
                sample_dt,
            )
            proper_acceleration_world = _world_acceleration_to_proper_acceleration(acceleration_world)
            acceleration_body = _rotate_world_vector_to_body(
                proper_acceleration_world,
                quaternion_wxyz,
            )

        self._last_linear_velocity_world = list(linear_velocity_world)
        return acceleration_body


def import_articulation():
    """Import the articulation wrapper with Isaac Sim 5.x/4.x compatibility."""
    try:
        from isaacsim.core.prims import Articulation
    except ImportError:
        from omni.isaac.core.prims import Articulation
    return Articulation


class RobotStateReader:
    """Thin articulation wrapper for reading and commanding simulator joints.

    The first development slices in this repository focused on reliable state
    extraction. The DDS command path now reuses the same articulation wrapper
    so joint-order assumptions stay localized and the rest of the codebase does
    not need to know which Isaac Sim articulation methods are available in a
    given runtime version.
    """

    def __init__(self, robot_prim_path: str) -> None:
        Articulation = import_articulation()
        self._robot_prim_path = robot_prim_path
        self._articulation = Articulation(prim_paths_expr=robot_prim_path, name="unitree_g1")
        self._initialized = False
        self._imu = ImuEmulator()
        self._warned_physics_view_unavailable = False

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
        self._require_initialized()
        joint_positions, joint_velocities, joint_efforts = self._read_joint_state()
        return JointStateSnapshot(
            joint_names=self.joint_names,
            joint_positions=_to_float_list(joint_positions),
            joint_velocities=_to_float_list(joint_velocities),
            joint_efforts=_to_float_list(joint_efforts) if joint_efforts is not None else None,
        )

    def read_kinematic_snapshot(self, sample_dt: float | None = None) -> RobotKinematicSnapshot:
        """Read joint state plus base and IMU-like signals for DDS publication.

        Quaternion output is normalized to `wxyz`. IMU-like signals are exposed
        in the body frame so downstream DDS packaging does not need to infer
        frame conventions from raw simulator APIs. Dynamic acceleration
        estimation uses `sample_dt` only when it is supplied from simulation
        time; otherwise the IMU acceleration falls back to a gravity-only
        stationary reading.
        """
        self._require_initialized()

        joint_positions, joint_velocities, joint_efforts = self._read_joint_state()

        base_position_world, base_quaternion_wxyz = self._read_world_pose()
        base_linear_velocity_world = self._read_linear_velocity_world()
        base_angular_velocity_world = self._read_angular_velocity_world()
        imu_linear_acceleration_body = self._imu.compute_body_linear_acceleration(
            base_linear_velocity_world,
            base_quaternion_wxyz,
            sample_dt,
        )
        imu_angular_velocity_body = _rotate_world_vector_to_body(
            base_angular_velocity_world,
            base_quaternion_wxyz,
        )

        return RobotKinematicSnapshot(
            joint_names=tuple(self.joint_names),
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

    def apply_joint_position_targets(self, joint_positions: Sequence[float]) -> bool:
        """Apply a full-body joint-position target vector in simulator order.

        Returns `True` when a target setter is found on the articulation.
        Falls back to `set_joint_positions()` only if a dedicated target API is
        unavailable in the current Isaac Sim runtime.
        """
        return self._apply_joint_vector(
            joint_positions,
            label="joint_positions",
            primary_method="set_joint_position_targets",
            fallback_method="set_joint_positions",
        )

    def apply_joint_velocity_targets(self, joint_velocities: Sequence[float]) -> bool:
        """Apply a full-body joint-velocity target vector in simulator order."""
        return self._apply_joint_vector(
            joint_velocities,
            label="joint_velocities",
            primary_method="set_joint_velocity_targets",
            fallback_method="set_joint_velocities",
        )

    def apply_joint_efforts(self, joint_efforts: Sequence[float]) -> bool:
        """Apply a full-body joint-effort vector in simulator order."""
        return self._apply_joint_vector(
            joint_efforts,
            label="joint_efforts",
            primary_method="set_joint_efforts",
        )

    def apply_joint_gains(self, joint_kp: Sequence[float], joint_kd: Sequence[float]) -> bool:
        """Apply full-body PD gains in simulator joint order.

        Unitree `kp` and `kd` are mapped directly onto Isaac Sim articulation
        stiffness and damping. The primary path uses `Articulation.set_gains`,
        which is the stable API exposed by the current Isaac Sim runtime. A
        controller-level fallback is kept for compatibility with runtimes that
        only expose gain updates through `get_articulation_controller()`.
        """
        self._require_initialized()
        self._require_physics_view_ready()
        joint_count = len(self.joint_names)
        normalized_kp = _validate_joint_command_width(joint_kp, joint_count, "joint_kp")
        normalized_kd = _validate_joint_command_width(joint_kd, joint_count, "joint_kd")

        if hasattr(self._articulation, "set_gains"):
            self._articulation.set_gains(
                kps=np.expand_dims(np.asarray(normalized_kp, dtype=np.float32), axis=0),
                kds=np.expand_dims(np.asarray(normalized_kd, dtype=np.float32), axis=0),
            )
            return True

        if hasattr(self._articulation, "get_articulation_controller"):
            controller = self._articulation.get_articulation_controller()
            if controller is not None and hasattr(controller, "set_gains"):
                controller.set_gains(
                    kps=np.asarray(normalized_kp, dtype=np.float32),
                    kds=np.asarray(normalized_kd, dtype=np.float32),
                )
                return True

        return False

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("RobotStateReader must be initialized before reading state.")

    def _read_joint_state(self):
        return self._ensure_physics_view_ready(require_velocities=True, require_efforts=True)

    def _require_physics_view_ready(self) -> None:
        """Fail fast when the articulation loses its live physics view."""
        self._ensure_physics_view_liveness()

    def _apply_joint_vector(
        self,
        values: Sequence[float],
        *,
        label: str,
        primary_method: str,
        fallback_method: str | None = None,
    ) -> bool:
        self._require_initialized()
        self._require_physics_view_ready()
        normalized = _validate_joint_command_width(values, len(self.joint_names), label)

        if hasattr(self._articulation, primary_method):
            getattr(self._articulation, primary_method)(normalized)
            return True

        if fallback_method is not None and hasattr(self._articulation, fallback_method):
            getattr(self._articulation, fallback_method)(normalized)
            return True

        return False

    def _ensure_physics_view_ready(
        self,
        *,
        require_velocities: bool = False,
        require_efforts: bool = False,
    ) -> tuple[list[float], list[float] | None, list[float] | None]:
        expected_joint_count = len(self.joint_names)
        if expected_joint_count == 0:
            self._raise_physics_view_unavailable("articulation joint names are unavailable")

        positions = _to_float_list(self._articulation.get_joint_positions())
        velocities = None
        efforts = None

        if require_velocities:
            velocities = _to_float_list(self._articulation.get_joint_velocities())
        if require_efforts and hasattr(self._articulation, "get_measured_joint_efforts"):
            efforts = _to_float_list(self._articulation.get_measured_joint_efforts())

        if len(positions) != expected_joint_count:
            self._raise_physics_view_unavailable(
                "joint position buffer is unavailable "
                f"(expected {expected_joint_count}, got {len(positions)})"
            )
        if velocities is not None and len(velocities) != expected_joint_count:
            self._raise_physics_view_unavailable(
                "joint velocity buffer is unavailable "
                f"(expected {expected_joint_count}, got {len(velocities)})"
            )
        if efforts is not None and len(efforts) not in (0, expected_joint_count):
            self._raise_physics_view_unavailable(
                "joint effort buffer width is invalid "
                f"(expected {expected_joint_count}, got {len(efforts)})"
            )

        self._mark_physics_view_ready()
        return positions, velocities, efforts

    def _ensure_physics_view_liveness(self) -> None:
        """Validate that the articulation physics view is present for control writes."""
        expected_joint_count = len(self.joint_names)
        if expected_joint_count == 0:
            self._raise_physics_view_unavailable("articulation joint names are unavailable")

        positions = self._articulation.get_joint_positions()
        position_count = len(_to_float_list(positions))
        if position_count != expected_joint_count:
            self._raise_physics_view_unavailable(
                "joint position buffer is unavailable "
                f"(expected {expected_joint_count}, got {position_count})"
            )
        self._mark_physics_view_ready()

    def _raise_physics_view_unavailable(self, reason: str) -> None:
        if not self._warned_physics_view_unavailable:
            print(
                "[unitree_g1_isaac_sim] articulation physics view unavailable; "
                "skipping joint state/control updates until physics resumes. "
                f"reason={reason}"
            )
            self._warned_physics_view_unavailable = True
        raise PhysicsViewUnavailableError(reason)

    def _mark_physics_view_ready(self) -> None:
        if self._warned_physics_view_unavailable:
            print("[unitree_g1_isaac_sim] articulation physics view restored; resuming DDS/state updates.")
        self._warned_physics_view_unavailable = False

    def _read_world_pose(self) -> tuple[list[float], list[float]]:
        """Read the base world pose from Isaac Sim.

        `get_world_poses()` is treated as returning scalar-first quaternions in
        wxyz order. Keep that ordering unchanged here so all downstream frame
        math uses the raw Isaac Sim convention directly.
        """
        if not hasattr(self._articulation, "get_world_poses"):
            # Keep the fallback aligned with the documented scalar-first `wxyz`
            # convention: identity rotation is [1, 0, 0, 0].
            return [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
        positions, orientations = self._articulation.get_world_poses()
        return (
            _to_fixed_float_list(positions, 3, strict=True),
            _to_fixed_float_list(orientations, 4, strict=True),
        )

    def _read_linear_velocity_world(self) -> list[float]:
        if not hasattr(self._articulation, "get_linear_velocities"):
            return [0.0, 0.0, 0.0]
        values = self._articulation.get_linear_velocities()
        return _to_fixed_float_list(values, 3)

    def _read_angular_velocity_world(self) -> list[float]:
        if not hasattr(self._articulation, "get_angular_velocities"):
            return [0.0, 0.0, 0.0]
        values = self._articulation.get_angular_velocities()
        return _to_fixed_float_list(values, 3)


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


def _to_fixed_float_list(
    values,
    expected_length: int,
    *,
    strict: bool = False,
) -> list[float]:
    normalized = _to_float_list(values)
    if strict and len(normalized) != expected_length:
        raise ValueError(f"Expected vector of length {expected_length}, got {len(normalized)}")
    if len(normalized) < expected_length:
        normalized = normalized + [0.0] * (expected_length - len(normalized))
    return normalized[:expected_length]


def _validate_joint_command_width(
    values: Sequence[float],
    expected_length: int,
    label: str,
) -> list[float]:
    """Normalize and validate a full-width joint command vector."""
    normalized = [float(value) for value in values]
    if len(normalized) != expected_length:
        raise ValueError(f"Expected {expected_length} {label} values, got {len(normalized)}")
    return normalized


def _rotate_world_vector_to_body(vector_world: list[float], quaternion_wxyz: list[float]) -> list[float]:
    """Rotate a world-frame vector into the body frame using a wxyz quaternion.

    `_quat_wxyz_to_rotation_matrix()` returns a body-to-world matrix `R`.
    Converting a world-frame vector into body coordinates is therefore `R^T * v`.
    """
    rotation_body_to_world = _quat_wxyz_to_rotation_matrix(quaternion_wxyz)
    rotation_world_to_body = _transpose_mat3(rotation_body_to_world)
    return _mat3_mul_vec3(rotation_world_to_body, vector_world)


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


def _gravity_only_acceleration_body(quaternion_wxyz: list[float]) -> list[float]:
    """Return the body-frame proper acceleration for a stationary robot."""
    return _rotate_world_vector_to_body(list(_WORLD_PROPER_ACCELERATION_AT_REST), quaternion_wxyz)


def _finite_difference_acceleration(
    previous_velocity_world: Sequence[float],
    current_velocity_world: Sequence[float],
    dt: float,
) -> list[float]:
    return [
        (current_velocity_world[index] - previous_velocity_world[index]) / dt
        for index in range(3)
    ]


def _world_acceleration_to_proper_acceleration(acceleration_world: Sequence[float]) -> list[float]:
    """Convert world-frame linear acceleration into IMU-style proper acceleration.

    World gravity is assumed to be [0, 0, -9.81], so proper acceleration is
    `a - g`, which adds +9.81 along world z in this z-up convention.
    """
    return [
        acceleration_world[0] - _WORLD_GRAVITY_VECTOR[0],
        acceleration_world[1] - _WORLD_GRAVITY_VECTOR[1],
        acceleration_world[2] - _WORLD_GRAVITY_VECTOR[2],
    ]


def _transpose_mat3(matrix: list[list[float]]) -> list[list[float]]:
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def _mat3_mul_vec3(matrix: list[list[float]], vector: Sequence[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def log_kinematic_snapshot(snapshot: RobotKinematicSnapshot) -> None:
    """Print a one-time startup summary for base pose conventions."""
    position = ", ".join(f"{value:.6f}" for value in snapshot.base_position_world)
    quaternion = ", ".join(f"{value:.6f}" for value in snapshot.base_quaternion_wxyz)
    print(f"[unitree_g1_isaac_sim] base_position_world=({position})")
    print(
        "[unitree_g1_isaac_sim] base_quaternion_wxyz="
        f"({quaternion}) from Isaac Sim get_world_poses()"
    )


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
