"""Torso-link IMU estimation helpers for the Unitree G1 simulator."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from runtime_logging import get_logger


LOGGER = get_logger("sensors.torso_imu")

_GRAVITY_MAGNITUDE_MPS2 = 9.81
_WORLD_GRAVITY_VECTOR = (0.0, 0.0, -_GRAVITY_MAGNITUDE_MPS2)
_WORLD_PROPER_ACCELERATION_AT_REST = (0.0, 0.0, _GRAVITY_MAGNITUDE_MPS2)


@dataclass(frozen=True)
class TorsoImuReading:
    """One simulated torso-link IMU sample."""

    quaternion_wxyz: tuple[float, ...]
    linear_acceleration_body: tuple[float, ...]
    angular_velocity_body: tuple[float, ...]


class TorsoImuSensor:
    """Estimate IMU orientation, proper acceleration, and gyro from `torso_link`."""

    def __init__(self, robot_prim_path: str, link_name: str = "torso_link") -> None:
        self._robot_prim_path = robot_prim_path.rstrip("/")
        self._link_name = link_name
        self._torso_prim_path: str | None = None
        self._last_position_world: list[float] | None = None
        self._last_linear_velocity_world: list[float] | None = None
        self._last_quaternion_wxyz: list[float] | None = None

    def initialize(self) -> None:
        if self._torso_prim_path is not None:
            return
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        torso_prim = _find_link_prim(stage, self._robot_prim_path, self._link_name)
        if torso_prim is None:
            raise RuntimeError(
                f"Unable to locate torso IMU link '{self._link_name}' under '{self._robot_prim_path}'."
            )
        self._torso_prim_path = str(torso_prim.GetPath())
        LOGGER.info("torso IMU sensor bound to %s", self._torso_prim_path)

    def reset(self) -> None:
        self._last_position_world = None
        self._last_linear_velocity_world = None
        self._last_quaternion_wxyz = None

    def read(self, sample_dt: float | None) -> TorsoImuReading:
        self.initialize()
        position_world, quaternion_wxyz = self._read_world_pose()

        if sample_dt is None or sample_dt <= 0.0 or self._last_position_world is None:
            linear_velocity_world = [0.0, 0.0, 0.0]
            linear_acceleration_body = _gravity_only_acceleration_body(quaternion_wxyz)
        else:
            linear_velocity_world = _finite_difference_vector(
                self._last_position_world,
                position_world,
                sample_dt,
            )
            if self._last_linear_velocity_world is None:
                linear_acceleration_body = _gravity_only_acceleration_body(quaternion_wxyz)
            else:
                linear_acceleration_world = _finite_difference_vector(
                    self._last_linear_velocity_world,
                    linear_velocity_world,
                    sample_dt,
                )
                proper_acceleration_world = _world_acceleration_to_proper_acceleration(linear_acceleration_world)
                linear_acceleration_body = _rotate_world_vector_to_body(
                    proper_acceleration_world,
                    quaternion_wxyz,
                )

        angular_velocity_body = _estimate_body_angular_velocity(
            self._last_quaternion_wxyz,
            quaternion_wxyz,
            sample_dt,
        )

        self._last_position_world = list(position_world)
        self._last_linear_velocity_world = list(linear_velocity_world)
        self._last_quaternion_wxyz = list(quaternion_wxyz)

        return TorsoImuReading(
            quaternion_wxyz=tuple(quaternion_wxyz),
            linear_acceleration_body=tuple(linear_acceleration_body),
            angular_velocity_body=tuple(angular_velocity_body),
        )

    def _read_world_pose(self) -> tuple[list[float], list[float]]:
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        if self._torso_prim_path is None:
            raise RuntimeError("Torso IMU sensor must be initialized before reading pose.")
        torso_prim = stage.GetPrimAtPath(self._torso_prim_path)
        if not torso_prim or not torso_prim.IsValid():
            raise RuntimeError(f"Torso IMU prim is no longer valid: {self._torso_prim_path}")

        transform = UsdGeom.Xformable(torso_prim).ComputeLocalToWorldTransform(0.0)
        position_world = transform.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        rotation = transform.ExtractRotationQuat()
        imaginary = rotation.GetImaginary()
        quaternion_wxyz = _normalize_quaternion_wxyz(
            [
                float(rotation.GetReal()),
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
            ]
        )
        return [
            float(position_world[0]),
            float(position_world[1]),
            float(position_world[2]),
        ], quaternion_wxyz


def _find_link_prim(stage, robot_prim_path: str, link_name: str):
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim or not robot_prim.IsValid():
        return None

    exact_path = f"{robot_prim_path.rstrip('/')}/{link_name}"
    exact_prim = stage.GetPrimAtPath(exact_path)
    if exact_prim and exact_prim.IsValid():
        return exact_prim

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue
        prim_path = str(prim.GetPath())
        if not prim_path.startswith(robot_prim_path.rstrip("/") + "/"):
            continue
        if prim.GetName() == link_name:
            return prim
    return None


def quaternion_wxyz_to_rpy(quaternion_wxyz: Sequence[float]) -> tuple[float, float, float]:
    """Convert a `wxyz` quaternion into roll, pitch, yaw in radians."""
    w, x, y, z = _normalize_quaternion_wxyz(quaternion_wxyz)

    sinr_cosp = 2.0 * ((w * x) + (y * z))
    cosr_cosp = 1.0 - 2.0 * ((x * x) + (y * y))
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * ((w * y) - (z * x))
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def _estimate_body_angular_velocity(
    previous_quaternion_wxyz: Sequence[float] | None,
    current_quaternion_wxyz: Sequence[float],
    sample_dt: float | None,
) -> list[float]:
    if previous_quaternion_wxyz is None or sample_dt is None or sample_dt <= 0.0:
        return [0.0, 0.0, 0.0]

    previous = _normalize_quaternion_wxyz(previous_quaternion_wxyz)
    current = _normalize_quaternion_wxyz(current_quaternion_wxyz)
    delta_body = _quat_multiply(_quat_conjugate(previous), current)
    delta_body = _normalize_quaternion_wxyz(delta_body)

    angle = 2.0 * math.atan2(
        math.sqrt(
            (delta_body[1] * delta_body[1])
            + (delta_body[2] * delta_body[2])
            + (delta_body[3] * delta_body[3])
        ),
        delta_body[0],
    )
    if angle > math.pi:
        angle -= 2.0 * math.pi
    axis_norm = math.sqrt(
        (delta_body[1] * delta_body[1])
        + (delta_body[2] * delta_body[2])
        + (delta_body[3] * delta_body[3])
    )
    if axis_norm <= 1e-9:
        return [0.0, 0.0, 0.0]
    axis = [
        delta_body[1] / axis_norm,
        delta_body[2] / axis_norm,
        delta_body[3] / axis_norm,
    ]
    return [axis_component * (angle / sample_dt) for axis_component in axis]


def _gravity_only_acceleration_body(quaternion_wxyz: Sequence[float]) -> list[float]:
    return _rotate_world_vector_to_body(list(_WORLD_PROPER_ACCELERATION_AT_REST), list(quaternion_wxyz))


def _world_acceleration_to_proper_acceleration(acceleration_world: Sequence[float]) -> list[float]:
    return [
        acceleration_world[0] - _WORLD_GRAVITY_VECTOR[0],
        acceleration_world[1] - _WORLD_GRAVITY_VECTOR[1],
        acceleration_world[2] - _WORLD_GRAVITY_VECTOR[2],
    ]


def _finite_difference_vector(
    previous: Sequence[float],
    current: Sequence[float],
    dt: float,
) -> list[float]:
    return [
        (float(current[index]) - float(previous[index])) / dt
        for index in range(3)
    ]


def _rotate_world_vector_to_body(vector_world: list[float], quaternion_wxyz: list[float]) -> list[float]:
    rotation_body_to_world = _quat_wxyz_to_rotation_matrix(quaternion_wxyz)
    rotation_world_to_body = _transpose_mat3(rotation_body_to_world)
    return _mat3_mul_vec3(rotation_world_to_body, vector_world)


def _quat_conjugate(quaternion_wxyz: Sequence[float]) -> list[float]:
    quaternion = _normalize_quaternion_wxyz(quaternion_wxyz)
    return [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]


def _quat_multiply(lhs_wxyz: Sequence[float], rhs_wxyz: Sequence[float]) -> list[float]:
    lw, lx, ly, lz = lhs_wxyz
    rw, rx, ry, rz = rhs_wxyz
    return [
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
    ]


def _normalize_quaternion_wxyz(quaternion_wxyz: Sequence[float]) -> list[float]:
    normalized = [float(value) for value in quaternion_wxyz[:4]]
    if len(normalized) < 4:
        normalized.extend([0.0] * (4 - len(normalized)))
    norm = math.sqrt(sum(value * value for value in normalized))
    if norm <= 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in normalized]


def _quat_wxyz_to_rotation_matrix(quaternion_wxyz: Sequence[float]) -> list[list[float]]:
    w, x, y, z = _normalize_quaternion_wxyz(quaternion_wxyz)
    return [
        [1.0 - (2.0 * y * y) - (2.0 * z * z), (2.0 * x * y) - (2.0 * z * w), (2.0 * x * z) + (2.0 * y * w)],
        [(2.0 * x * y) + (2.0 * z * w), 1.0 - (2.0 * x * x) - (2.0 * z * z), (2.0 * y * z) - (2.0 * x * w)],
        [(2.0 * x * z) - (2.0 * y * w), (2.0 * y * z) + (2.0 * x * w), 1.0 - (2.0 * x * x) - (2.0 * y * y)],
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
