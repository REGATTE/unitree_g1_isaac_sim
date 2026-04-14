"""Localhost packet protocol between Isaac Sim and the native SDK bridge."""

from __future__ import annotations

import json
from typing import Any

from mapping import to_dds_ordered_snapshot
from robot_state import JointStateSnapshot, RobotKinematicSnapshot


def encode_native_lowstate_packet(snapshot: RobotKinematicSnapshot, tick: int) -> bytes:
    """Serialize a kinematic snapshot for the native Unitree SDK sidecar."""
    dds_joint_snapshot = to_dds_ordered_snapshot(
        JointStateSnapshot(
            joint_names=list(snapshot.joint_names),
            joint_positions=list(snapshot.joint_positions),
            joint_velocities=list(snapshot.joint_velocities),
            joint_efforts=list(snapshot.joint_efforts) if snapshot.joint_efforts is not None else None,
        )
    )
    payload = {
        "tick": int(tick),
        "joint_positions_dds": [float(value) for value in dds_joint_snapshot.joint_positions],
        "joint_velocities_dds": [float(value) for value in dds_joint_snapshot.joint_velocities],
        "joint_efforts_dds": [float(value) for value in (dds_joint_snapshot.joint_efforts or [])],
        "imu_quaternion_wxyz": [float(value) for value in snapshot.base_quaternion_wxyz],
        "imu_accelerometer_body": [float(value) for value in snapshot.imu_linear_acceleration_body],
        "imu_gyroscope_body": [float(value) for value in snapshot.imu_angular_velocity_body],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_native_lowstate_packet(packet: bytes) -> dict[str, Any]:
    """Parse a native lowstate localhost packet."""
    return json.loads(packet.decode("utf-8"))
