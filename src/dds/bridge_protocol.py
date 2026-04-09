"""Localhost packet protocol between Isaac Sim and the ROS 2 sidecar bridge."""

from __future__ import annotations

import json
from typing import Any

from robot_state import JointStateSnapshot, RobotKinematicSnapshot
from mapping import to_dds_ordered_snapshot


def encode_lowstate_packet(snapshot: RobotKinematicSnapshot, tick: int) -> bytes:
    """Serialize a kinematic snapshot into a compact localhost packet."""
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
        "joint_positions": [float(value) for value in dds_joint_snapshot.joint_positions],
        "joint_velocities": [float(value) for value in dds_joint_snapshot.joint_velocities],
        "joint_efforts": [float(value) for value in (dds_joint_snapshot.joint_efforts or [])],
        "imu_quaternion_wxyz": [float(value) for value in snapshot.base_quaternion_wxyz],
        "imu_accelerometer": [float(value) for value in snapshot.imu_linear_acceleration_body],
        "imu_gyroscope": [float(value) for value in snapshot.imu_angular_velocity_body],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_lowstate_packet(packet: bytes) -> dict[str, Any]:
    """Parse a lowstate localhost packet."""
    return json.loads(packet.decode("utf-8"))


def encode_lowcmd_packet(
    *,
    mode_pr: int,
    mode_machine: int,
    positions: list[float],
    velocities: list[float],
    torques: list[float],
    kp: list[float],
    kd: list[float],
) -> bytes:
    """Serialize lowcmd fields from the ROS 2 sidecar to Isaac Sim."""
    payload = {
        "mode_pr": int(mode_pr),
        "mode_machine": int(mode_machine),
        "joint_positions_dds": [float(value) for value in positions],
        "joint_velocities_dds": [float(value) for value in velocities],
        "joint_torques_dds": [float(value) for value in torques],
        "joint_kp_dds": [float(value) for value in kp],
        "joint_kd_dds": [float(value) for value in kd],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_lowcmd_packet(packet: bytes) -> dict[str, Any]:
    """Parse a lowcmd localhost packet."""
    return json.loads(packet.decode("utf-8"))
