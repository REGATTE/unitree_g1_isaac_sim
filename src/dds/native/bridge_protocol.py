"""Localhost packet protocol between Isaac Sim and the native SDK bridge."""

from __future__ import annotations

import json
from typing import Any

from mapping import to_dds_ordered_snapshot
from robot_state import JointStateSnapshot, RobotKinematicSnapshot
from sensors.torso_imu import quaternion_wxyz_to_rpy


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
        "imu_quaternion_wxyz": [float(value) for value in snapshot.imu_quaternion_wxyz],
        "imu_accelerometer_body": [float(value) for value in snapshot.imu_linear_acceleration_body],
        "imu_gyroscope_body": [float(value) for value in snapshot.imu_angular_velocity_body],
        "imu_rpy": [float(value) for value in quaternion_wxyz_to_rpy(snapshot.imu_quaternion_wxyz)],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_native_lowstate_packet(packet: bytes) -> dict[str, Any]:
    """Parse a native lowstate localhost packet."""
    return json.loads(packet.decode("utf-8"))


def encode_native_secondary_imu_packet(snapshot: RobotKinematicSnapshot) -> bytes:
    """Serialize the torso/secondary IMU sample for SDK2 Python sidecar use."""
    payload = {
        "quaternion_wxyz": [float(value) for value in snapshot.secondary_imu_quaternion_wxyz],
        "accelerometer_body": [float(value) for value in snapshot.secondary_imu_linear_acceleration_body],
        "gyroscope_body": [float(value) for value in snapshot.secondary_imu_angular_velocity_body],
        "rpy": [float(value) for value in quaternion_wxyz_to_rpy(snapshot.secondary_imu_quaternion_wxyz)],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_native_secondary_imu_packet(packet: bytes) -> dict[str, Any]:
    """Parse a native secondary-imu localhost packet."""
    return json.loads(packet.decode("utf-8"))


def decode_native_lowcmd_packet(packet: bytes) -> dict[str, Any]:
    """Parse a native lowcmd localhost packet."""
    return json.loads(packet.decode("utf-8"))


def encode_native_lowcmd_packet(
    *,
    mode_pr: int,
    mode_machine: int,
    joint_positions_dds: list[float],
    joint_velocities_dds: list[float],
    joint_torques_dds: list[float],
    joint_kp_dds: list[float],
    joint_kd_dds: list[float],
) -> bytes:
    """Serialize a lowcmd packet for a Unitree SDK sidecar."""
    payload = {
        "mode_pr": int(mode_pr),
        "mode_machine": int(mode_machine),
        "joint_positions_dds": [float(value) for value in joint_positions_dds],
        "joint_velocities_dds": [float(value) for value in joint_velocities_dds],
        "joint_torques_dds": [float(value) for value in joint_torques_dds],
        "joint_kp_dds": [float(value) for value in joint_kp_dds],
        "joint_kd_dds": [float(value) for value in joint_kd_dds],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")
