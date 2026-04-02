"""Shared DDS joint-name helpers for standalone tooling."""

from __future__ import annotations

from typing import Protocol

from mapping.joints import DDS_G1_29DOF_JOINT_NAMES


class JointHistorySample(Protocol):
    """Small protocol for tooling samples that expose joint-aligned vectors."""

    positions: list[float]
    velocities: list[float]
    torques: list[float]


def resolve_joint_index(joint_name: str) -> int:
    """Resolve a DDS-order body joint name to its index."""
    try:
        return DDS_G1_29DOF_JOINT_NAMES.index(joint_name)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported DDS joint name `{joint_name}`. "
            f"Expected one of: {DDS_G1_29DOF_JOINT_NAMES}"
        ) from exc


def filter_joint_history_samples(
    history: list[JointHistorySample],
    joint_index: int,
) -> list[JointHistorySample]:
    """Keep only samples that contain the requested joint in all tracked fields."""
    return [
        sample
        for sample in history
        if joint_index < len(sample.positions)
        and joint_index < len(sample.velocities)
        and joint_index < len(sample.torques)
    ]
