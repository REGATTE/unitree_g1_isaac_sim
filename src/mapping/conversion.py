"""Pure helpers for converting joint-aligned data between sim and DDS order.

The rest of the codebase should read raw articulation state in simulator order
and use these helpers whenever data must cross the DDS boundary.
"""

from __future__ import annotations

from typing import Sequence, TypeVar

from robot_state import JointStateSnapshot

from .joints import G1_29DOF_JOINT_MAPPING, JointOrderMapping

T = TypeVar("T")


def reorder_sim_values_to_dds(
    values: Sequence[T],
    mapping: JointOrderMapping = G1_29DOF_JOINT_MAPPING,
) -> list[T]:
    """Reorder simulator-aligned values into the DDS joint order."""
    _validate_value_count(values, len(mapping.sim_joint_names), "sim")
    return [values[sim_index] for sim_index in mapping.dds_to_sim_index]


def reorder_dds_values_to_sim(
    values: Sequence[T],
    mapping: JointOrderMapping = G1_29DOF_JOINT_MAPPING,
) -> list[T]:
    """Reorder DDS-aligned values back into the simulator joint order."""
    _validate_value_count(values, len(mapping.dds_joint_names), "dds")
    return [values[dds_index] for dds_index in mapping.sim_to_dds_index]


def to_dds_ordered_snapshot(
    snapshot: JointStateSnapshot,
    mapping: JointOrderMapping = G1_29DOF_JOINT_MAPPING,
) -> JointStateSnapshot:
    """Build a DDS-ordered view of a raw simulator joint-state snapshot.

    This helper is intended to be safe as a standalone conversion boundary.
    Callers should be able to hand it a live simulator snapshot without
    remembering to validate joint order separately first.
    """
    # Refuse to relabel state unless the snapshot still matches the frozen
    # simulator order that the mapping tables were built against.
    _validate_snapshot_joint_names(snapshot, mapping)
    return JointStateSnapshot(
        joint_names=list(mapping.dds_joint_names),
        joint_positions=reorder_sim_values_to_dds(snapshot.joint_positions, mapping),
        joint_velocities=reorder_sim_values_to_dds(snapshot.joint_velocities, mapping),
        joint_efforts=(
            reorder_sim_values_to_dds(snapshot.joint_efforts, mapping)
            if snapshot.joint_efforts is not None
            else None
        ),
    )


def _validate_snapshot_joint_names(
    snapshot: JointStateSnapshot,
    mapping: JointOrderMapping,
) -> None:
    """Fail fast if a caller passes a snapshot with the wrong sim joint order."""
    live_joint_names = tuple(snapshot.joint_names)
    if live_joint_names != mapping.sim_joint_names:
        raise ValueError(
            "Snapshot joint_names do not match the frozen simulator joint order. "
            f"Expected {list(mapping.sim_joint_names)}, got {list(live_joint_names)}"
        )


def _validate_value_count(values: Sequence[T], expected_count: int, label: str) -> None:
    """Fail early if a joint-aligned vector does not match the expected width."""
    if len(values) != expected_count:
        raise ValueError(f"Expected {expected_count} {label}-ordered values, got {len(values)}")
