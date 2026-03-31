"""Joint-order definitions and index mapping for the Unitree G1 29-DoF body.

This module is the first compatibility layer between Isaac Sim and the
external Unitree DDS contract. It freezes two orders:

1. The articulation order reported by the Isaac Sim G1 USD used here.
2. The 29-joint body order used by the Isaac Lab DDS reference.

All DDS state publication and low-level command ingestion should route
through these explicit maps instead of relying on implicit joint order.
"""

from __future__ import annotations

from dataclasses import dataclass


SIM_G1_29DOF_JOINT_NAMES: list[str] = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]


# This order is taken from the Isaac Lab reference:
# tasks/common_observations/g1_29dof_state.py:get_robot_boy_joint_names()
DDS_G1_29DOF_JOINT_NAMES: list[str] = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


BODY_JOINT_COUNT = 29
LEG_AND_WAIST_JOINT_COUNT = 15
ARM_JOINT_COUNT = 14
DDS_ARM_JOINT_START = LEG_AND_WAIST_JOINT_COUNT


@dataclass(frozen=True)
class JointOrderMapping:
    """Precomputed lookup tables between simulator order and DDS order."""

    sim_joint_names: tuple[str, ...]
    dds_joint_names: tuple[str, ...]
    sim_name_to_index: dict[str, int]
    dds_name_to_index: dict[str, int]
    sim_to_dds_index: tuple[int, ...]
    dds_to_sim_index: tuple[int, ...]


def build_joint_order_mapping(
    sim_joint_names: list[str] | tuple[str, ...] = SIM_G1_29DOF_JOINT_NAMES,
    dds_joint_names: list[str] | tuple[str, ...] = DDS_G1_29DOF_JOINT_NAMES,
) -> JointOrderMapping:
    """Build and validate explicit maps between sim order and DDS order."""
    sim_joint_names = tuple(sim_joint_names)
    dds_joint_names = tuple(dds_joint_names)

    _validate_joint_name_list(sim_joint_names, "sim_joint_names")
    _validate_joint_name_list(dds_joint_names, "dds_joint_names")

    sim_name_to_index = {name: index for index, name in enumerate(sim_joint_names)}
    dds_name_to_index = {name: index for index, name in enumerate(dds_joint_names)}

    sim_name_set = set(sim_joint_names)
    dds_name_set = set(dds_joint_names)
    if sim_name_set != dds_name_set:
        missing_in_sim = sorted(dds_name_set - sim_name_set)
        missing_in_dds = sorted(sim_name_set - dds_name_set)
        raise ValueError(
            "Simulator and DDS joint names do not match. "
            f"Missing in sim: {missing_in_sim}; missing in DDS: {missing_in_dds}"
        )

    sim_to_dds_index = tuple(dds_name_to_index[name] for name in sim_joint_names)
    dds_to_sim_index = tuple(sim_name_to_index[name] for name in dds_joint_names)

    return JointOrderMapping(
        sim_joint_names=sim_joint_names,
        dds_joint_names=dds_joint_names,
        sim_name_to_index=sim_name_to_index,
        dds_name_to_index=dds_name_to_index,
        sim_to_dds_index=sim_to_dds_index,
        dds_to_sim_index=dds_to_sim_index,
    )


def _validate_joint_name_list(joint_names: tuple[str, ...], label: str) -> None:
    """Check basic assumptions before maps are built."""
    if len(joint_names) != BODY_JOINT_COUNT:
        raise ValueError(f"{label} must contain {BODY_JOINT_COUNT} joints, got {len(joint_names)}")
    if len(set(joint_names)) != len(joint_names):
        duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
        raise ValueError(f"{label} contains duplicate joints: {duplicates}")


G1_29DOF_JOINT_MAPPING = build_joint_order_mapping()
