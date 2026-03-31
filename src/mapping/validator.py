"""Validation helpers for G1 joint-order compatibility.

The goal of this module is to fail early if the live articulation order
drifts from the frozen simulator order that the DDS mapping layer assumes.
"""

from __future__ import annotations

from dataclasses import dataclass

from .joints import G1_29DOF_JOINT_MAPPING, JointOrderMapping


@dataclass(frozen=True)
class JointValidationReport:
    """Human-readable summary of a joint validation pass."""

    joint_count: int
    sim_joint_order_matches: bool
    dds_joint_count: int


def validate_live_joint_order(
    live_joint_names: list[str] | tuple[str, ...],
    mapping: JointOrderMapping = G1_29DOF_JOINT_MAPPING,
) -> JointValidationReport:
    """Validate the live articulation order against the frozen mapping.

    This checks two things:
    1. The live articulation exposes the same 29-joint set as the mapping.
    2. The live articulation order exactly matches the frozen simulator order.

    If either assumption breaks, startup should fail before DDS logic uses
    the wrong joint indices silently.
    """
    live_joint_names = tuple(live_joint_names)

    if len(live_joint_names) != len(mapping.sim_joint_names):
        raise ValueError(
            f"Expected {len(mapping.sim_joint_names)} live joints, got {len(live_joint_names)}: "
            f"{list(live_joint_names)}"
        )

    live_joint_set = set(live_joint_names)
    sim_joint_set = set(mapping.sim_joint_names)
    if live_joint_set != sim_joint_set:
        missing_in_live = sorted(sim_joint_set - live_joint_set)
        unexpected_in_live = sorted(live_joint_set - sim_joint_set)
        raise ValueError(
            "Live articulation joint set does not match the frozen simulator mapping. "
            f"Missing in live: {missing_in_live}; unexpected in live: {unexpected_in_live}"
        )

    sim_joint_order_matches = live_joint_names == mapping.sim_joint_names
    if not sim_joint_order_matches:
        mismatches: list[str] = []
        for index, (expected, actual) in enumerate(zip(mapping.sim_joint_names, live_joint_names)):
            if expected != actual:
                mismatches.append(f"{index}: expected {expected}, got {actual}")
        raise ValueError(
            "Live articulation joint order does not match the frozen simulator order. "
            + "; ".join(mismatches)
        )

    return JointValidationReport(
        joint_count=len(live_joint_names),
        sim_joint_order_matches=sim_joint_order_matches,
        dds_joint_count=len(mapping.dds_joint_names),
    )


def log_joint_validation_report(report: JointValidationReport) -> None:
    """Print a short startup summary once the joint order is validated."""
    print(
        "[unitree_g1_isaac_sim] joint mapping validated: "
        f"{report.joint_count} live joints, "
        f"{report.dds_joint_count} DDS joints, "
        f"sim_order_match={report.sim_joint_order_matches}"
    )
