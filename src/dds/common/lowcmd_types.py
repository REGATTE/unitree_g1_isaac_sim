"""Shared low-level command types used across DDS ingress transports."""

from __future__ import annotations

from dataclasses import dataclass

from mapping import reorder_dds_values_to_sim


@dataclass(frozen=True)
class SimOrderLowCmd:
    """Body-joint command vectors in simulator joint order."""

    positions: list[float]
    velocities: list[float]
    torques: list[float]
    kp: list[float]
    kd: list[float]


@dataclass(frozen=True)
class LowCmdCache:
    """Validated raw body command cache from an external lowcmd boundary."""

    mode_pr: int
    mode_machine: int
    joint_positions_dds: tuple[float, ...]
    joint_velocities_dds: tuple[float, ...]
    joint_torques_dds: tuple[float, ...]
    joint_kp_dds: tuple[float, ...]
    joint_kd_dds: tuple[float, ...]
    received_at_monotonic: float

    def to_sim_order(self) -> SimOrderLowCmd:
        """Return body-joint command vectors in simulator joint order."""
        return SimOrderLowCmd(
            positions=reorder_dds_values_to_sim(self.joint_positions_dds),
            velocities=reorder_dds_values_to_sim(self.joint_velocities_dds),
            torques=reorder_dds_values_to_sim(self.joint_torques_dds),
            kp=reorder_dds_values_to_sim(self.joint_kp_dds),
            kd=reorder_dds_values_to_sim(self.joint_kd_dds),
        )
