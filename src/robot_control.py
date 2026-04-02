"""Low-level body-command application for the Unitree G1 simulator.

This module owns the simulator side of the `rt/lowcmd` path. The DDS layer is
responsible for validating and caching raw Unitree messages; this module takes
the latest cached command, remaps it back into simulator joint order, and
applies the supported fields to the articulation.

The current control slice stays intentionally conservative: body joint position
targets remain the primary surface, while velocity, effort, and PD gains are
forwarded when the articulation wrapper exposes the corresponding setters.
"""

from __future__ import annotations

from dataclasses import dataclass

from dds.g1_lowcmd import LowCmdCache
from robot_state import PhysicsViewUnavailableError, RobotStateReader


@dataclass(frozen=True)
class LowCmdApplyResult:
    """Result of one attempt to apply a cached `rt/lowcmd` sample."""

    command_seen: bool
    position_applied: bool
    velocity_applied: bool
    effort_applied: bool
    gains_applied: bool


class RobotCommandApplier:
    """Apply cached DDS body commands to the live G1 articulation."""

    def __init__(self, state_reader: RobotStateReader) -> None:
        self._state_reader = state_reader
        self._warned_about_gains = False

    def apply_lowcmd(self, lowcmd: LowCmdCache | None) -> LowCmdApplyResult:
        """Apply the latest cached low-level DDS command if one exists.

        The incoming DDS message is already CRC-validated by the subscriber.
        This method only handles simulator-order translation plus articulation
        writes. When no command has been received yet, it returns a no-op
        result so the main loop can call it unconditionally.
        """
        if lowcmd is None:
            return LowCmdApplyResult(
                command_seen=False,
                position_applied=False,
                velocity_applied=False,
                effort_applied=False,
                gains_applied=False,
            )

        sim_order_command = lowcmd.to_sim_order()
        try:
            position_applied = self._state_reader.apply_joint_position_targets(sim_order_command.positions)
            velocity_applied = self._state_reader.apply_joint_velocity_targets(sim_order_command.velocities)
            effort_applied = self._state_reader.apply_joint_efforts(sim_order_command.torques)
            gains_applied = self._state_reader.apply_joint_gains(
                sim_order_command.kp,
                sim_order_command.kd,
            )
        except PhysicsViewUnavailableError:
            return LowCmdApplyResult(
                command_seen=True,
                position_applied=False,
                velocity_applied=False,
                effort_applied=False,
                gains_applied=False,
            )

        self._warn_if_gains_are_not_applied(lowcmd, gains_applied)
        return LowCmdApplyResult(
            command_seen=True,
            position_applied=position_applied,
            velocity_applied=velocity_applied,
            effort_applied=effort_applied,
            gains_applied=gains_applied,
        )

    def _warn_if_gains_are_not_applied(self, lowcmd: LowCmdCache, gains_applied: bool) -> None:
        """Emit a one-time note about `kp`/`kd` support gaps.

        The command cache preserves the gain values because they are part of
        the external Unitree contract. We do not silently discard that fact.
        Instead, we keep the current implementation explicit until a stable
        Isaac Sim runtime API for dynamic per-joint gain updates is selected
        for this project.
        """
        if gains_applied:
            return
        if self._warned_about_gains:
            return
        if any(value != 0.0 for value in lowcmd.joint_kp_dds) or any(value != 0.0 for value in lowcmd.joint_kd_dds):
            print(
                "[unitree_g1_isaac_sim] received lowcmd gains (`kp`/`kd`), but dynamic gain "
                "application is not implemented yet; applying position/velocity/effort fields only."
            )
            self._warned_about_gains = True
