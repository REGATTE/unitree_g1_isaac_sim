"""`rt/lowcmd` subscription boundary for the Unitree G1 simulator.

This module owns the raw DDS ingest side of the body low-level command path.
It validates incoming Unitree messages, clamps the message width to the
supported 29 body joints, and exposes a cached simulator-agnostic command
snapshot that the runtime loop can translate into Isaac Sim control writes.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
from threading import Lock
import time
from typing import Any

from mapping.joints import BODY_JOINT_COUNT
from mapping import reorder_dds_values_to_sim

LOGGER = logging.getLogger("unitree_g1_isaac_sim.dds.lowcmd")


@dataclass(frozen=True)
class LowCmdCache:
    """Validated raw body command cache from the DDS boundary."""

    mode_pr: int
    mode_machine: int
    joint_positions_dds: tuple[float, ...]
    joint_velocities_dds: tuple[float, ...]
    joint_torques_dds: tuple[float, ...]
    joint_kp_dds: tuple[float, ...]
    joint_kd_dds: tuple[float, ...]
    received_at_monotonic: float

    def to_sim_order(self) -> dict[str, list[float]]:
        """Return body-joint command vectors in simulator joint order.

        Keeping the DDS-to-simulator remap here ensures the rest of the runtime
        only handles simulator-order control vectors.
        """
        return {
            "positions": reorder_dds_values_to_sim(self.joint_positions_dds),
            "velocities": reorder_dds_values_to_sim(self.joint_velocities_dds),
            "torques": reorder_dds_values_to_sim(self.joint_torques_dds),
            "kp": reorder_dds_values_to_sim(self.joint_kp_dds),
            "kd": reorder_dds_values_to_sim(self.joint_kd_dds),
        }


class G1LowCmdSubscriber:
    """Receive raw `rt/lowcmd` messages and cache the latest validated sample."""

    def __init__(self, topic_name: str = "rt/lowcmd") -> None:
        self._topic_name = topic_name
        self._subscriber: Any | None = None
        self._crc_helper: Any | None = None
        self._latest_command: LowCmdCache | None = None
        self._sdk_enabled = False
        self._warned_unavailable = False
        self._warned_extra_widths: set[int] = set()
        self._warned_short_widths: set[int] = set()
        self._warning_state_lock = Lock()

    @property
    def latest_command(self) -> LowCmdCache | None:
        return self._latest_command

    def initialize(self) -> bool:
        """Create the Unitree subscriber if the SDK is installed."""
        if self._sdk_enabled:
            return True
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError:
            if not self._warned_unavailable:
                LOGGER.warning(
                    "[unitree_g1_isaac_sim] DDS lowcmd subscriber requested but "
                    "`unitree_sdk2py` is not installed. Skipping subscription."
                )
                self._warned_unavailable = True
            return False

        self._subscriber = ChannelSubscriber(self._topic_name, LowCmd_)
        self._subscriber.Init(self._on_message, 32)
        self._crc_helper = CRC()
        self._sdk_enabled = True
        LOGGER.info("[unitree_g1_isaac_sim] DDS lowcmd subscriber ready on %s", self._topic_name)
        return True

    def _on_message(self, msg: Any) -> None:
        """Validate and cache one incoming `LowCmd_` message."""
        if self._crc_helper is None:
            return
        if self._crc_helper.Crc(msg) != msg.crc:
            LOGGER.warning("[unitree_g1_isaac_sim] dropped `rt/lowcmd` message with invalid CRC")
            return

        motor_cmd = msg.motor_cmd
        incoming_count = len(motor_cmd)
        if incoming_count < BODY_JOINT_COUNT:
            self._warn_short_message(incoming_count)
            return
        if incoming_count > BODY_JOINT_COUNT:
            self._warn_extra_message_fields(incoming_count)

        count = BODY_JOINT_COUNT
        self._latest_command = LowCmdCache(
            mode_pr=int(msg.mode_pr),
            mode_machine=int(msg.mode_machine),
            joint_positions_dds=tuple(float(motor_cmd[index].q) for index in range(count)),
            joint_velocities_dds=tuple(float(motor_cmd[index].dq) for index in range(count)),
            joint_torques_dds=tuple(float(motor_cmd[index].tau) for index in range(count)),
            joint_kp_dds=tuple(float(motor_cmd[index].kp) for index in range(count)),
            joint_kd_dds=tuple(float(motor_cmd[index].kd) for index in range(count)),
            received_at_monotonic=time.monotonic(),
        )

    def _warn_extra_message_fields(self, incoming_count: int) -> None:
        """Log once per width when incoming DDS commands expose extra slots."""
        with self._warning_state_lock:
            if incoming_count in self._warned_extra_widths:
                return
            self._warned_extra_widths.add(incoming_count)
        LOGGER.warning(
            "[unitree_g1_isaac_sim] received `rt/lowcmd` with %s motor slots; "
            "consuming only the first %s G1 body-joint commands and ignoring the extra entries.",
            incoming_count,
            BODY_JOINT_COUNT,
        )

    def _warn_short_message(self, incoming_count: int) -> None:
        """Log once per width when incoming DDS commands are too short to use."""
        with self._warning_state_lock:
            if incoming_count in self._warned_short_widths:
                return
            self._warned_short_widths.add(incoming_count)
        LOGGER.warning(
            "[unitree_g1_isaac_sim] dropped `rt/lowcmd` message with %s motor slots; "
            "expected at least %s body-joint commands.",
            incoming_count,
            BODY_JOINT_COUNT,
        )
