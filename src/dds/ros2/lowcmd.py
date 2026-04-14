"""Local ROS 2 lowcmd subscription boundary between the sidecar and Isaac Sim."""

from __future__ import annotations

import errno
from functools import lru_cache
import socket
import time

from mapping.joints import BODY_JOINT_COUNT
from runtime_logging import get_logger
from .bridge_protocol import decode_lowcmd_packet
from dds.common.lowcmd_types import LowCmdCache

LOGGER = get_logger("dds.lowcmd")


class G1LowCmdSubscriber:
    """Receive lowcmd packets from the localhost sidecar and cache the latest sample."""

    def __init__(self, bind_host: str, bind_port: int) -> None:
        self._bind_host = bind_host
        self._bind_port = bind_port
        self._socket: socket.socket | None = None
        self._latest_command: LowCmdCache | None = None
        self._transport_ready = False

    @property
    def latest_command(self) -> LowCmdCache | None:
        return self._latest_command

    def clear_cached_command(self) -> None:
        """Discard any previously received lowcmd sample."""
        self._latest_command = None

    @property
    def bound_port(self) -> int:
        """Return the currently bound UDP port, if initialized."""
        if self._socket is None:
            return self._bind_port
        return int(self._socket.getsockname()[1])

    def initialize(self) -> bool:
        """Bind the localhost UDP socket used by the sidecar bridge."""
        if self._transport_ready:
            return True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._socket.bind((self._bind_host, self._bind_port))
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE or self._bind_port == 0:
                self._socket.close()
                self._socket = None
                raise
            LOGGER.warning(
                "lowcmd UDP port %s is already in use on %s; falling back to an ephemeral localhost port.",
                self._bind_port,
                self._bind_host,
            )
            self._socket.bind((self._bind_host, 0))
        self._socket.setblocking(False)
        self._transport_ready = True
        LOGGER.info("lowcmd UDP subscriber ready on %s:%s", self._bind_host, self.bound_port)
        return True

    def close(self) -> None:
        """Release the localhost UDP socket."""
        if self._socket is not None:
            self._socket.close()
        self._socket = None
        self._transport_ready = False

    def poll(self) -> None:
        """Consume any queued lowcmd packets from the sidecar."""
        if not self.initialize() or self._socket is None:
            return
        while True:
            try:
                packet, _address = self._socket.recvfrom(65535)
            except BlockingIOError:
                return
            self._on_packet(packet, time.monotonic())

    def _on_packet(self, packet: bytes, received_at_monotonic: float) -> None:
        """Cache one incoming localhost lowcmd packet."""
        payload = decode_lowcmd_packet(packet)
        positions = payload.get("joint_positions_dds", [])
        velocities = payload.get("joint_velocities_dds", [])
        torques = payload.get("joint_torques_dds", [])
        kp_values = payload.get("joint_kp_dds", [])
        kd_values = payload.get("joint_kd_dds", [])

        count = self._effective_motor_count(positions)
        if count is None:
            return
        self._latest_command = LowCmdCache(
            mode_pr=int(payload.get("mode_pr", 0)),
            mode_machine=int(payload.get("mode_machine", 0)),
            joint_positions_dds=tuple(float(positions[index]) for index in range(count)),
            joint_velocities_dds=tuple(float(velocities[index]) for index in range(count)),
            joint_torques_dds=tuple(float(torques[index]) for index in range(count)),
            joint_kp_dds=tuple(float(kp_values[index]) for index in range(count)),
            joint_kd_dds=tuple(float(kd_values[index]) for index in range(count)),
            received_at_monotonic=received_at_monotonic,
        )

    def _effective_motor_count(self, motor_cmd: list[float] | tuple[float, ...]) -> int | None:
        incoming_count = len(motor_cmd)
        if incoming_count < BODY_JOINT_COUNT:
            self._warn_short_message(incoming_count)
            return None
        if incoming_count > BODY_JOINT_COUNT:
            self._warn_extra_message_fields(incoming_count)
        return BODY_JOINT_COUNT

    @staticmethod
    @lru_cache(maxsize=None)
    def _warn_extra_message_fields(incoming_count: int) -> None:
        """Log once per width when incoming DDS commands expose extra slots."""
        LOGGER.warning(
            "received `rt/lowcmd` with %s motor slots; "
            "consuming only the first %s G1 body-joint commands and ignoring the extra entries.",
            incoming_count,
            BODY_JOINT_COUNT,
        )

    @staticmethod
    @lru_cache(maxsize=None)
    def _warn_short_message(incoming_count: int) -> None:
        """Log once per width when incoming DDS commands are too short to use."""
        LOGGER.warning(
            "dropped `rt/lowcmd` message with %s motor slots; "
            "expected at least %s body-joint commands.",
            incoming_count,
            BODY_JOINT_COUNT,
        )
