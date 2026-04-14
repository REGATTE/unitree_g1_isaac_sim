"""Isaac-side UDP lowstate publisher for the native Unitree SDK bridge."""

from __future__ import annotations

from dataclasses import dataclass
import socket

from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger

from .bridge_protocol import encode_native_lowstate_packet


LOGGER = get_logger("dds.native_lowstate")


@dataclass(frozen=True)
class NativeLowStatePublishStats:
    """Small runtime counters for native lowstate publication."""

    published_messages: int = 0
    skipped_messages: int = 0


class NativeLowStateUdpPublisher:
    """Send lowstate packets from Isaac Sim to the native C++ sidecar bridge."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket: socket.socket | None = None
        self._stats = NativeLowStatePublishStats()

    @property
    def stats(self) -> NativeLowStatePublishStats:
        return self._stats

    def initialize(self) -> bool:
        if self._socket is not None:
            return True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LOGGER.info("native lowstate UDP publisher ready for %s:%s", self._host, self._port)
        return True

    def publish(self, snapshot: RobotKinematicSnapshot) -> bool:
        if not self.initialize() or self._socket is None:
            self._stats = NativeLowStatePublishStats(
                published_messages=self._stats.published_messages,
                skipped_messages=self._stats.skipped_messages + 1,
            )
            return False
        packet = encode_native_lowstate_packet(snapshot, tick=self._stats.published_messages + 1)
        self._socket.sendto(packet, (self._host, self._port))
        self._stats = NativeLowStatePublishStats(
            published_messages=self._stats.published_messages + 1,
            skipped_messages=self._stats.skipped_messages,
        )
        return True

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
        self._socket = None
