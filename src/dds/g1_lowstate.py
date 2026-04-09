"""Local lowstate publication boundary between Isaac Sim and the sidecar bridge."""

from __future__ import annotations

from dataclasses import dataclass
import socket

from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger
from .bridge_protocol import encode_lowstate_packet


LOGGER = get_logger("dds.lowstate")


@dataclass(frozen=True)
class LowStatePublishStats:
    """Small runtime counters for DDS state publication."""

    published_messages: int = 0
    skipped_messages: int = 0


class G1LowStatePublisher:
    """Send lowstate packets from Isaac Sim to the ROS 2 sidecar bridge."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket: socket.socket | None = None
        self._stats = LowStatePublishStats()

    @property
    def stats(self) -> LowStatePublishStats:
        return self._stats

    def initialize(self) -> bool:
        """Create the localhost UDP socket used to reach the sidecar."""
        if self._socket is not None:
            return True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LOGGER.info("lowstate UDP publisher ready for %s:%s", self._host, self._port)
        return True

    def publish(self, snapshot: RobotKinematicSnapshot) -> bool:
        """Publish one simulator state sample to the sidecar bridge."""
        if not self.initialize() or self._socket is None:
            self._stats = LowStatePublishStats(
                published_messages=self._stats.published_messages,
                skipped_messages=self._stats.skipped_messages + 1,
            )
            return False

        packet = encode_lowstate_packet(snapshot, tick=self._stats.published_messages + 1)
        self._socket.sendto(packet, (self._host, self._port))
        self._stats = LowStatePublishStats(
            published_messages=self._stats.published_messages + 1,
            skipped_messages=self._stats.skipped_messages,
        )
        return True
