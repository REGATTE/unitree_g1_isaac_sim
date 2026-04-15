"""Isaac-side UDP secondary-IMU publisher for the Unitree SDK2 Python bridge."""

from __future__ import annotations

import socket

from dds.native.bridge_protocol import encode_native_secondary_imu_packet
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger


LOGGER = get_logger("dds.sdk2py_secondary_imu")


class Sdk2PySecondaryImuUdpPublisher:
    """Send torso IMU packets from Isaac Sim to the SDK2 Python sidecar bridge."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._socket: socket.socket | None = None

    def initialize(self) -> bool:
        if self._socket is not None:
            return True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LOGGER.info("SDK2 Python secondary IMU UDP publisher ready for %s:%s", self._host, self._port)
        return True

    def publish(self, snapshot: RobotKinematicSnapshot) -> bool:
        if not self.initialize() or self._socket is None:
            return False
        self._socket.sendto(encode_native_secondary_imu_packet(snapshot), (self._host, self._port))
        return True

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
        self._socket = None
