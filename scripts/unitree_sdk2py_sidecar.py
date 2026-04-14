#!/usr/bin/env python3
"""Unitree SDK2 Python sidecar bridge for the Unitree G1 simulator."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
LOCAL_SDK2PY_ROOT = Path.home() / "unitree_sdk2_python"
for import_root in (SRC_ROOT, LOCAL_SDK2PY_ROOT):
    if import_root.exists() and str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from dds.native.bridge_protocol import decode_native_lowstate_packet


BODY_JOINT_COUNT = 29
TOTAL_MOTOR_SLOTS = 35


def _load_sdk2py_modules():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC

    return ChannelFactoryInitialize, ChannelPublisher, LowState_, unitree_hg_msg_dds__LowState_, CRC


class UnitreeSdk2PySidecar:
    """Publish Unitree HG LowState_ messages from localhost simulator packets."""

    def __init__(
        self,
        *,
        domain_id: int,
        lowstate_topic: str,
        network_interface: str,
        bind_host: str,
        lowstate_port: int,
    ) -> None:
        (
            channel_factory_initialize,
            channel_publisher,
            lowstate_type,
            lowstate_factory,
            crc_type,
        ) = _load_sdk2py_modules()

        interface = network_interface or None
        if interface and interface.lower() == "none":
            interface = None
        channel_factory_initialize(domain_id, interface)
        self._lowstate_publisher = channel_publisher(lowstate_topic, lowstate_type)
        self._lowstate_publisher.Init()
        self._lowstate_factory = lowstate_factory
        self._crc = crc_type()

        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_socket.bind((bind_host, lowstate_port))
        self._recv_socket.setblocking(False)
        print(
            "SDK2 Python lowstate publisher initialized "
            f"(domain_id={domain_id}, topic={lowstate_topic}, "
            f"network_interface={interface}, udp_host={bind_host}, udp_port={lowstate_port})",
            file=sys.stderr,
            flush=True,
        )

    def poll_once(self) -> None:
        while True:
            try:
                packet, _addr = self._recv_socket.recvfrom(65535)
            except BlockingIOError:
                return
            self._publish_lowstate(packet)

    def shutdown(self) -> None:
        self._recv_socket.close()
        self._lowstate_publisher.Close()

    def _publish_lowstate(self, packet: bytes) -> None:
        payload = decode_native_lowstate_packet(packet)
        message = build_lowstate_message(payload, self._lowstate_factory, self._crc)
        self._lowstate_publisher.Write(message)


def build_lowstate_message(payload, lowstate_factory, crc):
    """Build a Unitree HG LowState_ message from a decoded simulator packet."""
    message = lowstate_factory()
    message.version = [1, 0]
    message.mode_pr = 0
    message.mode_machine = 0
    message.tick = int(payload.get("tick", 0))

    message.imu_state.quaternion = _fixed_float_list(payload.get("imu_quaternion_wxyz"), 4)
    message.imu_state.gyroscope = _fixed_float_list(payload.get("imu_gyroscope_body"), 3)
    message.imu_state.accelerometer = _fixed_float_list(payload.get("imu_accelerometer_body"), 3)
    message.imu_state.rpy = [0.0, 0.0, 0.0]
    message.imu_state.temperature = 0

    positions = _fixed_float_list(payload.get("joint_positions_dds"), BODY_JOINT_COUNT)
    velocities = _fixed_float_list(payload.get("joint_velocities_dds"), BODY_JOINT_COUNT)
    efforts = _fixed_float_list(payload.get("joint_efforts_dds"), BODY_JOINT_COUNT)
    for index in range(min(BODY_JOINT_COUNT, len(message.motor_state))):
        motor = message.motor_state[index]
        motor.mode = 0
        motor.q = positions[index]
        motor.dq = velocities[index]
        motor.ddq = 0.0
        motor.tau_est = efforts[index]
        motor.temperature = [0, 0]
        motor.vol = 0.0
        motor.sensor = [0, 0]
        motor.motorstate = 0
        motor.reserve = [0, 0, 0, 0]
    for index in range(BODY_JOINT_COUNT, min(TOTAL_MOTOR_SLOTS, len(message.motor_state))):
        motor = message.motor_state[index]
        motor.mode = 0
        motor.q = 0.0
        motor.dq = 0.0
        motor.ddq = 0.0
        motor.tau_est = 0.0
        motor.temperature = [0, 0]
        motor.vol = 0.0
        motor.sensor = [0, 0]
        motor.motorstate = 0
        motor.reserve = [0, 0, 0, 0]

    message.wireless_remote = [0 for _ in range(40)]
    message.reserve = [0, 0, 0, 0]
    message.crc = 0
    message.crc = crc.Crc(message)
    return message


def _fixed_float_list(values, width: int) -> list[float]:
    result = [0.0 for _ in range(width)]
    if values is None:
        return result
    for index, value in enumerate(values[:width]):
        result[index] = float(value)
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the Unitree SDK2 Python sidecar bridge.")
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--lowstate-topic", default="rt/lowstate")
    parser.add_argument("--network-interface", default="lo")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--lowstate-port", type=int, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    sidecar = UnitreeSdk2PySidecar(
        domain_id=args.domain_id,
        lowstate_topic=args.lowstate_topic,
        network_interface=args.network_interface,
        bind_host=args.bind_host,
        lowstate_port=args.lowstate_port,
    )
    try:
        while True:
            sidecar.poll_once()
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        sidecar.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
