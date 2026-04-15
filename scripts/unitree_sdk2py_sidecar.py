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

from dds.native.bridge_protocol import (
    decode_native_lowstate_packet,
    decode_native_secondary_imu_packet,
    encode_native_lowcmd_packet,
)


BODY_JOINT_COUNT = 29
TOTAL_MOTOR_SLOTS = 35


def _load_sdk2py_modules():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    return (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
        LowState_,
        LowCmd_,
        IMUState_,
        unitree_hg_msg_dds__LowState_,
        CRC,
    )


class UnitreeSdk2PySidecar:
    """Publish Unitree HG LowState_ messages from localhost simulator packets."""

    def __init__(
        self,
        *,
        domain_id: int,
        lowstate_topic: str,
        lowcmd_topic: str,
        network_interface: str,
        bind_host: str,
        lowstate_port: int,
        lowcmd_port: int,
        enable_lowstate: bool,
        enable_lowcmd: bool,
        enable_secondary_imu: bool,
        secondary_imu_topic: str,
        secondary_imu_port: int,
    ) -> None:
        (
            channel_factory_initialize,
            channel_publisher,
            channel_subscriber,
            lowstate_type,
            lowcmd_type,
            imu_state_type,
            lowstate_factory,
            crc_type,
        ) = _load_sdk2py_modules()

        interface = network_interface or None
        if interface and interface.lower() == "none":
            interface = None
        channel_factory_initialize(domain_id, interface)
        self._lowstate_publisher = None
        if enable_lowstate:
            self._lowstate_publisher = channel_publisher(lowstate_topic, lowstate_type)
            self._lowstate_publisher.Init()
        self._secondary_imu_publisher = None
        if enable_secondary_imu:
            self._secondary_imu_publisher = channel_publisher(secondary_imu_topic, imu_state_type)
            self._secondary_imu_publisher.Init()
        self._lowstate_factory = lowstate_factory
        self._imu_state_type = imu_state_type
        self._crc = crc_type()

        self._recv_socket = None
        if enable_lowstate:
            self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._recv_socket.bind((bind_host, lowstate_port))
            self._recv_socket.setblocking(False)
        self._secondary_imu_recv_socket = None
        if enable_secondary_imu:
            self._secondary_imu_recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._secondary_imu_recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._secondary_imu_recv_socket.bind((bind_host, secondary_imu_port))
            self._secondary_imu_recv_socket.setblocking(False)

        self._lowcmd_send_socket = None
        self._lowcmd_subscriber = None
        self._lowcmd_send_target = (bind_host, lowcmd_port)
        if enable_lowcmd:
            self._lowcmd_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._lowcmd_subscriber = channel_subscriber(lowcmd_topic, lowcmd_type)
            self._lowcmd_subscriber.Init(self._on_lowcmd, 10)
        print(
            "SDK2 Python sidecar initialized "
            f"(domain_id={domain_id}, lowstate_enabled={enable_lowstate}, lowstate_topic={lowstate_topic}, "
            f"lowcmd_enabled={enable_lowcmd}, lowcmd_topic={lowcmd_topic}, "
            f"secondary_imu_enabled={enable_secondary_imu}, secondary_imu_topic={secondary_imu_topic}, "
            f"network_interface={interface}, udp_host={bind_host}, "
            f"lowstate_port={lowstate_port}, secondary_imu_port={secondary_imu_port}, lowcmd_port={lowcmd_port})",
            file=sys.stderr,
            flush=True,
        )

    def poll_once(self) -> None:
        if self._recv_socket is None:
            return
        while True:
            try:
                packet, _addr = self._recv_socket.recvfrom(65535)
            except BlockingIOError:
                break
            self._publish_lowstate(packet)
        if self._secondary_imu_recv_socket is None:
            return
        while True:
            try:
                packet, _addr = self._secondary_imu_recv_socket.recvfrom(65535)
            except BlockingIOError:
                return
            self._publish_secondary_imu(packet)

    def shutdown(self) -> None:
        if self._recv_socket is not None:
            self._recv_socket.close()
        if self._lowstate_publisher is not None:
            self._lowstate_publisher.Close()
        if self._secondary_imu_recv_socket is not None:
            self._secondary_imu_recv_socket.close()
        if self._secondary_imu_publisher is not None:
            self._secondary_imu_publisher.Close()
        if self._lowcmd_subscriber is not None:
            self._lowcmd_subscriber.Close()
        if self._lowcmd_send_socket is not None:
            self._lowcmd_send_socket.close()

    def _publish_lowstate(self, packet: bytes) -> None:
        if self._lowstate_publisher is None:
            return
        payload = decode_native_lowstate_packet(packet)
        message = build_lowstate_message(payload, self._lowstate_factory, self._crc)
        self._lowstate_publisher.Write(message)

    def _publish_secondary_imu(self, packet: bytes) -> None:
        if self._secondary_imu_publisher is None:
            return
        payload = decode_native_secondary_imu_packet(packet)
        message = build_secondary_imu_message(payload, self._imu_state_type)
        self._secondary_imu_publisher.Write(message)

    def _on_lowcmd(self, message) -> None:
        if self._lowcmd_send_socket is None:
            return
        if not has_valid_or_unset_crc(message, self._crc):
            print("dropped SDK2 Python lowcmd with invalid CRC", file=sys.stderr, flush=True)
            return
        packet = build_lowcmd_packet(message)
        self._lowcmd_send_socket.sendto(packet, self._lowcmd_send_target)


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
    message.imu_state.rpy = _fixed_float_list(payload.get("imu_rpy"), 3)
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


def build_secondary_imu_message(payload, imu_state_type):
    """Build a Unitree HG IMUState_ message from a decoded simulator packet."""
    quaternion = _fixed_float_list(payload.get("quaternion_wxyz"), 4)
    gyroscope = _fixed_float_list(payload.get("gyroscope_body"), 3)
    accelerometer = _fixed_float_list(payload.get("accelerometer_body"), 3)
    rpy = _fixed_float_list(payload.get("rpy"), 3)
    temperature = 0
    try:
        return imu_state_type(quaternion, gyroscope, accelerometer, rpy, temperature)
    except TypeError:
        # Test doubles may allow default construction plus attribute writes.
        message = imu_state_type()
        message.quaternion = quaternion
        message.gyroscope = gyroscope
        message.accelerometer = accelerometer
        message.rpy = rpy
        message.temperature = temperature
        return message


def has_valid_or_unset_crc(message, crc) -> bool:
    """Accept zero CRC for compatibility, otherwise require SDK2 Python CRC match."""
    original_crc = int(getattr(message, "crc", 0))
    if original_crc == 0:
        return True
    message.crc = 0
    try:
        expected = crc.Crc(message)
    finally:
        message.crc = original_crc
    return original_crc == int(expected)


def build_lowcmd_packet(message) -> bytes:
    """Build a localhost lowcmd packet from a Unitree HG LowCmd_ message."""
    motor_cmd = message.motor_cmd
    count = min(BODY_JOINT_COUNT, len(motor_cmd))
    return encode_native_lowcmd_packet(
        mode_pr=int(message.mode_pr),
        mode_machine=int(message.mode_machine),
        joint_positions_dds=[float(motor_cmd[index].q) for index in range(count)],
        joint_velocities_dds=[float(motor_cmd[index].dq) for index in range(count)],
        joint_torques_dds=[float(motor_cmd[index].tau) for index in range(count)],
        joint_kp_dds=[float(motor_cmd[index].kp) for index in range(count)],
        joint_kd_dds=[float(motor_cmd[index].kd) for index in range(count)],
    )


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
    parser.add_argument("--lowcmd-topic", default="rt/lowcmd")
    parser.add_argument("--network-interface", default="lo")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--lowstate-port", type=int, required=True)
    parser.add_argument("--secondary-imu-topic", default="rt/secondary_imu")
    parser.add_argument("--secondary-imu-port", type=int, required=True)
    parser.add_argument("--lowcmd-port", type=int, required=True)
    parser.add_argument("--enable-lowstate", action="store_true")
    parser.add_argument("--enable-lowcmd", action="store_true")
    parser.add_argument("--enable-secondary-imu", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    sidecar = UnitreeSdk2PySidecar(
        domain_id=args.domain_id,
        lowstate_topic=args.lowstate_topic,
        lowcmd_topic=args.lowcmd_topic,
        network_interface=args.network_interface,
        bind_host=args.bind_host,
        lowstate_port=args.lowstate_port,
        lowcmd_port=args.lowcmd_port,
        enable_lowstate=args.enable_lowstate,
        enable_lowcmd=args.enable_lowcmd,
        enable_secondary_imu=args.enable_secondary_imu,
        secondary_imu_topic=args.secondary_imu_topic,
        secondary_imu_port=args.secondary_imu_port,
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
