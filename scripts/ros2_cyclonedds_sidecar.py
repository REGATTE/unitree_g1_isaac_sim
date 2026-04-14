#!/usr/bin/env python3
"""ROS 2 / CycloneDDS sidecar bridge for the Unitree G1 simulator."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState

from dds.ros2.bridge_protocol import decode_lowstate_packet, encode_lowcmd_packet


BODY_JOINT_COUNT = 29
TOTAL_MOTOR_SLOTS = 35


class Ros2CycloneDdsSidecar(Node):
    """Publish lowstate and forward lowcmd over localhost UDP."""

    def __init__(
        self,
        *,
        lowstate_topic: str,
        lowcmd_topic: str,
        bind_host: str,
        lowstate_port: int,
        lowcmd_port: int,
    ) -> None:
        super().__init__("unitree_g1_ros2_sidecar", enable_rosout=False)
        self._lowstate_publisher = self.create_publisher(LowState, lowstate_topic, 10)
        self.create_subscription(LowCmd, lowcmd_topic, self._on_lowcmd, 32)

        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_socket.bind((bind_host, lowstate_port))
        self._recv_socket.setblocking(False)

        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_target = (bind_host, lowcmd_port)

    def poll_once(self) -> None:
        while True:
            try:
                packet, _addr = self._recv_socket.recvfrom(65535)
            except BlockingIOError:
                return
            self._publish_lowstate(packet)

    def shutdown(self) -> None:
        self._recv_socket.close()
        self._send_socket.close()

    def _publish_lowstate(self, packet: bytes) -> None:
        payload = decode_lowstate_packet(packet)
        message = LowState()
        message.version = [1, 0]
        message.tick = int(payload.get("tick", 0))
        message.imu_state.quaternion = payload.get("imu_quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
        message.imu_state.accelerometer = payload.get("imu_accelerometer", [0.0, 0.0, 0.0])
        message.imu_state.gyroscope = payload.get("imu_gyroscope", [0.0, 0.0, 0.0])

        positions = payload.get("joint_positions", [])
        velocities = payload.get("joint_velocities", [])
        efforts = payload.get("joint_efforts", [])
        for joint_index in range(min(BODY_JOINT_COUNT, len(message.motor_state), len(positions))):
            motor = message.motor_state[joint_index]
            motor.q = float(positions[joint_index])
            motor.dq = float(velocities[joint_index]) if joint_index < len(velocities) else 0.0
            motor.tau_est = float(efforts[joint_index]) if joint_index < len(efforts) else 0.0

        message.crc = 0
        self._lowstate_publisher.publish(message)

    def _on_lowcmd(self, message: LowCmd) -> None:
        count = min(BODY_JOINT_COUNT, len(message.motor_cmd))
        packet = encode_lowcmd_packet(
            mode_pr=message.mode_pr,
            mode_machine=message.mode_machine,
            positions=[float(message.motor_cmd[index].q) for index in range(count)],
            velocities=[float(message.motor_cmd[index].dq) for index in range(count)],
            torques=[float(message.motor_cmd[index].tau) for index in range(count)],
            kp=[float(message.motor_cmd[index].kp) for index in range(count)],
            kd=[float(message.motor_cmd[index].kd) for index in range(count)],
        )
        self._send_socket.sendto(packet, self._send_target)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the ROS 2 / CycloneDDS sidecar bridge.")
    parser.add_argument("--lowstate-topic", default="rt/lowstate")
    parser.add_argument("--lowcmd-topic", default="rt/lowcmd")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--lowstate-port", type=int, required=True)
    parser.add_argument("--lowcmd-port", type=int, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    rclpy.init(args=None)
    node = Ros2CycloneDdsSidecar(
        lowstate_topic=args.lowstate_topic,
        lowcmd_topic=args.lowcmd_topic,
        bind_host=args.bind_host,
        lowstate_port=args.lowstate_port,
        lowcmd_port=args.lowcmd_port,
    )
    exit_code = 0
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.01)
                node.poll_once()
            except Exception as exc:
                message = str(exc)
                if "context is invalid" in message or "rcl_shutdown already called" in message:
                    break
                print(f"ROS 2 sidecar stopping after runtime error: {exc}", file=sys.stderr)
                exit_code = 1
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
