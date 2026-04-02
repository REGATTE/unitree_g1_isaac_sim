#!/usr/bin/env python3
"""Passive `rt/lowstate` listener for the Unitree G1 Isaac Sim bridge."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from mapping.joints import DDS_G1_29DOF_JOINT_NAMES


@dataclass
class LowStateSample:
    tick: int
    positions: list[float]
    velocities: list[float]
    torques: list[float]
    imu_quaternion_xyzw: list[float]
    imu_accelerometer: list[float]
    imu_gyroscope: list[float]


class LowStateListener:
    def __init__(self, topic_name: str) -> None:
        from unitree_sdk2py.utils.crc import CRC

        self._topic_name = topic_name
        self._crc = CRC()
        self._latest: LowStateSample | None = None
        self._lock = threading.Lock()
        self._messages_seen = 0
        self._messages_rejected = 0

    @property
    def latest(self) -> LowStateSample | None:
        with self._lock:
            return self._latest

    def on_message(self, msg) -> None:
        if self._crc.Crc(msg) != msg.crc:
            self._messages_rejected += 1
            return

        motor_state = msg.motor_state
        sample = LowStateSample(
            tick=int(msg.tick),
            positions=[float(motor_state[index].q) for index in range(len(motor_state))],
            velocities=[float(motor_state[index].dq) for index in range(len(motor_state))],
            torques=[float(motor_state[index].tau_est) for index in range(len(motor_state))],
            imu_quaternion_xyzw=[float(value) for value in msg.imu_state.quaternion],
            imu_accelerometer=[float(value) for value in msg.imu_state.accelerometer],
            imu_gyroscope=[float(value) for value in msg.imu_state.gyroscope],
        )
        with self._lock:
            self._latest = sample
        self._messages_seen += 1

    def print_summary(self, preview_joints: int) -> None:
        sample = self.latest
        if sample is None:
            print("No valid lowstate sample received yet.")
            return

        print(f"tick={sample.tick} valid_messages={self._messages_seen} crc_rejected={self._messages_rejected}")
        for index, joint_name in enumerate(DDS_G1_29DOF_JOINT_NAMES[:preview_joints]):
            print(
                f"{index:02d} {joint_name}: "
                f"q={sample.positions[index]: .5f} "
                f"dq={sample.velocities[index]: .5f} "
                f"tau={sample.torques[index]: .5f}"
            )
        print(f"imu_quaternion_xyzw={_format_vector(sample.imu_quaternion_xyzw)}")
        print(f"imu_accelerometer={_format_vector(sample.imu_accelerometer)}")
        print(f"imu_gyroscope={_format_vector(sample.imu_gyroscope)}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Listen to Unitree G1 `rt/lowstate`.")
    parser.add_argument("--dds-domain-id", type=int, default=1, help="DDS domain id.")
    parser.add_argument("--topic", type=str, default="rt/lowstate", help="Lowstate DDS topic.")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to wait before printing the latest sample summary.",
    )
    parser.add_argument(
        "--preview-joints",
        type=int,
        default=8,
        help="How many DDS-ordered joints to print from the latest sample.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        print(f"Failed to import unitree_sdk2py: {exc}", file=sys.stderr)
        return 1

    listener = LowStateListener(topic_name=args.topic)
    ChannelFactoryInitialize(args.dds_domain_id)
    subscriber = ChannelSubscriber(args.topic, LowState_)
    subscriber.Init(listener.on_message, 32)

    deadline = time.time() + max(args.duration, 0.0)
    while time.time() < deadline:
        time.sleep(0.05)

    listener.print_summary(preview_joints=max(args.preview_joints, 0))
    return 0


def _format_vector(values: list[float]) -> str:
    return "(" + ", ".join(f"{value: .5f}" for value in values) + ")"


if __name__ == "__main__":
    raise SystemExit(main())
