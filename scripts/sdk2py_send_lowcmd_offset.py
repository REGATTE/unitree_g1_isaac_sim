#!/usr/bin/env python3
"""Conservative SDK2 Python `rt/lowcmd` publisher for the Unitree G1 simulator."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
LOCAL_SDK2PY_ROOT = Path.home() / "unitree_sdk2_python"
for import_root in (SRC_ROOT, LOCAL_SDK2PY_ROOT):
    if import_root.exists() and str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from mapping.joints import BODY_JOINT_COUNT, DDS_G1_29DOF_JOINT_NAMES
from tooling import resolve_joint_index


def positive_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected a positive finite float, got {value!r}")
    return parsed


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected a non-negative finite float, got {value!r}")
    return parsed


class LowStateSeedListener:
    def __init__(self) -> None:
        self.positions: list[float] | None = None

    def on_message(self, msg) -> None:
        self.positions = [float(motor.q) for motor in msg.motor_state]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a conservative SDK2 Python G1 `rt/lowcmd` offset.")
    parser.add_argument("--dds-domain-id", type=int, default=1, help="DDS domain id.")
    parser.add_argument("--lowstate-topic", type=str, default="rt/lowstate", help="Lowstate DDS topic.")
    parser.add_argument("--lowcmd-topic", type=str, default="rt/lowcmd", help="Lowcmd DDS topic.")
    parser.add_argument(
        "--network-interface",
        type=str,
        default="lo",
        help="Network interface passed to Unitree SDK2 Python. Use 'none' to omit it.",
    )
    parser.add_argument(
        "--joint-name",
        type=str,
        default="left_shoulder_pitch_joint",
        choices=DDS_G1_29DOF_JOINT_NAMES,
        help="DDS-order body joint to offset from the current lowstate pose.",
    )
    parser.add_argument(
        "--offset-rad",
        type=float,
        default=0.05,
        help="Position offset in radians added to the latest lowstate pose.",
    )
    parser.add_argument("--default-kp", type=float, default=20.0)
    parser.add_argument("--default-kd", type=float, default=1.0)
    parser.add_argument("--target-kp", type=float, default=None)
    parser.add_argument("--target-kd", type=float, default=None)
    parser.add_argument("--rate-hz", type=positive_finite_float, default=50.0)
    parser.add_argument("--duration", type=non_negative_float, default=2.0)
    parser.add_argument("--seed-timeout", type=non_negative_float, default=5.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
    except ImportError as exc:
        print(
            "Failed to import Unitree SDK2 Python lowcmd dependencies: "
            f"{exc}. Expected unitree_sdk2py on PYTHONPATH or at {LOCAL_SDK2PY_ROOT}.",
            file=sys.stderr,
        )
        return 1

    try:
        target_index = resolve_joint_index(args.joint_name)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    network_interface = args.network_interface
    if network_interface.lower() == "none":
        network_interface = None

    ChannelFactoryInitialize(args.dds_domain_id, network_interface)
    seed_listener = LowStateSeedListener()
    lowstate_subscriber = ChannelSubscriber(args.lowstate_topic, LowState_)
    lowstate_subscriber.Init(seed_listener.on_message, 10)
    lowcmd_publisher = ChannelPublisher(args.lowcmd_topic, LowCmd_)
    lowcmd_publisher.Init()
    crc = CRC()

    try:
        deadline = time.monotonic() + max(args.seed_timeout, 0.0)
        while seed_listener.positions is None and time.monotonic() < deadline:
            time.sleep(0.05)

        seed_positions = seed_listener.positions
        if seed_positions is None:
            print(
                f"Did not receive a valid `{args.lowstate_topic}` sample within {args.seed_timeout:.1f}s. "
                "Start Isaac Sim with SDK2 Python lowstate enabled first.",
                file=sys.stderr,
            )
            return 1
        if len(seed_positions) < BODY_JOINT_COUNT:
            print(
                f"Received only {len(seed_positions)} motor positions, expected at least {BODY_JOINT_COUNT}.",
                file=sys.stderr,
            )
            return 1

        target_positions = list(seed_positions[:BODY_JOINT_COUNT])
        target_positions[target_index] += args.offset_rad
        target_kp = float(args.default_kp if args.target_kp is None else args.target_kp)
        target_kd = float(args.default_kd if args.target_kd is None else args.target_kd)

        period_seconds = 1.0 / args.rate_hz
        end_time = time.monotonic() + args.duration
        publish_count = 0
        while time.monotonic() < end_time:
            msg = unitree_hg_msg_dds__LowCmd_()
            msg.mode_pr = 0
            msg.mode_machine = 0
            for index in range(min(BODY_JOINT_COUNT, len(msg.motor_cmd))):
                motor = msg.motor_cmd[index]
                motor.q = float(target_positions[index])
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = target_kp if index == target_index else float(args.default_kp)
                motor.kd = target_kd if index == target_index else float(args.default_kd)
            msg.crc = 0
            msg.crc = crc.Crc(msg)
            lowcmd_publisher.Write(msg)
            publish_count += 1
            time.sleep(period_seconds)
    finally:
        lowstate_subscriber.Close()
        lowcmd_publisher.Close()

    print(
        f"Published {publish_count} SDK2 Python lowcmd samples on {args.lowcmd_topic} "
        f"for joint `{args.joint_name}` with offset {args.offset_rad:+.4f} rad."
    )
    print(
        f"Seed position was {seed_positions[target_index]:+.4f} rad; "
        f"target was {target_positions[target_index]:+.4f} rad."
    )
    print(
        f"Default gains were kp={float(args.default_kp):.4f}, kd={float(args.default_kd):.4f}; "
        f"target joint gains were kp={target_kp:.4f}, kd={target_kd:.4f}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
