#!/usr/bin/env python3
"""Conservative `rt/lowcmd` publisher for the Unitree G1 Isaac Sim bridge.

This tool first listens for one valid `rt/lowstate` sample, then reuses the
current joint positions as a hold posture and applies a small offset to one
selected DDS joint. That avoids sending a blind full-body posture jump.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
# These helpers are shipped as standalone scripts, so make the in-repo `src/`
# package importable when the script is launched directly from the checkout.
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from mapping.joints import BODY_JOINT_COUNT, DDS_G1_29DOF_JOINT_NAMES
from tooling import resolve_joint_index


def positive_finite_float(value: str) -> float:
    """Parse a CLI float argument that must be positive and finite."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected a positive finite float, got {value!r}")
    return parsed


def non_negative_float(value: str) -> float:
    """Parse a CLI float argument that may be zero but not negative or non-finite."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected a non-negative finite float, got {value!r}")
    return parsed


@dataclass
class LowStateSeed:
    positions: list[float]


class LowStateSeedListener:
    def __init__(self) -> None:
        self._seed: LowStateSeed | None = None

    def on_message(self, msg) -> None:
        motor_state = msg.motor_state
        self._seed = LowStateSeed(
            positions=[float(motor_state[index].q) for index in range(len(motor_state))],
        )

    @property
    def seed(self) -> LowStateSeed | None:
        return self._seed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a conservative G1 `rt/lowcmd` offset.")
    parser.add_argument("--dds-domain-id", type=int, default=1, help="DDS domain id.")
    parser.add_argument("--lowstate-topic", type=str, default="rt/lowstate", help="Lowstate DDS topic.")
    parser.add_argument("--lowcmd-topic", type=str, default="rt/lowcmd", help="Lowcmd DDS topic.")
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
        default=0.10,
        help="Position offset in radians added to the latest lowstate pose.",
    )
    parser.add_argument(
        "--default-kp",
        type=float,
        default=20.0,
        help="Proportional gain written to all non-target joints to hold posture.",
    )
    parser.add_argument(
        "--default-kd",
        type=float,
        default=1.0,
        help="Derivative gain written to all non-target joints to hold posture.",
    )
    parser.add_argument(
        "--target-kp",
        type=float,
        default=None,
        help="Proportional gain written only to the selected target joint. Defaults to --default-kp.",
    )
    parser.add_argument(
        "--target-kd",
        type=float,
        default=None,
        help="Derivative gain written only to the selected target joint. Defaults to --default-kd.",
    )
    parser.add_argument(
        "--rate-hz",
        type=positive_finite_float,
        default=50.0,
        help="Publish frequency for the command hold burst.",
    )
    parser.add_argument(
        "--duration",
        type=non_negative_float,
        default=2.0,
        help="How long to publish the same hold command.",
    )
    parser.add_argument(
        "--seed-timeout",
        type=non_negative_float,
        default=5.0,
        help="How long to wait for an initial lowstate seed sample.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from unitree_hg.msg import LowCmd, LowState
    except ImportError as exc:
        print(f"Failed to import ROS 2 lowcmd dependencies: {exc}", file=sys.stderr)
        return 1

    try:
        target_index = resolve_joint_index(args.joint_name)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    seed_listener = LowStateSeedListener()
    rclpy.init(args=None)
    node = Node("unitree_g1_lowcmd_sender", enable_rosout=False)
    node.create_subscription(LowState, args.lowstate_topic, seed_listener.on_message, 32)
    publisher = node.create_publisher(LowCmd, args.lowcmd_topic, 32)

    deadline = time.monotonic() + max(args.seed_timeout, 0.0)
    try:
        while seed_listener.seed is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

        seed = seed_listener.seed
        if seed is None:
            print(
                f"Did not receive a valid `{args.lowstate_topic}` sample within {args.seed_timeout:.1f}s. "
                "Start Isaac Sim with DDS enabled first.",
                file=sys.stderr,
            )
            return 1

        if len(seed.positions) < BODY_JOINT_COUNT:
            print(
                f"Received only {len(seed.positions)} motor positions, expected at least {BODY_JOINT_COUNT}.",
                file=sys.stderr,
            )
            return 1

        target_positions = list(seed.positions[:BODY_JOINT_COUNT])
        target_positions[target_index] += args.offset_rad
        target_kp = float(args.default_kp if args.target_kp is None else args.target_kp)
        target_kd = float(args.default_kd if args.target_kd is None else args.target_kd)

        period_seconds = 1.0 / args.rate_hz
        end_time = time.monotonic() + args.duration
        publish_count = 0

        while time.monotonic() < end_time and rclpy.ok():
            msg = LowCmd()
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
            publisher.publish(msg)
            publish_count += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_seconds)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(
        f"Published {publish_count} lowcmd samples on {args.lowcmd_topic} "
        f"for joint `{args.joint_name}` with offset {args.offset_rad:+.4f} rad."
    )
    print(f"Seed position was {seed.positions[target_index]:+.4f} rad; target was {target_positions[target_index]:+.4f} rad.")
    print(
        f"Default gains were kp={float(args.default_kp):.4f}, kd={float(args.default_kd):.4f}; "
        f"target joint gains were kp={target_kp:.4f}, kd={target_kd:.4f}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
