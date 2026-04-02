#!/usr/bin/env python3
"""Conservative `rt/lowcmd` publisher for the Unitree G1 Isaac Sim bridge.

This tool first listens for one valid `rt/lowstate` sample, then reuses the
current joint positions as a hold posture and applies a small offset to one
selected DDS joint. That avoids sending a blind full-body posture jump.
"""

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

from mapping.joints import BODY_JOINT_COUNT, DDS_G1_29DOF_JOINT_NAMES


@dataclass
class LowStateSeed:
    positions: list[float]


class LowStateSeedListener:
    def __init__(self) -> None:
        from unitree_sdk2py.utils.crc import CRC

        self._crc = CRC()
        self._seed: LowStateSeed | None = None
        self._event = threading.Event()

    def on_message(self, msg) -> None:
        if self._crc.Crc(msg) != msg.crc:
            return
        motor_state = msg.motor_state
        self._seed = LowStateSeed(
            positions=[float(motor_state[index].q) for index in range(len(motor_state))],
        )
        self._event.set()

    def wait_for_seed(self, timeout_seconds: float) -> LowStateSeed | None:
        if not self._event.wait(timeout_seconds):
            return None
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
        "--kp",
        type=float,
        default=20.0,
        help="Per-joint proportional gain written into the outgoing DDS command.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1.0,
        help="Per-joint derivative gain written into the outgoing DDS command.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Publish frequency for the command hold burst.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="How long to publish the same hold command.",
    )
    parser.add_argument(
        "--seed-timeout",
        type=float,
        default=5.0,
        help="How long to wait for an initial lowstate seed sample.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
    except ImportError as exc:
        print(f"Failed to import unitree_sdk2py: {exc}", file=sys.stderr)
        return 1

    ChannelFactoryInitialize(args.dds_domain_id)

    seed_listener = LowStateSeedListener()
    lowstate_subscriber = ChannelSubscriber(args.lowstate_topic, LowState_)
    lowstate_subscriber.Init(seed_listener.on_message, 32)

    seed = seed_listener.wait_for_seed(timeout_seconds=max(args.seed_timeout, 0.0))
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

    target_index = DDS_G1_29DOF_JOINT_NAMES.index(args.joint_name)
    target_positions = list(seed.positions[:BODY_JOINT_COUNT])
    target_positions[target_index] += args.offset_rad

    publisher = ChannelPublisher(args.lowcmd_topic, LowCmd_)
    publisher.Init()
    crc = CRC()
    period_seconds = 1.0 / max(args.rate_hz, 1e-6)
    end_time = time.time() + max(args.duration, 0.0)
    publish_count = 0

    while time.time() < end_time:
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_pr = 0
        msg.mode_machine = 0
        for index in range(BODY_JOINT_COUNT):
            motor = msg.motor_cmd[index]
            motor.q = float(target_positions[index])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(args.kp)
            motor.kd = float(args.kd)
        msg.crc = crc.Crc(msg)
        publisher.Write(msg)
        publish_count += 1
        time.sleep(period_seconds)

    print(
        f"Published {publish_count} lowcmd samples on {args.lowcmd_topic} "
        f"for joint `{args.joint_name}` with offset {args.offset_rad:+.4f} rad."
    )
    print(f"Seed position was {seed.positions[target_index]:+.4f} rad; target was {target_positions[target_index]:+.4f} rad.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
