#!/usr/bin/env python3
"""Passive SDK2 Python `rt/lowstate` listener for the Unitree G1 simulator."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
LOCAL_SDK2PY_ROOT = Path.home() / "unitree_sdk2_python"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for import_root in (SRC_ROOT, SCRIPT_ROOT, LOCAL_SDK2PY_ROOT):
    if import_root.exists() and str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from mapping.joints import DDS_G1_29DOF_JOINT_NAMES
from lowstate_listener import LowStateListener, print_lowstate_capture_summary


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected a non-negative finite float, got {value!r}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Listen to Unitree G1 SDK2 Python `rt/lowstate`.")
    parser.add_argument("--dds-domain-id", type=int, default=1, help="DDS domain id.")
    parser.add_argument("--topic", type=str, default="rt/lowstate", help="SDK2 Python lowstate DDS topic.")
    parser.add_argument(
        "--network-interface",
        type=str,
        default="lo",
        help="Network interface passed to Unitree SDK2 Python. Use 'none' to omit it.",
    )
    parser.add_argument(
        "--duration",
        type=non_negative_float,
        default=5.0,
        help="Seconds to wait before printing the latest sample summary.",
    )
    parser.add_argument(
        "--preview-joints",
        type=int,
        default=8,
        help="How many DDS-ordered joints to print from the latest sample.",
    )
    parser.add_argument(
        "--joint-name",
        type=str,
        default=None,
        choices=DDS_G1_29DOF_JOINT_NAMES,
        help="Optional DDS-order joint to print explicitly, even if it falls outside the preview range.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        print(
            "Failed to import Unitree SDK2 Python lowstate dependencies: "
            f"{exc}. Expected unitree_sdk2py on PYTHONPATH or at {LOCAL_SDK2PY_ROOT}.",
            file=sys.stderr,
        )
        return 1

    listener = LowStateListener(topic_name=args.topic)
    network_interface = args.network_interface
    if network_interface.lower() == "none":
        network_interface = None

    ChannelFactoryInitialize(args.dds_domain_id, network_interface)
    subscriber = ChannelSubscriber(args.topic, LowState_)
    subscriber.Init(listener.on_message, 10)

    deadline = time.monotonic() + max(args.duration, 0.0)
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        subscriber.Close()

    print_lowstate_capture_summary(
        listener.capture(),
        preview_joints=max(args.preview_joints, 0),
        target_joint_name=args.joint_name,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
