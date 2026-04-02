#!/usr/bin/env python3
"""Passive `rt/lowstate` listener for the Unitree G1 Isaac Sim bridge."""

from __future__ import annotations

import argparse
import csv
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
    received_at_monotonic: float
    tick: int
    positions: list[float]
    velocities: list[float]
    torques: list[float]
    imu_quaternion_xyzw: list[float]
    imu_accelerometer: list[float]
    imu_gyroscope: list[float]


@dataclass(frozen=True)
class LowStateCapture:
    latest: LowStateSample | None
    history: list[LowStateSample]
    messages_seen: int
    messages_rejected: int


class LowStateListener:
    def __init__(self, topic_name: str, max_history_samples: int = 5000) -> None:
        from unitree_sdk2py.utils.crc import CRC

        self._topic_name = topic_name
        self._max_history_samples = max(max_history_samples, 1)
        self._crc = CRC()
        self._latest: LowStateSample | None = None
        self._history: list[LowStateSample] = []
        self._lock = threading.Lock()
        self._messages_seen = 0
        self._messages_rejected = 0

    @property
    def latest(self) -> LowStateSample | None:
        with self._lock:
            return self._latest

    @property
    def history(self) -> list[LowStateSample]:
        with self._lock:
            return list(self._history)

    def capture(self) -> LowStateCapture:
        with self._lock:
            return LowStateCapture(
                latest=self._latest,
                history=list(self._history),
                messages_seen=self._messages_seen,
                messages_rejected=self._messages_rejected,
            )

    def on_message(self, msg) -> None:
        if self._crc.Crc(msg) != msg.crc:
            self._messages_rejected += 1
            return

        motor_state = msg.motor_state
        sample = LowStateSample(
            received_at_monotonic=time.monotonic(),
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
            self._history.append(sample)
            overflow = len(self._history) - self._max_history_samples
            if overflow > 0:
                del self._history[:overflow]
        self._messages_seen += 1

    def print_summary(
        self,
        preview_joints: int,
        target_joint_name: str | None = None,
        *,
        capture: LowStateCapture | None = None,
    ) -> None:
        if capture is None:
            capture = self.capture()
        print_lowstate_capture_summary(
            capture,
            preview_joints=preview_joints,
            target_joint_name=target_joint_name,
        )


def print_lowstate_capture_summary(
    capture: LowStateCapture,
    *,
    preview_joints: int,
    target_joint_name: str | None = None,
) -> None:
    sample = capture.latest
    if sample is None:
        print("No valid lowstate sample received yet.")
        return

    print(f"tick={sample.tick} valid_messages={capture.messages_seen} crc_rejected={capture.messages_rejected}")
    target_joint_index = None
    if target_joint_name is not None:
        target_joint_index = resolve_joint_index(target_joint_name)
        print(
            f"target {target_joint_name}: "
            f"q={sample.positions[target_joint_index]: .5f} "
            f"dq={sample.velocities[target_joint_index]: .5f} "
            f"tau={sample.torques[target_joint_index]: .5f}"
        )
    for index, joint_name in enumerate(DDS_G1_29DOF_JOINT_NAMES[:preview_joints]):
        if target_joint_index is not None and index == target_joint_index:
            continue
        print(
            f"{index:02d} {joint_name}: "
            f"q={sample.positions[index]: .5f} "
            f"dq={sample.velocities[index]: .5f} "
            f"tau={sample.torques[index]: .5f}"
        )
    print(f"imu_quaternion_xyzw={_format_vector(sample.imu_quaternion_xyzw)}")
    print(f"imu_accelerometer={_format_vector(sample.imu_accelerometer)}")
    print(f"imu_gyroscope={_format_vector(sample.imu_gyroscope)}")
    if target_joint_name is not None:
        target_stats = summarize_joint_history(
            capture.history,
            target_joint_name,
            joint_index=target_joint_index,
        )
        if target_stats is not None:
            print(
                f"target_history {target_joint_name}: "
                f"samples={target_stats.sample_count} "
                f"duration_s={target_stats.duration_seconds:.3f} "
                f"q_min={target_stats.position_min:.5f} "
                f"q_max={target_stats.position_max:.5f} "
                f"dq_peak={target_stats.velocity_peak_abs:.5f} "
                f"tau_peak={target_stats.torque_peak_abs:.5f}"
            )


@dataclass(frozen=True)
class JointHistorySummary:
    sample_count: int
    duration_seconds: float
    position_min: float
    position_max: float
    velocity_peak_abs: float
    torque_peak_abs: float


def resolve_joint_index(joint_name: str) -> int:
    try:
        return DDS_G1_29DOF_JOINT_NAMES.index(joint_name)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported DDS joint name `{joint_name}`. "
            f"Expected one of: {DDS_G1_29DOF_JOINT_NAMES}"
        ) from exc


def _filter_joint_history_samples(history: list[LowStateSample], joint_index: int) -> list[LowStateSample]:
    return [
        sample
        for sample in history
        if joint_index < len(sample.positions)
        and joint_index < len(sample.velocities)
        and joint_index < len(sample.torques)
    ]


def summarize_joint_history(
    history: list[LowStateSample],
    joint_name: str,
    *,
    joint_index: int | None = None,
) -> JointHistorySummary | None:
    if not history:
        return None
    if joint_index is None:
        joint_index = resolve_joint_index(joint_name)
    filtered_history = _filter_joint_history_samples(history, joint_index)
    if not filtered_history:
        return None
    start_time = filtered_history[0].received_at_monotonic
    end_time = filtered_history[-1].received_at_monotonic
    positions = [sample.positions[joint_index] for sample in filtered_history]
    velocities = [sample.velocities[joint_index] for sample in filtered_history]
    torques = [sample.torques[joint_index] for sample in filtered_history]
    return JointHistorySummary(
        sample_count=len(filtered_history),
        duration_seconds=max(end_time - start_time, 0.0),
        position_min=min(positions),
        position_max=max(positions),
        velocity_peak_abs=max(abs(value) for value in velocities),
        torque_peak_abs=max(abs(value) for value in torques),
    )


def write_joint_history_csv(
    history: list[LowStateSample],
    joint_name: str,
    output_path: Path,
    *,
    joint_index: int | None = None,
) -> None:
    if joint_index is None:
        joint_index = resolve_joint_index(joint_name)
    filtered_history = _filter_joint_history_samples(history, joint_index)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    start_time = filtered_history[0].received_at_monotonic if filtered_history else 0.0
    with output_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time_s", "tick", "joint_name", "q", "dq", "tau"])
        for sample in filtered_history:
            writer.writerow(
                [
                    f"{sample.received_at_monotonic - start_time:.6f}",
                    sample.tick,
                    joint_name,
                    f"{sample.positions[joint_index]:.9f}",
                    f"{sample.velocities[joint_index]:.9f}",
                    f"{sample.torques[joint_index]:.9f}",
                ]
            )


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
    parser.add_argument(
        "--joint-name",
        type=str,
        default=None,
        choices=DDS_G1_29DOF_JOINT_NAMES,
        help="Optional DDS-order joint to print explicitly, even if it falls outside the preview range.",
    )
    parser.add_argument(
        "--csv-path",
        type=Path,
        default=None,
        help="Optional CSV output path for a selected `--joint-name` time series.",
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

    target_joint_index = resolve_joint_index(args.joint_name) if args.joint_name is not None else None
    listener = LowStateListener(topic_name=args.topic)
    ChannelFactoryInitialize(args.dds_domain_id)
    subscriber = ChannelSubscriber(args.topic, LowState_)
    subscriber.Init(listener.on_message, 32)

    deadline = time.time() + max(args.duration, 0.0)
    while time.time() < deadline:
        time.sleep(0.05)

    capture = listener.capture()
    history = capture.history
    if args.csv_path is not None:
        if args.joint_name is None:
            print("--csv-path requires --joint-name so the listener knows which DDS joint to export.", file=sys.stderr)
            return 1
        if not history:
            print("No valid lowstate sample received yet.", file=sys.stderr)
            return 1
        write_joint_history_csv(
            history,
            args.joint_name,
            args.csv_path,
            joint_index=target_joint_index,
        )
        exported_history = _filter_joint_history_samples(history, target_joint_index)
        print(f"Wrote {len(exported_history)} lowstate samples for `{args.joint_name}` to {args.csv_path}")

    listener.print_summary(
        preview_joints=max(args.preview_joints, 0),
        target_joint_name=args.joint_name,
        capture=capture,
    )
    return 0


def _format_vector(values: list[float]) -> str:
    return "(" + ", ".join(f"{value: .5f}" for value in values) + ")"


if __name__ == "__main__":
    raise SystemExit(main())
