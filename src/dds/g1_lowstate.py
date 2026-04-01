"""`rt/lowstate` publication for the Unitree G1 simulator.

The simulator keeps its internal state in Isaac Sim conventions:

- body joints are first read in simulator articulation order
- base orientation is stored in validated Isaac Sim `wxyz`
- IMU-like acceleration is derived from simulation time, never wall time

This module is the DDS boundary that repackages those values into the
Unitree low-level state message expected by external SDK or ROS 2 clients.
The goal is for those clients to talk to the simulator exactly as they would
to a real robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from mapping import to_dds_ordered_snapshot
from robot_state import JointStateSnapshot, RobotKinematicSnapshot


def unitree_sdk_is_available() -> bool:
    """Return whether the Unitree SDK Python package can be imported.

    Importing lazily keeps this repository importable on machines that only
    have Isaac Sim available. The runtime will emit a clear warning instead of
    failing during module import if the DDS SDK is not installed yet.
    """
    try:
        import unitree_sdk2py  # noqa: F401
    except ImportError:
        return False
    return True


@dataclass(frozen=True)
class LowStatePublishStats:
    """Small runtime counters for DDS state publication."""

    published_messages: int = 0
    skipped_messages: int = 0


class G1LowStatePublisher:
    """Publish G1 simulator state on `rt/lowstate`.

    The publisher uses the existing validated state path:

    1. `RobotStateReader.read_kinematic_snapshot(sample_dt=...)`
    2. `to_dds_ordered_snapshot(...)` for the 29 body joints
    3. Unitree SDK2 `LowState_` message construction
    4. CRC calculation immediately before publish

    The publisher is intentionally stateless with respect to the simulator.
    It consumes snapshots produced by the main loop and only owns DDS-facing
    objects such as the topic publisher, the reusable `LowState_` message,
    and the CRC helper.
    """

    def __init__(self, topic_name: str = "rt/lowstate") -> None:
        self._topic_name = topic_name
        self._publisher: Any | None = None
        self._crc_helper: Any | None = None
        self._low_state_message: Any | None = None
        self._sdk_enabled = False
        self._warned_unavailable = False
        self._stats = LowStatePublishStats()

    @property
    def topic_name(self) -> str:
        return self._topic_name

    @property
    def sdk_enabled(self) -> bool:
        return self._sdk_enabled

    @property
    def stats(self) -> LowStatePublishStats:
        return self._stats

    def initialize(self) -> bool:
        """Create the Unitree publisher objects if the SDK is available."""
        if self._sdk_enabled:
            return True
        if not unitree_sdk_is_available():
            self._warn_sdk_unavailable()
            return False

        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.utils.crc import CRC

        self._publisher = ChannelPublisher(self._topic_name, LowState_)
        self._publisher.Init()
        self._crc_helper = CRC()
        self._low_state_message = unitree_hg_msg_dds__LowState_()
        self._sdk_enabled = True
        print(f"[unitree_g1_isaac_sim] DDS lowstate publisher ready on {self._topic_name}")
        return True

    def publish(self, snapshot: RobotKinematicSnapshot) -> bool:
        """Publish one simulator state sample as `LowState_`.

        Returns `True` when a DDS message is written. Returns `False` when
        the SDK is unavailable or the publisher could not be initialized.
        """
        if not self.initialize():
            self._stats = LowStatePublishStats(
                published_messages=self._stats.published_messages,
                skipped_messages=self._stats.skipped_messages + 1,
            )
            return False

        low_state_message = self._low_state_message
        dds_joint_snapshot = self._build_dds_joint_snapshot(snapshot)

        self._populate_motor_state(low_state_message, dds_joint_snapshot)
        self._populate_imu_state(low_state_message, snapshot)

        low_state_message.tick += 1
        low_state_message.crc = self._crc_helper.Crc(low_state_message)
        self._publisher.Write(low_state_message)
        self._stats = LowStatePublishStats(
            published_messages=self._stats.published_messages + 1,
            skipped_messages=self._stats.skipped_messages,
        )
        return True

    def _build_dds_joint_snapshot(self, snapshot: RobotKinematicSnapshot) -> JointStateSnapshot:
        """Convert the kinematic snapshot into validated DDS joint order.

        The conversion helper remains the only place where simulator-order
        joint vectors are relabeled into DDS order, preserving the explicit
        compatibility boundary requested in the project constraints.
        """
        return to_dds_ordered_snapshot(
            JointStateSnapshot(
                joint_names=list(snapshot.joint_names),
                joint_positions=list(snapshot.joint_positions),
                joint_velocities=list(snapshot.joint_velocities),
                joint_efforts=list(snapshot.joint_efforts) if snapshot.joint_efforts is not None else None,
            )
        )

    def _populate_motor_state(self, low_state_message: Any, snapshot: JointStateSnapshot) -> None:
        """Copy the 29 body-joint state vectors into the DDS motor array."""
        motor_state = low_state_message.motor_state
        effort_values = snapshot.joint_efforts or [0.0] * len(snapshot.joint_positions)
        joint_count = min(len(motor_state), len(snapshot.joint_positions))
        for joint_index in range(joint_count):
            motor = motor_state[joint_index]
            motor.q = float(snapshot.joint_positions[joint_index])
            motor.dq = float(snapshot.joint_velocities[joint_index])
            motor.tau_est = float(effort_values[joint_index])

    def _populate_imu_state(self, low_state_message: Any, snapshot: RobotKinematicSnapshot) -> None:
        """Populate the DDS IMU fields from the simulator kinematic snapshot.

        Internal orientation stays in Isaac Sim `wxyz`. The Unitree reference
        DDS bridge writes the IMU quaternion field in `xyzw`, so this method
        performs that conversion explicitly instead of mutating the simulator
        convention elsewhere in the codebase.
        """
        imu_state = low_state_message.imu_state
        quaternion_wxyz = tuple(float(value) for value in snapshot.base_quaternion_wxyz)
        imu_state.quaternion[:] = _wxyz_to_xyzw(quaternion_wxyz)
        imu_state.accelerometer[:] = [float(value) for value in snapshot.imu_linear_acceleration_body]
        imu_state.gyroscope[:] = [float(value) for value in snapshot.imu_angular_velocity_body]

    def _warn_sdk_unavailable(self) -> None:
        if self._warned_unavailable:
            return
        print(
            "[unitree_g1_isaac_sim] DDS requested but `unitree_sdk2py` is not installed. "
            "Skipping `rt/lowstate` publication."
        )
        self._warned_unavailable = True


def _wxyz_to_xyzw(quaternion_wxyz: tuple[float, float, float, float]) -> list[float]:
    """Convert a scalar-first quaternion into the Unitree IMU field layout."""
    w, x, y, z = quaternion_wxyz
    return [x, y, z, w]
