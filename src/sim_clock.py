"""ROS 2 /clock publication from the Isaac simulation timeline."""

from __future__ import annotations

import math

from config import AppConfig
from ros2_bridge_runtime import (
    enable_isaac_ros2_bridge_extension,
    prepare_isaac_ros2_bridge_environment,
)
from runtime_logging import get_logger


LOGGER = get_logger("sim_clock")


class IsaacSimClockPublisher:
    """Publish ROS 2 clock samples from Isaac simulation seconds."""

    def __init__(self, *, domain_id: int) -> None:
        prepare_isaac_ros2_bridge_environment(domain_id)
        enable_isaac_ros2_bridge_extension()

        import rclpy
        from rclpy.qos import QoSProfile
        from rosgraph_msgs.msg import Clock

        self._rclpy = rclpy
        self._clock_type = Clock
        self._owns_rclpy_context = not rclpy.ok()
        if self._owns_rclpy_context:
            rclpy.init(args=None)

        self._node = rclpy.create_node("unitree_g1_isaac_sim_clock")
        self._publisher = self._node.create_publisher(Clock, "/clock", QoSProfile(depth=1))
        self._last_stamp_seconds: float | None = None
        LOGGER.info("created Isaac simulation /clock publisher")

    def publish(self, simulation_time_seconds: float) -> None:
        """Publish a /clock sample for the current simulation timestamp."""
        message = self._clock_type()
        message.clock = seconds_to_ros_time_msg(simulation_time_seconds)
        self._publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        self._last_stamp_seconds = simulation_time_seconds

    def shutdown(self) -> None:
        """Tear down the ROS node."""
        try:
            if hasattr(self, "_node"):
                self._node.destroy_node()
            if self._owns_rclpy_context and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            LOGGER.exception("failed to shutdown Isaac simulation /clock publisher")


def setup_sim_clock(config: AppConfig) -> IsaacSimClockPublisher:
    return IsaacSimClockPublisher(domain_id=config.dds_domain_id)


def seconds_to_ros_time_msg(stamp_seconds: float):
    """Convert non-negative finite seconds into a builtin_interfaces/Time message."""
    if stamp_seconds < 0.0 or not math.isfinite(stamp_seconds):
        raise ValueError(f"simulation time must be finite and non-negative, got {stamp_seconds!r}")

    from builtin_interfaces.msg import Time

    stamp_sec = int(stamp_seconds)
    stamp_nanosec = int(round((stamp_seconds - stamp_sec) * 1_000_000_000))
    if stamp_nanosec >= 1_000_000_000:
        stamp_sec += 1
        stamp_nanosec -= 1_000_000_000

    message = Time()
    message.sec = stamp_sec
    message.nanosec = stamp_nanosec
    return message
