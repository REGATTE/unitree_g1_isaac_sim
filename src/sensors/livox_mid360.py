"""Isaac LiDAR setup for the G1-mounted Livox MID360.

The real robot ROS 2 path launches ``livox_ros_driver2`` directly and publishes
``sensor_msgs/msg/PointCloud2`` with ``frame_id=mid360_link``. This module keeps
the simulator aligned with that contract by publishing a direct ROS 2
``PointCloud2`` from Isaac, instead of routing point clouds through the Unitree
LowState / LowCmd sidecar bridge. The current live data path uses PhysX scene
raycasts because Isaac Sim 5.1's RTX/Replicator LiDAR writer path currently
fails in this environment while setting OmniGraph array attributes.

The MID360 scan attributes are an approximation adapted from a public NVIDIA
forum snippet for Isaac Sim 5.0. Livox does not currently ship an official
Isaac RTX profile for the MID360, so this should be treated as a practical
navigation-development model rather than a sensor-calibration authority.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from config import AppConfig
from ros2_bridge_runtime import (
    is_incompatible_ros_python_path as _is_incompatible_ros_python_path,
    prepare_isaac_ros2_bridge_environment as _prepare_isaac_ros2_bridge_environment,
)
from runtime_logging import get_logger
from sim_clock import seconds_to_ros_time_msg


LOGGER = get_logger("sensors.livox_mid360")


# Mirrors g1_29dof_description/urdf/g1_29dof.urdf:
#   <joint name="mid360_joint" type="fixed">
#     <origin xyz="0.0002835 0.00003 0.40618" rpy="0 0.04014257279586953 0"/>
#     <parent link="torso_link"/>
#     <child link="mid360_link"/>
#   </joint>
MID360_TRANSLATION_IN_TORSO = (0.0002835, 0.00003, 0.40618)
MID360_RPY_IN_TORSO = (0.0, 0.04014257279586953, 0.0)
MID360_CHANNEL_COUNT = 40
MID360_FRAME_RATE_HZ = 10.0
MID360_POINT_RATE_HZ = 200_000
MID360_POINTS_PER_FRAME = int(round(MID360_POINT_RATE_HZ / MID360_FRAME_RATE_HZ))
MID360_VERTICAL_FOV_DEG = (-7.0, 52.0)
MID360_ELEVATION_DEG = np.linspace(
    MID360_VERTICAL_FOV_DEG[0],
    MID360_VERTICAL_FOV_DEG[1],
    MID360_CHANNEL_COUNT,
).tolist()
MID360_FRAME_PHASE_RAD = math.pi * (3.0 - math.sqrt(5.0))


MID360_RTX_ATTRIBUTES: dict[str, Any] = {
    "omni:sensor:Core:scanType": "ROTARY",
    "omni:sensor:Core:intensityProcessing": "NORMALIZATION",
    "omni:sensor:Core:rotationDirection": "CW",
    "omni:sensor:Core:rayType": "IDEALIZED",
    "omni:sensor:Core:nearRangeM": 0.1,
    "omni:sensor:Core:farRangeM": 40.0,
    "omni:sensor:Core:rangeResolutionM": 0.004,
    "omni:sensor:Core:rangeAccuracyM": 0.025,
    "omni:sensor:Core:avgPowerW": 0.002,
    "omni:sensor:Core:minReflectance": 0.1,
    "omni:sensor:Core:minReflectanceRange": 70.0,
    "omni:sensor:Core:wavelengthNm": 905.0,
    "omni:sensor:Core:pulseTimeNs": 6,
    "omni:sensor:Core:azimuthErrorMean": 0.1,
    "omni:sensor:Core:azimuthErrorStd": 0.5,
    "omni:sensor:Core:elevationErrorMean": 0.1,
    "omni:sensor:Core:elevationErrorStd": 0.5,
    "omni:sensor:Core:maxReturns": 2,
    "omni:sensor:Core:scanRateBaseHz": MID360_FRAME_RATE_HZ,
    "omni:sensor:Core:reportRateBaseHz": MID360_POINT_RATE_HZ,
    "omni:sensor:Core:numberOfEmitters": MID360_CHANNEL_COUNT,
    "omni:sensor:Core:numberOfChannels": MID360_CHANNEL_COUNT,
    "omni:sensor:Core:rangeOffset": 0.03,
    "omni:sensor:Core:intensityMappingType": "LINEAR",
    "omni:sensor:Core:emitterState:s001:azimuthDeg": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:elevationDeg": MID360_ELEVATION_DEG,
    "omni:sensor:Core:emitterState:s001:fireTimeNs": [
        index * 1000 for index in range(MID360_CHANNEL_COUNT)
    ],
    "omni:sensor:Core:emitterState:s001:distanceCorrectionM": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:focalDistM": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:focalSlope": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:horOffsetM": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:reportRateDiv": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:vertOffsetM": [0.0] * MID360_CHANNEL_COUNT,
    "omni:sensor:Core:emitterState:s001:channelId": list(range(1, MID360_CHANNEL_COUNT + 1)),
}


@dataclass(frozen=True)
class LivoxMid360Setup:
    """Details of the Isaac-native MID360 sensor path."""

    frame_prim_path: str
    sensor_prim_path: str
    topic_name: str
    frame_id: str
    publisher: "LivoxMid360RosPublisher | None" = None


class LivoxMid360RosPublisher:
    """Publish a MID360-like PhysX raycast point cloud as ROS 2 PointCloud2."""

    def __init__(
        self,
        *,
        sensor_frame_prim_path: str,
        robot_prim_path: str,
        topic_name: str,
        frame_id: str,
        publish_hz: float,
    ) -> None:
        import omni.physx
        import omni.usd
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        normalized_topic = "/" + _strip_ros_topic_prefix(topic_name)
        self._rclpy = rclpy
        self._stage = omni.usd.get_context().get_stage()
        self._scene_query = omni.physx.get_physx_scene_query_interface()
        self._sensor_frame_prim_path = sensor_frame_prim_path
        self._robot_prim_path = robot_prim_path.rstrip("/")
        self._topic_name = normalized_topic
        self._frame_id = frame_id
        self._publish_period_seconds = 1.0 / publish_hz
        self._next_publish_time_seconds = 0.0
        self._last_data_warning_time_seconds = -math.inf
        self._first_publish_logged = False
        self._scan_index = 0
        self._points_per_scan = MID360_POINTS_PER_FRAME
        self._near_range_m = float(MID360_RTX_ATTRIBUTES["omni:sensor:Core:nearRangeM"])
        self._far_range_m = float(MID360_RTX_ATTRIBUTES["omni:sensor:Core:farRangeM"])
        self._elevations_rad = np.deg2rad(
            np.asarray(
                MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:elevationDeg"],
                dtype=np.float64,
            )
        )
        self._owns_rclpy_context = not rclpy.ok()
        if self._owns_rclpy_context:
            rclpy.init(args=None)

        self._node = rclpy.create_node("unitree_g1_livox_mid360_sim")
        self._publisher = self._node.create_publisher(PointCloud2, normalized_topic, qos_profile_sensor_data)
        LOGGER.info(
            "created direct Livox MID360 ROS 2 publisher node=%s topic='%s' frame_id='%s' "
            "source=physx_raycast points_per_scan=%s publish_hz=%.3f near_range=%.3fm far_range=%.3fm",
            self._node.get_name(),
            normalized_topic,
            frame_id,
            self._points_per_scan,
            publish_hz,
            self._near_range_m,
            self._far_range_m,
        )

    def step(self, simulation_time_seconds: float) -> None:
        """Publish the latest full-scan point cloud if the publish period elapsed."""
        if simulation_time_seconds + 1e-12 < self._next_publish_time_seconds:
            return

        points = self._raycast_scan()
        if points.size == 0:
            self._log_no_data(simulation_time_seconds, "PhysX raycasts did not hit any collision geometry")
            return

        self._publisher.publish(_make_point_cloud2(points, self._frame_id, simulation_time_seconds))
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        self._next_publish_time_seconds = simulation_time_seconds + self._publish_period_seconds
        self._scan_index += 1
        if not self._first_publish_logged:
            self._first_publish_logged = True
            LOGGER.info(
                "Livox MID360 published first PointCloud2 message topic='%s' frame_id='%s' points=%s stamp=%.6f",
                self._topic_name,
                self._frame_id,
                points.shape[0],
                simulation_time_seconds,
            )

    def shutdown(self) -> None:
        """Tear down the ROS node."""
        try:
            if hasattr(self, "_node"):
                self._node.destroy_node()
            if self._owns_rclpy_context and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            LOGGER.exception("failed to shutdown Livox MID360 ROS 2 publisher")

    def _raycast_scan(self) -> np.ndarray:
        from pxr import Gf, UsdGeom

        sensor_prim = self._stage.GetPrimAtPath(self._sensor_frame_prim_path)
        if not sensor_prim or not sensor_prim.IsValid():
            return np.empty((0, 3), dtype=np.float32)

        sensor_to_world = UsdGeom.Xformable(sensor_prim).ComputeLocalToWorldTransform(0.0)
        world_to_sensor = sensor_to_world.GetInverse()
        origin_world = sensor_to_world.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        points: list[tuple[float, float, float]] = []

        for elevation, azimuth in _iter_mid360_scan_angles(
            self._scan_index,
            self._points_per_scan,
            self._elevations_rad,
        ):

            # MID360 is mounted inverted on this G1, so the vertical fan is flipped
            # relative to the sensor profile. This keeps the simulated returns in
            # the same practical direction as the robot-mounted unit.
            direction_sensor = Gf.Vec3d(
                math.cos(elevation) * math.cos(azimuth),
                math.cos(elevation) * math.sin(azimuth),
                -math.sin(elevation),
            )
            direction_world = sensor_to_world.Transform(direction_sensor) - origin_world
            if direction_world.GetLength() <= 0.0:
                continue
            direction_world.Normalize()
            ray_origin_world = origin_world + direction_world * self._near_range_m

            hit = self._scene_query.raycast_closest(
                Gf.Vec3f(ray_origin_world),
                Gf.Vec3f(direction_world),
                self._far_range_m - self._near_range_m,
            )
            if not hit.get("hit", False):
                continue
            collision_path = str(hit.get("collision", ""))
            if collision_path.startswith(self._robot_prim_path + "/"):
                continue
            hit_world = hit["position"]
            hit_sensor = world_to_sensor.Transform(Gf.Vec3d(hit_world.x, hit_world.y, hit_world.z))
            points.append((float(hit_sensor[0]), float(hit_sensor[1]), float(hit_sensor[2])))

        if not points:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def _log_no_data(self, simulation_time_seconds: float, reason: str) -> None:
        if simulation_time_seconds - self._last_data_warning_time_seconds < 2.0:
            return
        self._last_data_warning_time_seconds = simulation_time_seconds
        LOGGER.warning("Livox MID360 point cloud not published yet: %s", reason)


def setup_livox_mid360(config: AppConfig) -> LivoxMid360Setup | None:
    """Create the simulated MID360 and attach a ROS 2 PointCloud2 publisher."""
    if not config.enable_livox_lidar:
        LOGGER.info("simulated Livox MID360 LiDAR disabled")
        return None

    import omni
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    _prepare_isaac_ros2_bridge_environment(config.dds_domain_id)
    _enable_extension("isaacsim.sensors.rtx")
    _enable_extension("isaacsim.ros2.bridge")

    stage = omni.usd.get_context().get_stage()
    parent_prim = _find_parent_prim(stage, config.robot_prim_path, config.livox_lidar_parent_link_name)
    if parent_prim is None:
        raise RuntimeError(
            "Unable to mount simulated MID360: could not find USD parent prim "
            f"named '{config.livox_lidar_parent_link_name}' under '{config.robot_prim_path}'."
        )

    frame_prim_path = f"{parent_prim.GetPath()}/{_clean_prim_name(config.livox_lidar_prim_name)}"
    sensor_prim_name = _clean_prim_name(config.livox_lidar_sensor_prim_name)
    frame_xform = UsdGeom.Xform.Define(stage, Sdf.Path(frame_prim_path))
    frame_xform.ClearXformOpOrder()
    frame_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*MID360_TRANSLATION_IN_TORSO)
    )
    frame_xform.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*_radians_to_degrees(MID360_RPY_IN_TORSO))
    )

    sensor_config = _optional_isaac_string(config.livox_lidar_rtx_config)
    sensor_variant = _optional_isaac_string(config.livox_lidar_rtx_variant)
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=sensor_prim_name,
        parent=frame_prim_path,
        config=sensor_config,
        variant=sensor_variant,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=False,
        **MID360_RTX_ATTRIBUTES,
    )

    publisher = LivoxMid360RosPublisher(
        sensor_frame_prim_path=frame_prim_path,
        robot_prim_path=config.robot_prim_path,
        topic_name=config.livox_lidar_topic,
        frame_id=config.livox_lidar_frame_id,
        publish_hz=float(MID360_RTX_ATTRIBUTES["omni:sensor:Core:scanRateBaseHz"]),
    )

    setup = LivoxMid360Setup(
        frame_prim_path=frame_prim_path,
        sensor_prim_path=str(sensor.GetPath()),
        topic_name=config.livox_lidar_topic,
        frame_id=config.livox_lidar_frame_id,
        publisher=publisher,
    )
    LOGGER.info(
        "created simulated Livox MID360 LiDAR at %s; publishing PointCloud2 on '/%s' with frame_id='%s'",
        setup.sensor_prim_path,
        _strip_ros_topic_prefix(setup.topic_name),
        setup.frame_id,
    )
    return setup


def _make_point_cloud2(points: np.ndarray, frame_id: str, stamp_seconds: float):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    contiguous_points = np.ascontiguousarray(points, dtype=np.float32)

    header = Header()
    header.stamp = seconds_to_ros_time_msg(stamp_seconds)
    header.frame_id = frame_id

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(contiguous_points.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = contiguous_points.tobytes()
    message.is_dense = True
    return message


def _iter_mid360_scan_angles(
    scan_index: int,
    points_per_scan: int,
    elevations_rad: np.ndarray,
):
    """Yield a deterministic 40-channel 360-degree MID360-like scan pattern."""
    if points_per_scan <= 0 or elevations_rad.size == 0:
        return

    channel_count = int(elevations_rad.size)
    columns = max(1, math.ceil(points_per_scan / channel_count))
    phase = (scan_index * MID360_FRAME_PHASE_RAD) % (2.0 * math.pi)
    emitted = 0

    for column_index in range(columns):
        base_azimuth = (2.0 * math.pi * column_index / columns + phase) % (2.0 * math.pi)
        for channel_index, elevation in enumerate(elevations_rad):
            if emitted >= points_per_scan:
                return
            azimuth = (
                base_azimuth
                + (2.0 * math.pi * channel_index / (columns * channel_count))
            ) % (2.0 * math.pi)
            yield float(elevation), float(azimuth)
            emitted += 1


def _enable_extension(extension_id: str) -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_id):
        manager.set_extension_enabled_immediate(extension_id, True)

def _find_parent_prim(stage, robot_prim_path: str, parent_link_name: str):
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim or not robot_prim.IsValid():
        return None

    exact_path = f"{robot_prim_path.rstrip('/')}/{parent_link_name}"
    exact_prim = stage.GetPrimAtPath(exact_path)
    if exact_prim and exact_prim.IsValid():
        return exact_prim

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(robot_prim_path.rstrip("/") + "/"):
            continue
        if prim.GetName() == parent_link_name:
            return prim
    return None


def _clean_prim_name(value: str) -> str:
    cleaned = value.strip().strip("/")
    if not cleaned:
        raise ValueError("USD prim name cannot be empty.")
    if "/" in cleaned:
        raise ValueError(f"Expected a USD prim name, not a path: {value!r}")
    return cleaned


def _optional_isaac_string(value: str) -> str | None:
    normalized = value.strip()
    if normalized.lower() in {"", "none", "null"}:
        return None
    return normalized


def _radians_to_degrees(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(math.degrees(value) for value in values)


def _strip_ros_topic_prefix(topic_name: str) -> str:
    return topic_name.strip().lstrip("/")
