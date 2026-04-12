"""Isaac RTX LiDAR setup for the G1-mounted Livox MID360.

The real robot ROS 2 path launches ``livox_ros_driver2`` directly and publishes
``sensor_msgs/msg/PointCloud2`` with ``frame_id=mid360_link``. This module keeps
the simulator aligned with that contract by publishing from Isaac's ROS 2 RTX
LiDAR helper directly, instead of routing point clouds through the Unitree
LowState / LowCmd sidecar bridge.

The MID360 scan attributes are an approximation adapted from a public NVIDIA
forum snippet for Isaac Sim 5.0. Livox does not currently ship an official
Isaac RTX profile for the MID360, so this should be treated as a practical
navigation-development model rather than a sensor-calibration authority.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any
import sys

from config import AppConfig
from runtime_logging import get_logger


LOGGER = get_logger("sensors.livox_mid360")


# Mirrors g1_29dof_description/urdf/g1_29dof.urdf:
#   <joint name="mid360_joint" type="fixed">
#     <origin xyz="0.0002835 0.00003 0.40618" rpy="0 0.04014257279586953 0"/>
#     <parent link="torso_link"/>
#     <child link="mid360_link"/>
#   </joint>
MID360_TRANSLATION_IN_TORSO = (0.0002835, 0.00003, 0.40618)
MID360_RPY_IN_TORSO = (0.0, 0.04014257279586953, 0.0)


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
    "omni:sensor:Core:scanRateBaseHz": 20.0,
    "omni:sensor:Core:reportRateBaseHz": 7761,
    "omni:sensor:Core:numberOfEmitters": 40,
    "omni:sensor:Core:numberOfChannels": 40,
    "omni:sensor:Core:rangeOffset": 0.03,
    "omni:sensor:Core:intensityMappingType": "LINEAR",
    "omni:sensor:Core:emitterState:s001:azimuthDeg": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:elevationDeg": [
        -7.0,
        -5.525,
        -4.050,
        -2.575,
        -1.1004,
        0.374,
        1.849,
        3.324,
        4.799,
        6.274,
        7.7494,
        9.2249,
        10.699,
        12.174,
        13.645,
        15.1243,
        16.5999,
        18.074,
        19.5499,
        21.024,
        22.493,
        23.9749,
        25.44,
        26.924,
        28.39,
        29.8743,
        31.3499,
        32.824,
        34.29,
        35.774,
        37.2486,
        38.724,
        40.19,
        41.674,
        43.14,
        44.624,
        46.09,
        47.574,
        49.048,
        50.524,
    ],
    "omni:sensor:Core:emitterState:s001:fireTimeNs": [index * 1000 for index in range(40)],
    "omni:sensor:Core:emitterState:s001:distanceCorrectionM": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:focalDistM": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:focalSlope": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:horOffsetM": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:reportRateDiv": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:vertOffsetM": [0.0] * 40,
    "omni:sensor:Core:emitterState:s001:channelId": list(range(1, 41)),
}


@dataclass(frozen=True)
class LivoxMid360Setup:
    """Details of the Isaac-native MID360 sensor path."""

    frame_prim_path: str
    sensor_prim_path: str
    topic_name: str
    frame_id: str


def setup_livox_mid360(config: AppConfig) -> LivoxMid360Setup | None:
    """Create the simulated MID360 and attach a ROS 2 PointCloud2 writer."""
    if not config.enable_livox_lidar:
        LOGGER.info("simulated Livox MID360 LiDAR disabled")
        return None

    import omni
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    _prepare_isaac_ros2_bridge_environment()
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

    hydra_texture = rep.create.render_product(sensor.GetPath(), resolution=(128, 128), name="MID360")
    omni.kit.app.get_app().update()
    _create_ros2_point_cloud_graph(
        render_product_path=hydra_texture.path,
        topic_name=config.livox_lidar_topic,
        frame_id=config.livox_lidar_frame_id,
    )

    setup = LivoxMid360Setup(
        frame_prim_path=frame_prim_path,
        sensor_prim_path=str(sensor.GetPath()),
        topic_name=config.livox_lidar_topic,
        frame_id=config.livox_lidar_frame_id,
    )
    LOGGER.info(
        "created simulated Livox MID360 RTX LiDAR at %s; publishing PointCloud2 on '/%s' with frame_id='%s'",
        setup.sensor_prim_path,
        _strip_ros_topic_prefix(setup.topic_name),
        setup.frame_id,
    )
    return setup


def _enable_extension(extension_id: str) -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_id):
        manager.set_extension_enabled_immediate(extension_id, True)


def _create_ros2_point_cloud_graph(
    *,
    render_product_path: str,
    topic_name: str,
    frame_id: str,
) -> None:
    """Publish RTX LiDAR output through the ROS 2 bridge helper node.

    Isaac Sim 5.1's Replicator writer registry can fail in headless mode while
    setting an empty render-product resolution array. The bridge helper node is
    the lower-level graph path used by Isaac's own RTX sensor tests and avoids
    that writer attach path.
    """
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": "/ActionGraph/LivoxMid360Lidar", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PointCloudPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PointCloudPublish.inputs:renderProductPath", render_product_path),
                ("PointCloudPublish.inputs:topicName", _strip_ros_topic_prefix(topic_name)),
                ("PointCloudPublish.inputs:type", "point_cloud"),
                ("PointCloudPublish.inputs:frameId", frame_id),
                ("PointCloudPublish.inputs:resetSimulationTimeOnStop", True),
                ("PointCloudPublish.inputs:fullScan", True),
                ("PointCloudPublish.inputs:useSystemTime", False),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PointCloudPublish.inputs:execIn"),
            ],
        },
    )


def _prepare_isaac_ros2_bridge_environment() -> None:
    """Keep Isaac's Python 3.11 ROS bridge away from sourced ROS 3.10 paths."""
    bridge_humble_root = _find_isaac_ros2_bridge_humble_root()
    if bridge_humble_root is not None:
        bridge_lib = bridge_humble_root / "lib"
        _prepend_env_path("LD_LIBRARY_PATH", str(bridge_lib))

    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    python_path_entries = os.environ.get("PYTHONPATH", "").split(os.pathsep)
    filtered_entries = [
        entry
        for entry in python_path_entries
        if entry and not _is_incompatible_ros_python_path(entry)
    ]
    removed_count = len([entry for entry in python_path_entries if entry]) - len(filtered_entries)
    if removed_count:
        os.environ["PYTHONPATH"] = os.pathsep.join(filtered_entries)
        sys.path[:] = [
            entry
            for entry in sys.path
            if not _is_incompatible_ros_python_path(entry)
        ]
        LOGGER.info(
            "removed %s ROS Python 3.10 path(s) before enabling Isaac ROS 2 bridge; "
            "using RMW_IMPLEMENTATION=%s",
            removed_count,
            os.environ["RMW_IMPLEMENTATION"],
        )


def _find_isaac_ros2_bridge_humble_root():
    from pathlib import Path

    try:
        import isaacsim.ros2.bridge as ros2_bridge
    except ImportError:
        return None
    bridge_file = Path(ros2_bridge.__file__).resolve()
    extension_root = bridge_file.parents[3]
    humble_root = extension_root / "humble"
    return humble_root if humble_root.exists() else None


def _is_incompatible_ros_python_path(path_value: str) -> bool:
    normalized = path_value.replace("\\", "/")
    return (
        "/python3.10/" in normalized
        and (
            normalized.startswith("/opt/ros/")
            or "/Workspaces/ros2_ws/install/" in normalized
            or "/Workspaces/unitree_ros2/" in normalized
        )
    )


def _prepend_env_path(name: str, value: str) -> None:
    entries = [entry for entry in os.environ.get(name, "").split(os.pathsep) if entry]
    if value in entries:
        return
    os.environ[name] = os.pathsep.join([value, *entries])


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
