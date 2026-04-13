"""ROS 2 runtime helpers for Isaac Sim-native publishers."""

from __future__ import annotations

import os
from pathlib import Path
import sys

from runtime_logging import get_logger


LOGGER = get_logger("ros2_bridge_runtime")


def prepare_isaac_ros2_bridge_environment(domain_id: int) -> None:
    """Keep Isaac's Python 3.11 ROS bridge away from sourced ROS 3.10 paths."""
    bridge_humble_root = _find_isaac_ros2_bridge_humble_root()
    if bridge_humble_root is not None:
        bridge_lib = bridge_humble_root / "lib"
        _prepend_env_path("LD_LIBRARY_PATH", str(bridge_lib))

    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)

    python_path_entries = os.environ.get("PYTHONPATH", "").split(os.pathsep)
    filtered_entries = [
        entry
        for entry in python_path_entries
        if entry and not is_incompatible_ros_python_path(entry)
    ]
    removed_count = len([entry for entry in python_path_entries if entry]) - len(filtered_entries)
    if removed_count:
        os.environ["PYTHONPATH"] = os.pathsep.join(filtered_entries)
        sys.path[:] = [
            entry
            for entry in sys.path
            if not is_incompatible_ros_python_path(entry)
        ]
        LOGGER.info(
            "removed %s ROS Python 3.10 path(s) before enabling Isaac ROS 2 bridge; "
            "using RMW_IMPLEMENTATION=%s ROS_DOMAIN_ID=%s",
            removed_count,
            os.environ["RMW_IMPLEMENTATION"],
            os.environ["ROS_DOMAIN_ID"],
        )
    else:
        LOGGER.info(
            "prepared Isaac ROS 2 bridge environment with RMW_IMPLEMENTATION=%s ROS_DOMAIN_ID=%s",
            os.environ["RMW_IMPLEMENTATION"],
            os.environ["ROS_DOMAIN_ID"],
        )


def enable_isaac_ros2_bridge_extension() -> None:
    """Enable Isaac's ROS 2 bridge extension before importing ROS Python modules."""
    import omni.kit.app

    extension_id = "isaacsim.ros2.bridge"
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(extension_id):
        manager.set_extension_enabled_immediate(extension_id, True)


def is_incompatible_ros_python_path(path_value: str) -> bool:
    normalized = path_value.replace("\\", "/")
    return (
        "/python3.10/" in normalized
        and (
            normalized.startswith("/opt/ros/")
            or "/Workspaces/ros2_ws/install/" in normalized
            or "/Workspaces/unitree_ros2/" in normalized
        )
    )


def _find_isaac_ros2_bridge_humble_root() -> Path | None:
    try:
        import isaacsim.ros2.bridge as ros2_bridge
    except ImportError:
        return None
    bridge_file = Path(ros2_bridge.__file__).resolve()
    extension_root = bridge_file.parents[3]
    humble_root = extension_root / "humble"
    return humble_root if humble_root.exists() else None


def _prepend_env_path(name: str, value: str) -> None:
    entries = [entry for entry in os.environ.get(name, "").split(os.pathsep) if entry]
    if value in entries:
        return
    os.environ[name] = os.pathsep.join([value, *entries])
