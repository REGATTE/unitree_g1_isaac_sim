"""Follow-camera helpers for the Unitree G1 Isaac Sim app."""

from __future__ import annotations

import math

from config import AppConfig
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger


LOGGER = get_logger("camera")


class FollowCameraController:
    """Keep an Isaac Sim camera positioned behind the robot base."""

    def __init__(self, config: AppConfig) -> None:
        self._camera_prim_path = config.follow_camera_prim_path
        self._distance = config.follow_camera_distance
        self._height = config.follow_camera_height
        self._target_height = config.follow_camera_target_height
        self._set_camera_view = None
        self._warned_update_unavailable = False

    def initialize(self) -> None:
        """Create the camera prim and make it the active viewport camera."""
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, self._camera_prim_path)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
        camera.CreateFocalLengthAttr(24.0)

        self._set_camera_view = _import_set_camera_view()
        _set_active_viewport_camera(self._camera_prim_path)
        LOGGER.info("follow camera initialized at %s", self._camera_prim_path)

    def update(self, snapshot: RobotKinematicSnapshot) -> None:
        """Place the camera behind the robot and aim it at the upper body."""
        if self._set_camera_view is None:
            if not self._warned_update_unavailable:
                LOGGER.warning("follow camera update skipped because set_camera_view is unavailable")
                self._warned_update_unavailable = True
            return

        base_x, base_y, base_z = snapshot.base_position_world[:3]
        yaw = _yaw_from_quaternion_wxyz(snapshot.base_quaternion_wxyz)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        eye = (
            base_x - forward_x * self._distance,
            base_y - forward_y * self._distance,
            base_z + self._height,
        )
        target = (
            base_x,
            base_y,
            base_z + self._target_height,
        )
        self._set_camera_view(
            eye=eye,
            target=target,
            camera_prim_path=self._camera_prim_path,
        )


def _import_set_camera_view():
    try:
        from isaacsim.core.utils.viewports import set_camera_view
    except ImportError:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view
        except ImportError:
            LOGGER.warning("set_camera_view helper is unavailable; follow camera updates are disabled")
            return None
    return set_camera_view


def _set_active_viewport_camera(camera_prim_path: str) -> None:
    try:
        from omni.kit.viewport.utility import get_active_viewport
    except ImportError:
        LOGGER.warning("active viewport helper is unavailable; follow camera prim was still created")
        return

    viewport = get_active_viewport()
    if viewport is None:
        LOGGER.warning("active viewport is unavailable; follow camera prim was still created")
        return
    viewport.camera_path = camera_prim_path


def _yaw_from_quaternion_wxyz(quaternion_wxyz) -> float:
    w, x, y, z = quaternion_wxyz[:4]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
