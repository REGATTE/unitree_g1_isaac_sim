"""Runtime configuration for the Unitree G1 Isaac Sim launcher."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODELS_ROOT = PROJECT_ROOT / "Models" / "USD"
DEFAULT_WORLD_PATH = PROJECT_ROOT / "Models" / "world" / "Hospital_World.usd"
DEFAULT_VARIANT = "29dof"
DEFAULT_ASSET_BY_VARIANT = {
    "23dof": MODELS_ROOT / "23dof" / "usd" / "g1_23dof_rev_1_0" / "g1_23dof_rev_1_0.usd",
    "29dof": MODELS_ROOT / "29dof" / "usd" / "g1_29dof_rev_1_0" / "g1_29dof_rev_1_0.usd",
}


def positive_finite_float(value: str) -> float:
    """Parse a CLI float argument that must be positive and finite."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected a positive finite float, got {value!r}")
    return parsed


def non_negative_finite_float(value: str) -> float:
    """Parse a CLI float argument that must be finite and non-negative."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected a non-negative finite float, got {value!r}")
    return parsed


def parse_bool(value: str) -> bool:
    """Parse a CLI bool argument from common true/false spellings."""
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"expected true or false, got {value!r}")


@dataclass(frozen=True)
class AppConfig:
    robot_variant: str
    asset_path: Path | None
    robot_prim_path: str
    robot_height: float
    physics_dt: float
    headless: bool
    renderer: str
    width: int
    height: int
    max_frames: int
    reset_after_frames: int
    print_all_joints: bool
    enable_dds: bool
    dds_domain_id: int
    enable_ros2_lowstate: bool
    enable_ros2_lowcmd: bool
    lowstate_topic: str
    lowcmd_topic: str
    lowstate_publish_hz: float
    lowcmd_max_position_delta_rad: float
    enable_native_unitree_lowstate: bool
    enable_native_unitree_lowcmd: bool
    native_unitree_domain_id: int | None
    native_unitree_lowstate_topic: str
    native_unitree_lowcmd_topic: str
    native_unitree_bridge_exe: Path
    lowcmd_timeout_seconds: float
    lowstate_cadence_report_interval: int
    lowstate_cadence_warn_ratio: float
    unitree_ros2_install_prefix: Path | None
    ros2_python_exe: str
    bridge_bind_host: str
    bridge_lowstate_port: int
    bridge_lowcmd_port: int
    enable_livox_lidar: bool = True
    livox_lidar_topic: str = "livox/lidar"
    livox_lidar_frame_id: str = "mid360_link"
    livox_lidar_parent_link_name: str = "torso_link"
    livox_lidar_prim_name: str = "mid360_link"
    livox_lidar_sensor_prim_name: str = "mid360_rtx_lidar"
    livox_lidar_rtx_config: str = "OS1"
    livox_lidar_rtx_variant: str = "OS1_REV6_32ch20hz1024res"
    use_world: bool = False
    world_path: Path = DEFAULT_WORLD_PATH
    world_prim_path: str = "/World/Environment"
    enable_follow_camera: bool = True
    follow_camera_prim_path: str = "/World/FollowCamera"
    follow_camera_distance: float = 4.0
    follow_camera_height: float = 0.6
    follow_camera_target_height: float = 0.3

    def resolve_asset_path(self) -> Path:
        asset_path = self.asset_path or DEFAULT_ASSET_BY_VARIANT[self.robot_variant]
        asset_path = asset_path.expanduser().resolve()
        if not asset_path.exists():
            raise FileNotFoundError(f"Robot USD asset does not exist: {asset_path}")
        return asset_path

    def resolve_world_path(self) -> Path | None:
        if not self.use_world:
            return None
        world_path = self.world_path.expanduser().resolve()
        if not world_path.exists():
            raise FileNotFoundError(f"World USD asset does not exist: {world_path}")
        return world_path

    @property
    def simulation_app_config(self) -> dict[str, object]:
        return {
            "headless": self.headless,
            "renderer": self.renderer,
            "width": self.width,
            "height": self.height,
        }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch the Unitree G1 Isaac Sim application.")
    parser.add_argument(
        "--robot-variant",
        choices=sorted(DEFAULT_ASSET_BY_VARIANT),
        default=DEFAULT_VARIANT,
        help="Robot asset variant to load.",
    )
    parser.add_argument(
        "--asset-path",
        type=Path,
        default=None,
        help="Override the default robot USD path.",
    )
    parser.add_argument(
        "--robot-prim-path",
        type=str,
        default="/World/G1",
        help="Prim path where the robot USD will be referenced.",
    )
    parser.add_argument(
        "--robot-height",
        type=float,
        default=0.8,
        help="Spawn height for the robot root prim in meters.",
    )
    parser.add_argument(
        "--use-world",
        type=parse_bool,
        nargs="?",
        const=True,
        default=False,
        help=(
            "Reference the configured world USD into the stage. Accepts true/false; "
            "passing --use-world without a value is treated as true."
        ),
    )
    parser.add_argument(
        "--world-path",
        type=Path,
        default=DEFAULT_WORLD_PATH,
        help="World USD path used when --use-world true is set.",
    )
    parser.add_argument(
        "--world-prim-path",
        type=str,
        default="/World/Environment",
        help="Prim path where the optional world USD will be referenced.",
    )
    parser.add_argument(
        "--enable-follow-camera",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Keep the active viewport camera following the robot base. "
            "Enabled by default."
        ),
    )
    parser.add_argument(
        "--follow-camera-prim-path",
        type=str,
        default="/World/FollowCamera",
        help="Prim path for the camera that follows the robot.",
    )
    parser.add_argument(
        "--follow-camera-distance",
        type=positive_finite_float,
        default=4.0,
        help="Distance in meters behind the robot for the follow camera.",
    )
    parser.add_argument(
        "--follow-camera-height",
        type=positive_finite_float,
        default=0.6,
        help="Height offset in meters above the robot base for the follow camera.",
    )
    parser.add_argument(
        "--follow-camera-target-height",
        type=non_negative_finite_float,
        default=0.3,
        help="Height offset in meters above the robot base that the follow camera looks at.",
    )
    parser.add_argument(
        "--physics-dt",
        type=positive_finite_float,
        default=1.0 / 500.0,
        help="Physics timestep in seconds.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim without the GUI window.",
    )
    parser.add_argument(
        "--renderer",
        type=str,
        default="RayTracedLighting",
        help="Renderer passed to Isaac Sim.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Window width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Window height.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N Kit frames. Use 0 to run until the app closes.",
    )
    parser.add_argument(
        "--reset-after-frames",
        type=int,
        default=0,
        help=(
            "Trigger one deterministic runtime reset after N simulation frames. "
            "Use 0 to disable. Primarily intended for reset validation."
        ),
    )
    parser.add_argument(
        "--print-all-joints",
        action="store_true",
        help="Print the full articulation joint list during startup validation.",
    )
    parser.add_argument(
        "--enable-dds",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Enable the Cyclone DDS bridge so external Unitree SDK or ROS 2 "
            "clients can treat the simulator like a real robot. Enabled by default."
        ),
    )
    parser.add_argument(
        "--dds-domain-id",
        type=int,
        default=1,
        help="Cyclone DDS domain id passed to the Unitree SDK channel factory.",
    )
    parser.add_argument(
        "--enable-ros2-lowstate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish ROS 2 `rt/lowstate` through the existing sidecar path. Enabled by default.",
    )
    parser.add_argument(
        "--enable-ros2-lowcmd",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Accept ROS 2 `rt/lowcmd` through the existing sidecar path. Disabled by default.",
    )
    parser.add_argument(
        "--lowstate-topic",
        type=str,
        default="rt/lowstate",
        help="DDS topic used for low-level robot state publication.",
    )
    parser.add_argument(
        "--lowcmd-topic",
        type=str,
        default="rt/lowcmd",
        help="DDS topic used for low-level robot command subscription.",
    )
    parser.add_argument(
        "--lowstate-publish-hz",
        type=positive_finite_float,
        default=500.0,
        help="Target DDS publish rate in Hz for `rt/lowstate`.",
    )
    parser.add_argument(
        "--lowcmd-max-position-delta-rad",
        type=float,
        default=0.25,
        help=(
            "Maximum allowed absolute joint-position delta, in radians, between "
            "the current simulator pose and an incoming `rt/lowcmd` target. "
            "Use 0 to disable bounded-motion rejection."
        ),
    )
    parser.add_argument(
        "--enable-native-unitree-lowstate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable native Unitree SDK `rt/lowstate` publication. Enabled by default.",
    )
    parser.add_argument(
        "--enable-native-unitree-lowcmd",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable native Unitree SDK `rt/lowcmd` ingress. Enabled by default.",
    )
    parser.add_argument(
        "--native-unitree-domain-id",
        type=int,
        default=None,
        help="DDS domain id for the native Unitree SDK bridge. Defaults to --dds-domain-id.",
    )
    parser.add_argument(
        "--native-unitree-lowstate-topic",
        type=str,
        default="rt/lowstate",
        help="Topic used for native Unitree SDK lowstate publication.",
    )
    parser.add_argument(
        "--native-unitree-lowcmd-topic",
        type=str,
        default="rt/lowcmd",
        help="Topic used for native Unitree SDK lowcmd subscription.",
    )
    parser.add_argument(
        "--native-unitree-bridge-exe",
        type=Path,
        default=PROJECT_ROOT / "native_sdk_bridge" / "build" / "unitree_g1_native_bridge",
        help="Path to the compiled native Unitree SDK bridge sidecar executable.",
    )
    parser.add_argument(
        "--lowcmd-timeout-seconds",
        type=float,
        default=0.5,
        help=(
            "How long a cached `rt/lowcmd` sample stays fresh before the runtime "
            "treats it as stale and stops reapplying it. Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--lowstate-cadence-report-interval",
        type=int,
        default=500,
        help=(
            "How many `rt/lowstate` publishes to accumulate before reporting the "
            "observed simulation-time publish cadence. Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--lowstate-cadence-warn-ratio",
        type=float,
        default=0.05,
        help=(
            "Relative tolerance for cadence diagnostics. If the observed "
            "`rt/lowstate` rate differs from the configured rate by more than "
            "this ratio, the runtime emits a warning instead of an info log."
        ),
    )
    parser.add_argument(
        "--unitree-ros2-install-prefix",
        type=Path,
        default=None,
        help=(
            "Path to the built `unitree_ros2/cyclonedds_ws/install` prefix. "
            "If omitted, the runtime also checks the UNITREE_ROS2_INSTALL_PREFIX "
            "environment variable and the default ~/Workspaces/unitree_ros2 "
            "location."
        ),
    )
    parser.add_argument(
        "--ros2-python-exe",
        type=str,
        default=os.environ.get("ROS2_PYTHON_EXE", "/usr/bin/python3"),
        help="Python executable used for the ROS 2 sidecar bridge process.",
    )
    parser.add_argument(
        "--bridge-bind-host",
        type=str,
        default="127.0.0.1",
        help="Localhost interface used for Isaac Sim <-> ROS 2 sidecar UDP traffic.",
    )
    parser.add_argument(
        "--bridge-lowstate-port",
        type=int,
        default=35501,
        help="UDP port used for Isaac Sim -> sidecar lowstate packets.",
    )
    parser.add_argument(
        "--bridge-lowcmd-port",
        type=int,
        default=35502,
        help="UDP port used for sidecar -> Isaac Sim lowcmd packets.",
    )
    parser.add_argument(
        "--enable-livox-lidar",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Create an Isaac RTX LiDAR that approximates the inverted Livox "
            "MID360 and publishes PointCloud2 through the Isaac ROS 2 bridge. "
            "Enabled by default."
        ),
    )
    parser.add_argument(
        "--livox-lidar-topic",
        type=str,
        default="livox/lidar",
        help="ROS 2 PointCloud2 topic for the simulated MID360 LiDAR.",
    )
    parser.add_argument(
        "--livox-lidar-frame-id",
        type=str,
        default="mid360_link",
        help="Frame id used in the simulated MID360 PointCloud2 messages.",
    )
    parser.add_argument(
        "--livox-lidar-parent-link-name",
        type=str,
        default="torso_link",
        help=(
            "USD prim name to search for when mounting the simulated MID360. "
            "The default mirrors the URDF mid360_joint parent."
        ),
    )
    parser.add_argument(
        "--livox-lidar-prim-name",
        type=str,
        default="mid360_link",
        help=(
            "USD Xform name created under the parent link for the simulated "
            "MID360 frame. The default mirrors the URDF child link."
        ),
    )
    parser.add_argument(
        "--livox-lidar-sensor-prim-name",
        type=str,
        default="mid360_rtx_lidar",
        help="USD prim name for the RTX LiDAR sensor under the MID360 frame.",
    )
    parser.add_argument(
        "--livox-lidar-rtx-config",
        type=str,
        default="OS1",
        help=(
            "Base Isaac RTX LiDAR config used for the MID360 approximation. "
            "Use 'none' to create a generic OmniLidar with only explicit attributes."
        ),
    )
    parser.add_argument(
        "--livox-lidar-rtx-variant",
        type=str,
        default="OS1_REV6_32ch20hz1024res",
        help=(
            "RTX LiDAR config variant used with --livox-lidar-rtx-config. "
            "Use 'none' to pass no variant."
        ),
    )
    return parser


def resolve_unitree_ros2_install_prefix(cli_value: Path | None) -> Path | None:
    """Resolve the Unitree ROS 2 install prefix from CLI, env, or a default path."""
    candidates: list[Path] = []
    if cli_value is not None:
        candidates.append(cli_value)

    env_value = os.environ.get("UNITREE_ROS2_INSTALL_PREFIX")
    if env_value:
        candidates.append(Path(env_value))

    workspace_root = next(
        (
            parent
            for parent in PROJECT_ROOT.parents
            if parent.name == "src" and parent.parent != parent
        ),
        None,
    )
    if workspace_root is not None:
        candidates.append(workspace_root.parent / "unitree_ros2" / "cyclonedds_ws" / "install")
    candidates.append(Path.home() / "Workspaces" / "unitree_ros2" / "cyclonedds_ws" / "install")

    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.exists():
            return resolved
    return None


def parse_config(argv: list[str] | None = None) -> AppConfig:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    physics_hz = 1.0 / args.physics_dt
    if args.lowstate_publish_hz - physics_hz > 1e-9:
        parser.error(
            "--lowstate-publish-hz cannot exceed the physics rate derived from "
            f"--physics-dt. requested={args.lowstate_publish_hz:.3f}Hz "
            f"physics_rate={physics_hz:.3f}Hz"
        )
    if args.lowcmd_max_position_delta_rad < 0.0 or not math.isfinite(args.lowcmd_max_position_delta_rad):
        parser.error("--lowcmd-max-position-delta-rad must be a finite non-negative float.")
    if args.enable_ros2_lowcmd and args.enable_native_unitree_lowcmd:
        parser.error(
            "multiple lowcmd sources enabled: ROS 2 sidecar and native Unitree SDK. "
            "Enable only one lowcmd source at a time."
        )
    for label, value in (
        ("--livox-lidar-topic", args.livox_lidar_topic),
        ("--livox-lidar-frame-id", args.livox_lidar_frame_id),
        ("--livox-lidar-parent-link-name", args.livox_lidar_parent_link_name),
        ("--livox-lidar-prim-name", args.livox_lidar_prim_name),
        ("--livox-lidar-sensor-prim-name", args.livox_lidar_sensor_prim_name),
        ("--lowstate-topic", args.lowstate_topic),
        ("--lowcmd-topic", args.lowcmd_topic),
        ("--native-unitree-lowstate-topic", args.native_unitree_lowstate_topic),
        ("--native-unitree-lowcmd-topic", args.native_unitree_lowcmd_topic),
    ):
        if not value.strip():
            parser.error(f"{label} cannot be empty.")
    return AppConfig(
        robot_variant=args.robot_variant,
        asset_path=args.asset_path,
        robot_prim_path=args.robot_prim_path,
        robot_height=args.robot_height,
        use_world=args.use_world,
        world_path=args.world_path,
        world_prim_path=args.world_prim_path,
        enable_follow_camera=args.enable_follow_camera,
        follow_camera_prim_path=args.follow_camera_prim_path,
        follow_camera_distance=args.follow_camera_distance,
        follow_camera_height=args.follow_camera_height,
        follow_camera_target_height=args.follow_camera_target_height,
        physics_dt=args.physics_dt,
        headless=args.headless,
        renderer=args.renderer,
        width=args.width,
        height=args.height,
        max_frames=args.max_frames,
        reset_after_frames=args.reset_after_frames,
        print_all_joints=args.print_all_joints,
        enable_dds=args.enable_dds,
        dds_domain_id=args.dds_domain_id,
        enable_ros2_lowstate=args.enable_ros2_lowstate,
        enable_ros2_lowcmd=args.enable_ros2_lowcmd,
        lowstate_topic=args.lowstate_topic,
        lowcmd_topic=args.lowcmd_topic,
        lowstate_publish_hz=args.lowstate_publish_hz,
        lowcmd_max_position_delta_rad=args.lowcmd_max_position_delta_rad,
        enable_native_unitree_lowstate=args.enable_native_unitree_lowstate,
        enable_native_unitree_lowcmd=args.enable_native_unitree_lowcmd,
        native_unitree_domain_id=args.native_unitree_domain_id or args.dds_domain_id,
        native_unitree_lowstate_topic=args.native_unitree_lowstate_topic,
        native_unitree_lowcmd_topic=args.native_unitree_lowcmd_topic,
        native_unitree_bridge_exe=args.native_unitree_bridge_exe,
        lowcmd_timeout_seconds=args.lowcmd_timeout_seconds,
        lowstate_cadence_report_interval=args.lowstate_cadence_report_interval,
        lowstate_cadence_warn_ratio=args.lowstate_cadence_warn_ratio,
        unitree_ros2_install_prefix=resolve_unitree_ros2_install_prefix(args.unitree_ros2_install_prefix),
        ros2_python_exe=args.ros2_python_exe,
        bridge_bind_host=args.bridge_bind_host,
        bridge_lowstate_port=args.bridge_lowstate_port,
        bridge_lowcmd_port=args.bridge_lowcmd_port,
        enable_livox_lidar=args.enable_livox_lidar,
        livox_lidar_topic=args.livox_lidar_topic,
        livox_lidar_frame_id=args.livox_lidar_frame_id,
        livox_lidar_parent_link_name=args.livox_lidar_parent_link_name,
        livox_lidar_prim_name=args.livox_lidar_prim_name,
        livox_lidar_sensor_prim_name=args.livox_lidar_sensor_prim_name,
        livox_lidar_rtx_config=args.livox_lidar_rtx_config,
        livox_lidar_rtx_variant=args.livox_lidar_rtx_variant,
    )
