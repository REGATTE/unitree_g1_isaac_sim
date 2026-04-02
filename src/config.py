"""Runtime configuration for the Unitree G1 Isaac Sim launcher."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODELS_ROOT = PROJECT_ROOT / "Models" / "USD"
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
    lowstate_topic: str
    lowcmd_topic: str
    lowstate_publish_hz: float
    enable_lowcmd_subscriber: bool
    lowcmd_timeout_seconds: float
    lowstate_cadence_report_interval: int
    lowstate_cadence_warn_ratio: float

    def resolve_asset_path(self) -> Path:
        asset_path = self.asset_path or DEFAULT_ASSET_BY_VARIANT[self.robot_variant]
        asset_path = asset_path.expanduser().resolve()
        if not asset_path.exists():
            raise FileNotFoundError(f"Robot USD asset does not exist: {asset_path}")
        return asset_path

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
        "--physics-dt",
        type=positive_finite_float,
        default=1.0 / 120.0,
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
        action="store_true",
        help=(
            "Enable the Cyclone DDS bridge so external Unitree SDK or ROS 2 "
            "clients can treat the simulator like a real robot."
        ),
    )
    parser.add_argument(
        "--dds-domain-id",
        type=int,
        default=1,
        help="Cyclone DDS domain id passed to the Unitree SDK channel factory.",
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
        default=100.0,
        help="Target DDS publish rate in Hz for `rt/lowstate`.",
    )
    parser.add_argument(
        "--enable-lowcmd-subscriber",
        action="store_true",
        help=(
            "Create the `rt/lowcmd` subscriber and apply accepted body commands "
            "into the live articulation."
        ),
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
    return parser


def parse_config(argv: list[str] | None = None) -> AppConfig:
    args = build_arg_parser().parse_args(argv)
    return AppConfig(
        robot_variant=args.robot_variant,
        asset_path=args.asset_path,
        robot_prim_path=args.robot_prim_path,
        robot_height=args.robot_height,
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
        lowstate_topic=args.lowstate_topic,
        lowcmd_topic=args.lowcmd_topic,
        lowstate_publish_hz=args.lowstate_publish_hz,
        enable_lowcmd_subscriber=args.enable_lowcmd_subscriber,
        lowcmd_timeout_seconds=args.lowcmd_timeout_seconds,
        lowstate_cadence_report_interval=args.lowstate_cadence_report_interval,
        lowstate_cadence_warn_ratio=args.lowstate_cadence_warn_ratio,
    )
