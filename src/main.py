#!/usr/bin/env python3
"""Thin Isaac Sim entrypoint for the Unitree G1 simulator.

This module intentionally stays focused on application orchestration:
it starts Isaac Sim, builds the scene, prepares the simulation world,
validates that robot state can be read, and then owns the main loop.
"""

from __future__ import annotations

import sys
import traceback

from config import PROJECT_ROOT, AppConfig, parse_config
from mapping import log_joint_validation_report, to_dds_ordered_snapshot, validate_live_joint_order
from robot_state import RobotStateReader, log_joint_state
from scene import build_scene


def load_simulation_app(config: AppConfig):
    """Start the Isaac Sim application using the current runtime config."""
    try:
        from isaacsim import SimulationApp  # Isaac Sim 5.x pip/binary path.
    except ImportError:
        from omni.isaac.kit import SimulationApp  # Isaac Sim 4.x compatibility path.
    return SimulationApp(config.simulation_app_config)


def create_world(config: AppConfig):
    """Create and reset the Isaac Sim World so physics views are valid.

    The articulation wrapper depends on an initialized simulation view.
    Resetting the world here ensures the physics backend is ready before
    any robot-state object calls ``Articulation.initialize()``.
    """
    try:
        from isaacsim.core.api import World
    except ImportError:
        from omni.isaac.core import World

    print("[unitree_g1_isaac_sim] creating simulation world")
    world = World(stage_units_in_meters=1.0, physics_dt=config.physics_dt)
    print("[unitree_g1_isaac_sim] resetting simulation world")
    world.reset()
    return world


def run_main_loop(simulation_app, world, max_frames: int, headless: bool) -> None:
    """Advance the simulator until the app closes or the frame cap is reached."""

    frame_count = 0
    try:
        while simulation_app.is_running():
            # Use World.step so the simulation context, physics, and rendering stay aligned.
            world.step(render=not headless)
            frame_count += 1
            if max_frames > 0 and frame_count >= max_frames:
                break
    except KeyboardInterrupt:
        pass


def initialize_robot_state_reader(config: AppConfig) -> RobotStateReader:
    """Create the articulation reader and print a one-time joint-state snapshot."""

    state_reader = RobotStateReader(config.robot_prim_path)
    state_reader.initialize()
    sim_snapshot = state_reader.read_snapshot()
    validation_report = validate_live_joint_order(sim_snapshot.joint_names)
    log_joint_validation_report(validation_report)
    log_joint_state(
        sim_snapshot,
        limit=None if config.print_all_joints else 12,
        order_label="sim",
    )

    # Print a short DDS-ordered preview so the next DDS publisher can rely on
    # the exact same reordered state path already exercised at startup.
    dds_snapshot = to_dds_ordered_snapshot(sim_snapshot)
    log_joint_state(
        dds_snapshot,
        limit=8,
        order_label="dds",
    )
    return state_reader


def main() -> int:
    config = parse_config()

    try:
        asset_path = config.resolve_asset_path()
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1

    simulation_app = load_simulation_app(config)
    try:
        print(f"[unitree_g1_isaac_sim] project_root={PROJECT_ROOT}")
        print(f"[unitree_g1_isaac_sim] asset_path={asset_path}")
        build_scene(asset_path, config)
        world = create_world(config)
        initialize_robot_state_reader(config)
        run_main_loop(simulation_app, world, config.max_frames, config.headless)
    except Exception:
        print("[unitree_g1_isaac_sim] fatal error during startup/runtime", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
