#!/usr/bin/env python3
"""Thin Isaac Sim entrypoint for the Unitree G1 simulator.

This module intentionally stays focused on application orchestration:
it starts Isaac Sim, builds the scene, prepares the simulation world,
validates that robot state can be read, and then owns the main loop.
"""

from __future__ import annotations

import logging

from config import PROJECT_ROOT, AppConfig, parse_config
from dds import DdsManager
from mapping import log_joint_validation_report, to_dds_ordered_snapshot, validate_live_joint_order
from robot_control import RobotCommandApplier
from robot_state import (
    JointStateSnapshot,
    PhysicsViewUnavailableError,
    RobotStateReader,
    log_joint_state,
    log_kinematic_snapshot,
)
from runtime_logging import configure_logging, get_logger
from scene import build_scene

LOGGER = get_logger("main")


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

    LOGGER.info("creating simulation world")
    world = World(stage_units_in_meters=1.0, physics_dt=config.physics_dt)
    LOGGER.info("resetting simulation world")
    world.reset()
    return world


def perform_runtime_reset(
    world,
    state_reader: RobotStateReader,
    dds_manager: DdsManager | None,
) -> None:
    """Reset the live runtime back to the canonical deterministic state."""
    LOGGER.info("triggering deterministic runtime reset")
    world.reset()
    state_reader.reinitialize_after_world_reset()
    state_reader.apply_deterministic_reset_state()
    if dds_manager is not None:
        dds_manager.reset_runtime_state()
    LOGGER.info("deterministic runtime reset complete")


def run_main_loop(
    simulation_app,
    world,
    max_frames: int,
    headless: bool,
    physics_dt: float,
    reset_after_frames: int,
    state_reader: RobotStateReader,
    dds_manager: DdsManager | None,
    command_applier: RobotCommandApplier | None,
) -> None:
    """Advance the simulator until the app closes or the frame cap is reached."""

    frame_count = 0
    simulation_time_seconds = 0.0
    reset_triggered = False
    try:
        while simulation_app.is_running():
            # Apply the latest cached low-level command before stepping physics
            # so the next simulator frame reflects the current DDS input.
            if dds_manager is not None and command_applier is not None:
                command_applier.apply_lowcmd(dds_manager.latest_lowcmd)
            # Use World.step so the simulation context, physics, and rendering stay aligned.
            world.step(render=not headless)
            frame_count += 1
            simulation_time_seconds += physics_dt
            if reset_after_frames > 0 and not reset_triggered and frame_count >= reset_after_frames:
                perform_runtime_reset(world, state_reader, dds_manager)
                simulation_time_seconds = 0.0
                reset_triggered = True
            if dds_manager is not None:
                try:
                    snapshot = state_reader.read_kinematic_snapshot(sample_dt=physics_dt)
                except PhysicsViewUnavailableError:
                    snapshot = None
                if snapshot is not None:
                    dds_manager.step(simulation_time_seconds, snapshot)
            if max_frames > 0 and frame_count >= max_frames:
                break
    except KeyboardInterrupt:
        pass


def initialize_robot_state_reader(config: AppConfig) -> RobotStateReader:
    """Create the articulation reader and print a one-time joint-state snapshot."""

    state_reader = RobotStateReader(config.robot_prim_path)
    state_reader.initialize()
    state_reader.apply_deterministic_startup_state()
    kinematic_snapshot = state_reader.read_kinematic_snapshot(sample_dt=config.physics_dt)
    log_kinematic_snapshot(kinematic_snapshot)
    sim_snapshot = JointStateSnapshot(
        joint_names=list(kinematic_snapshot.joint_names),
        joint_positions=list(kinematic_snapshot.joint_positions),
        joint_velocities=list(kinematic_snapshot.joint_velocities),
        joint_efforts=list(kinematic_snapshot.joint_efforts) if kinematic_snapshot.joint_efforts is not None else None,
    )
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
    configure_logging(level=logging.INFO)

    try:
        asset_path = config.resolve_asset_path()
    except FileNotFoundError as exc:
        LOGGER.error("%s", exc)
        return 1

    simulation_app = load_simulation_app(config)
    try:
        LOGGER.info("project_root=%s", PROJECT_ROOT)
        LOGGER.info("asset_path=%s", asset_path)
        build_scene(asset_path, config)
        world = create_world(config)
        state_reader = initialize_robot_state_reader(config)
        dds_manager = DdsManager(config) if config.enable_dds else None
        command_applier = (
            RobotCommandApplier(
                state_reader,
                max_position_delta_rad=config.lowcmd_max_position_delta_rad,
            )
            if config.enable_dds
            else None
        )
        if dds_manager is not None:
            dds_manager.initialize()
        run_main_loop(
            simulation_app,
            world,
            config.max_frames,
            config.headless,
            config.physics_dt,
            config.reset_after_frames,
            state_reader,
            dds_manager,
            command_applier,
        )
    except Exception:
        LOGGER.exception("fatal error during startup/runtime")
        return 1
    finally:
        if "dds_manager" in locals() and dds_manager is not None:
            dds_manager.shutdown()
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
