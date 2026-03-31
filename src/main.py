#!/usr/bin/env python3
"""Thin Isaac Sim entrypoint for the Unitree G1 simulator."""

from __future__ import annotations

import sys
import traceback

from config import PROJECT_ROOT, AppConfig, parse_config
from scene import build_scene


def load_simulation_app(config: AppConfig):
    try:
        from isaacsim import SimulationApp  # Isaac Sim 5.x pip/binary path.
    except ImportError:
        from omni.isaac.kit import SimulationApp  # Isaac Sim 4.x compatibility path.
    return SimulationApp(config.simulation_app_config)

def run_main_loop(simulation_app, max_frames: int) -> None:
    import omni.kit.app
    import omni.timeline

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    frame_count = 0
    try:
        while simulation_app.is_running():
            app.update()
            frame_count += 1
            if max_frames > 0 and frame_count >= max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        timeline.stop()


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
        run_main_loop(simulation_app, config.max_frames)
    except Exception:
        print("[unitree_g1_isaac_sim] fatal error during startup/runtime", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
