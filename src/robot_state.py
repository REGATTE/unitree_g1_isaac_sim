"""Robot articulation discovery and state access helpers.

This module isolates the first articulation-facing code path for the project.
It exists to answer one question before any DDS work starts:
can the G1 robot be discovered reliably and can its joint state be read?
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointStateSnapshot:
    """Small immutable container for one articulation state sample."""

    joint_names: list[str]
    joint_positions: list[float]
    joint_velocities: list[float]
    joint_efforts: list[float] | None


def import_articulation():
    """Import the articulation wrapper with Isaac Sim 5.x/4.x compatibility."""
    try:
        from isaacsim.core.prims import Articulation
    except ImportError:
        from omni.isaac.core.prims import Articulation
    return Articulation


class RobotStateReader:
    """Thin articulation wrapper for reading simulator joint state."""

    def __init__(self, robot_prim_path: str) -> None:
        Articulation = import_articulation()
        self._robot_prim_path = robot_prim_path
        self._articulation = Articulation(prim_paths_expr=robot_prim_path, name="unitree_g1")
        self._initialized = False

    def initialize(self) -> None:
        if self._initialized:
            return
        self._articulation.initialize()
        self._initialized = True

    @property
    def joint_names(self) -> list[str]:
        names = getattr(self._articulation, "dof_names", None)
        if names is None:
            names = []
        return list(names)

    def read_snapshot(self) -> JointStateSnapshot:
        """Read the current joint-level state from the simulator."""
        if not self._initialized:
            raise RuntimeError("RobotStateReader must be initialized before reading state.")

        joint_positions = self._articulation.get_joint_positions()
        joint_velocities = self._articulation.get_joint_velocities()
        joint_efforts = None
        if hasattr(self._articulation, "get_measured_joint_efforts"):
            joint_efforts = self._articulation.get_measured_joint_efforts()

        return JointStateSnapshot(
            joint_names=self.joint_names,
            joint_positions=_to_float_list(joint_positions),
            joint_velocities=_to_float_list(joint_velocities),
            joint_efforts=_to_float_list(joint_efforts) if joint_efforts is not None else None,
        )


def _to_float_list(values) -> list[float]:
    """Normalize Isaac Sim array-like outputs into a plain Python float list."""
    if values is None:
        return []
    if hasattr(values, "tolist"):
        values = values.tolist()
    # Some Isaac Sim APIs return ``[[...]]`` for a single articulation view.
    if isinstance(values, (list, tuple)) and values and isinstance(values[0], (list, tuple)):
        values = values[0]
    return [float(value) for value in values]


def log_joint_state(
    snapshot: JointStateSnapshot,
    limit: int | None = 12,
    order_label: str | None = None,
) -> None:
    """Print a startup joint-state preview.

    Set ``limit`` to ``None`` to print the full articulation order.
    Use ``order_label`` to distinguish simulator-order and DDS-order logs.
    """
    label_prefix = f"{order_label}_" if order_label else ""
    total = len(snapshot.joint_names)
    print(f"[unitree_g1_isaac_sim] detected {total} {label_prefix}joints")
    visible_joint_count = total if limit is None else min(total, limit)
    for index, name in enumerate(snapshot.joint_names[:visible_joint_count]):
        position = snapshot.joint_positions[index] if index < len(snapshot.joint_positions) else float("nan")
        velocity = snapshot.joint_velocities[index] if index < len(snapshot.joint_velocities) else float("nan")
        if snapshot.joint_efforts is not None and index < len(snapshot.joint_efforts):
            effort = snapshot.joint_efforts[index]
            print(
                f"[unitree_g1_isaac_sim] {label_prefix}joint[{index:02d}] {name}: "
                f"pos={position:.6f} vel={velocity:.6f} effort={effort:.6f}"
            )
        else:
            print(
                f"[unitree_g1_isaac_sim] {label_prefix}joint[{index:02d}] {name}: "
                f"pos={position:.6f} vel={velocity:.6f}"
            )
    if limit is not None and total > limit:
        print(f"[unitree_g1_isaac_sim] ... truncated {total - limit} additional {label_prefix}joints")
