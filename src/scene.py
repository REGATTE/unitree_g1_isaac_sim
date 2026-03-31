"""Scene construction helpers for the Unitree G1 Isaac Sim app."""

from __future__ import annotations

from pathlib import Path

from config import AppConfig


def import_stage_helpers():
    try:
        from isaacsim.core.utils.stage import add_reference_to_stage
    except ImportError:
        from omni.isaac.core.utils.stage import add_reference_to_stage
    return add_reference_to_stage


def import_ground_plane():
    try:
        from isaacsim.core.api.objects import GroundPlane
    except ImportError:
        from omni.isaac.core.objects import GroundPlane
    return GroundPlane


def import_physics_context():
    try:
        from isaacsim.core.api.physics_context import PhysicsContext
    except ImportError:
        from omni.isaac.core.physics_context import PhysicsContext
    return PhysicsContext


def build_scene(asset_path: Path, config: AppConfig) -> None:
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    print("[unitree_g1_isaac_sim] creating stage")
    add_reference_to_stage = import_stage_helpers()
    GroundPlane = import_ground_plane()
    PhysicsContext = import_physics_context()

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    stage = usd_context.get_stage()

    print("[unitree_g1_isaac_sim] configuring world prim")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    print("[unitree_g1_isaac_sim] creating physics context")
    PhysicsContext(physics_dt=config.physics_dt, prim_path="/World/PhysicsScene")
    print("[unitree_g1_isaac_sim] creating ground plane")
    GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)

    print("[unitree_g1_isaac_sim] creating light")
    light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    light.CreateIntensityAttr(500.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 0.0))

    print("[unitree_g1_isaac_sim] referencing robot asset")
    add_reference_to_stage(str(asset_path), config.robot_prim_path)
    robot_xform = UsdGeom.Xformable(stage.GetPrimAtPath(config.robot_prim_path))
    translate_op = None
    for op in robot_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = robot_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(0.0, 0.0, config.robot_height))
    print("[unitree_g1_isaac_sim] stage setup complete")
