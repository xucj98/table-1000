"""15 mm binder-clip experiments; engineering proxy, still under validation.

free_impulse uses the 4 ms standard condition. Functional experiments explicitly
use 1 ms: the current 4 ms jaw/contact response is not accepted. Native fixture
forces act only on the handle tails and the ideal gauge; no asset runtime pose
writes or direct jaw forcing. Automatic checks are recorded in experiment
results to preserve videos even when a criterion needs changes. Execution
completed is not acceptance: review complete videos and curves before accepting.
"""

import math
from functools import partial
from pathlib import Path

import numpy as np

from table_1000.physics.testing import TestContext


def free_impulse(ctx: TestContext):
    from table_1000.physics.joints import frames

    ctx.simulation(gravity=(0, 0, 0))
    ctx.camera(position=(0.14, -0.15, 0.075), target=(0.025, 0, 0.004))
    for name in ctx.asset.body_names:
        ctx.observe_body(ctx.asset.body(name))
    for name in ctx.asset.joint_names:
        ctx.observe_joint(ctx.asset, name)
    ctx.ensure_initialized()
    masses = ctx.asset.mass.astype(float)
    total_mass = float(masses.sum())

    def com_x():
        return (
            sum(
                m * ctx.asset.body(n).state["com"][0]
                for m, n in zip(masses, ctx.asset.body_names)
            )
            / total_mass
        )

    def momentum_x():
        return sum(
            m * ctx.asset.body(n).state["velocity"][0]
            for m, n in zip(masses, ctx.asset.body_names)
        )

    def anchor_error():
        return max(
            float(
                np.linalg.norm(
                    f["anchors"][1]["position"] - f["anchors"][0]["position"]
                )
            )
            for f in (frames(ctx.asset, n) for n in ctx.asset.joint_names)
        )

    ctx.observe("assembly.com.x", com_x, unit="m")
    ctx.observe("assembly.momentum.x", momentum_x, unit="kg*m/s")
    ctx.observe("assembly.anchor_error", anchor_error, unit="m")
    force = 0.5 * total_mass
    ctx.force(
        "push",
        ctx.asset.body("binder_clip.back"),
        keyframes=[
            {"time": 0, "force": (0, 0, 0)},
            {"time": 0.2, "force": (0, 0, 0)},
            {"time": 0.204, "force": (force, 0, 0)},
            {"time": 0.396, "force": (force, 0, 0)},
            {"time": 0.4, "force": (0, 0, 0)},
            {"time": 1, "force": (0, 0, 0)},
        ],
    )
    ctx.plots(
        "push.force.x", "assembly.com.x", "assembly.momentum.x", "assembly.anchor_error"
    )
    ctx.video_annotation = lambda: (
        "Foundation only | free assembly | axial impulse | clamping not evaluated"
    )
    ctx.run(1)
    impulse = sum(row["push.force.x"] * ctx.dt for row in ctx.rows[1:])
    delta_p = ctx.rows[-1]["assembly.momentum.x"] - ctx.rows[0]["assembly.momentum.x"]
    relative_error = abs(delta_p - impulse) / abs(impulse)
    maximum_anchor_error = max(row["assembly.anchor_error"] for row in ctx.rows)
    ctx.experiment_details = {
        "scope": "foundation only, not clamping/return acceptance",
        "fixture": None,
        "gravity_m_s2": [0, 0, 0],
        "total_mass_kg": total_mass,
        "force_N": force,
        "impulse_N_s": impulse,
        "momentum_change_kg_m_s": delta_p,
        "relative_momentum_error": relative_error,
        "maximum_anchor_error_m": maximum_anchor_error,
        "criteria": {"relative_momentum_error_max": 0.05, "anchor_error_max_m": 0.0002},
        "asset_runtime_pose_writes": False,
    }
    assert relative_error <= 0.05, ctx.experiment_details
    assert maximum_anchor_error <= 0.0002, ctx.experiment_details


WIDTH = 0.015
HEIGHT = 0.0105
BACK_HALF_DEPTH = 0.00305
JAW_BOTTOM = 0.00072
HANDLE_AXIS_Y = 0.0011
TAIL_Z = -0.007
HANDLE_FORCE = 1.5
OPENING_TARGET = 0.0025
WEAK_PULL = 0.05
STRONG_PULL = 0.5
THIN_GAUGE = 0.0005
THICK_GAUGE = 0.002
SCALE = WIDTH / 0.022
TAIL_Y = HANDLE_AXIS_Y + (HEIGHT - TAIL_Z) * math.tan(math.pi / 12)


def _point(body, local):
    return body.state["position"] + body.state["rotation"] @ np.array(local)


def _observe_joints(ctx, instance):
    # Native articulation tensor readback is already in radians. Read one batch
    # per sampled step and preserve the public TestContext column names/units.
    cache = {"step": None}

    def values():
        if cache["step"] != ctx.session.steps:
            articulation = instance.articulation
            cache["position"] = articulation.get_dof_positions()[0]
            cache["velocity"] = articulation.get_dof_velocities()[0]
            cache["indices"] = (
                {
                    n: articulation.shared_metatype.dof_names.index(
                        instance.joint(n).GetName()
                    )
                    for n in instance.joint_names
                }
                if "indices" not in cache
                else cache["indices"]
            )
            cache["step"] = ctx.session.steps
        return cache

    for name in instance.joint_names:
        for field, unit in [("position", "rad"), ("velocity", "rad/s")]:
            ctx.observe(
                f"{instance.name}.{name}.{field}",
                lambda name=name, field=field: values()[field][
                    values()["indices"][name]
                ],
                unit=unit,
            )


def _setup(ctx):
    ctx.camera(
        position=tuple(SCALE * x for x in (0.10, -0.065, 0.045)),
        target=(0, 0, 0.005 * SCALE),
        focal_length=28,
    )
    _observe_joints(ctx, ctx.asset)
    for name in ctx.asset.body_names:
        ctx.observe_body(ctx.asset.body(name))
    front = ctx.asset.body("binder_clip.front_jaw")
    rear = ctx.asset.body("binder_clip.rear_jaw")
    ctx.observe(
        "mouth.gap",
        lambda: float(
            (_point(rear, (0, 0, HEIGHT)) - _point(front, (0, 0, HEIGHT)))
            @ ctx.asset.body("binder_clip.back").state["rotation"][:, 1]
        ),
        unit="m",
    )
    for side, sign in [("front", 1), ("rear", -1)]:
        body = ctx.asset.body(f"binder_clip.{side}_handle")
        ctx.observe(
            f"{side}.tail.y",
            lambda body=body, sign=sign: float(
                _point(body, (0, -sign * TAIL_Y, TAIL_Z))[1]
            ),
            unit="m",
        )
    ctx.video_annotation = lambda: (
        "Native handle stops + passive jaw torsion | 1 ms condition | pending review"
    )


def _press(
    ctx,
    side,
    start=0.4,
    ramp=1.4,
    hold=1.8,
    release=2.0,
    end=3.0,
    instance=None,
    prefix="",
):
    instance = instance or ctx.asset
    sign = 1 if side == "front" else -1
    ctx.force(
        prefix + side + "_press",
        instance.body(f"binder_clip.{side}_handle"),
        point_frame="body",
        keyframes=[
            {"time": 0, "force": (0, 0, 0), "point": (0, -sign * TAIL_Y, TAIL_Z)},
            {"time": start, "force": (0, 0, 0)},
            {"time": ramp, "force": (0, sign * HANDLE_FORCE, 0)},
            {"time": hold, "force": (0, sign * HANDLE_FORCE, 0)},
            {"time": release, "force": (0, 0, 0)},
            {"time": end, "force": (0, 0, 0)},
        ],
    )


def press_release(ctx: TestContext, variant="native_stop"):
    from pxr import Usd, UsdPhysics

    ctx.simulation(dt=0.001, gravity=(0, 0, 0))
    _setup(ctx)
    if variant == "stop_removed_control":
        ctx.video_annotation = lambda: (
            "CONTROL | handle limits/contact removed | passive jaw springs | 1 ms"
        )
        ctx.camera(
            position=tuple(SCALE * x for x in (0.18, -0.135, 0.09)),
            target=(0, 0, HEIGHT),
            focal_length=28,
        )
        for side in ["front", "rear"]:
            UsdPhysics.RevoluteJoint(
                ctx.asset.joint(f"binder_clip.{side}_handle_turn")
            ).GetUpperLimitAttr().Set(float("inf"))
            UsdPhysics.RevoluteJoint(
                ctx.asset.joint(f"binder_clip.{side}_handle_turn")
            ).GetLowerLimitAttr().Set(-float("inf"))
            # Isolate the 0-degree stop from alternative handle contact paths.
            # This deliberately modified test control is never exported.
            body = ctx.session.stage.GetPrimAtPath(
                ctx.asset.body(f"binder_clip.{side}_handle").path
            )
            for prim in Usd.PrimRange(body):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
    if variant == "outward_initial":
        ctx.initial(
            joints={
                f"binder_clip.{s}_handle_turn": -math.pi / 6 for s in ["front", "rear"]
            }
        )
    for side in ["front", "rear"]:
        if variant not in ("asymmetric", "asymmetric_rear") or side == (
            "rear" if variant == "asymmetric_rear" else "front"
        ):
            _press(ctx, side)
    ctx.plots(
        "rear_press.force.y" if variant == "asymmetric_rear" else "front_press.force.y",
        "mouth.gap",
        "asset.binder_clip.front_jaw_open.position",
        "asset.binder_clip.rear_jaw_open.position",
        "asset.binder_clip.front_handle_turn.position",
    )
    with ctx.fixed(ctx.asset.body("binder_clip.back")):
        ctx.run(3)
    ctx.experiment_details = {
        "scope": "1 ms functional experiment; standard 4 ms remains unaccepted",
        "variant": variant,
        "control_only_handle_collisions_disabled": variant == "stop_removed_control",
        "root_fixture": "native fixed joint",
        "handle_locks": False,
        "forces_on_jaws": False,
        "asset_runtime_pose_writes": False,
        "handle_force_N": HANDLE_FORCE,
        "force_point": "handle tail bridge center, body frame",
        "gravity_m_s2": [0, 0, 0],
    }
    checks = {
        "opened": max(r["mouth.gap"] for r in ctx.rows) >= OPENING_TARGET,
        "closed_after_release": abs(ctx.rows[-1]["mouth.gap"]) <= 0.00015,
        "joint_limits": all(
            -0.0174533
            <= r[f"asset.binder_clip.{side}_jaw_open.position"]
            <= math.radians(16)
            and math.radians(-31)
            <= r[f"asset.binder_clip.{side}_handle_turn.position"]
            <= math.radians(1)
            for r in ctx.rows
            for side in ["front", "rear"]
        ),
    }
    if variant in ("asymmetric", "asymmetric_rear"):
        checks.pop("opened")  # One-side opening has a different physical target.
        active = "rear" if variant == "asymmetric_rear" else "front"
        passive = "front" if active == "rear" else "rear"
        checks["independent_sides"] = max(
            r[f"asset.binder_clip.{active}_jaw_open.position"]
            - r[f"asset.binder_clip.{passive}_jaw_open.position"]
            for r in ctx.rows
        ) >= math.radians(2)
    if variant == "stop_removed_control":
        checks = {
            "control_has_smaller_opening": max(r["mouth.gap"] for r in ctx.rows)
            < 0.002 * SCALE
        }
    ctx.experiment_details["automatic_checks"] = checks


def _gauge(ctx: TestContext, fixture_path, thickness=0.001, strong=False):
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    ctx.simulation(dt=0.001, gravity=(0, 0, 0))
    _setup(ctx)
    ctx.camera(
        position=tuple(SCALE * x for x in (0.15, -0.0975, 0.0675)),
        target=(0, 0, 0.0125 * SCALE),
        focal_length=28,
    )
    ctx.video_annotation = lambda: (
        "Ideal rigid gauge | weak then strong pull | 1 ms"
        if strong
        else "Ideal rigid gauge | same weak pull during handle release | 1 ms"
    )
    # Independent ideal gauge: no attraction, no clip/gauge connection, no glue.
    path = fixture_path
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Gauge")
    stage.SetDefaultPrim(root.GetPrim())
    rigid = UsdGeom.Xform.Define(stage, "/Gauge/body").GetPrim()
    rigid.SetCustomDataByKey("table1000:name", "gauge.body")
    UsdPhysics.RigidBodyAPI.Apply(rigid)
    UsdPhysics.MassAPI.Apply(rigid).CreateMassAttr(0.001 * SCALE**3)
    physx = PhysxSchema.PhysxRigidBodyAPI.Apply(rigid)
    physx.CreateEnableCCDAttr(True)
    physx.CreateSolverPositionIterationCountAttr(16)
    physx.CreateSolverVelocityIterationCountAttr(4)
    cube = UsdGeom.Cube.Define(stage, "/Gauge/body/shape")
    cube.CreateSizeAttr(1)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, HEIGHT + 0.001 * SCALE))
    xf.AddScaleOp().Set(Gf.Vec3f(0.012 * SCALE, thickness, 0.010 * SCALE))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.82)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim()).CreateContactOffsetAttr(0.0001)
    mat = UsdShade.Material.Define(stage, "/Gauge/material")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(0.5)
    api.CreateDynamicFrictionAttr(0.4)
    api.CreateRestitutionAttr(0)
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(
        "average"
    )
    UsdShade.MaterialBindingAPI.Apply(rigid).Bind(mat, materialPurpose="physics")
    stage.GetRootLayer().Save()
    g = ctx.spawn(path, name="gauge")
    ctx.observe_body(g.body("gauge.body"))
    # The ideal gauge spans the moving lips at this size.
    # Begin with 0.3 mm clearance before the passive jaws settle onto it.
    from scipy.optimize import brentq

    def gap(q):
        return 2 * BACK_HALF_DEPTH * (1 - math.cos(q)) + 2 * (
            HEIGHT - JAW_BOTTOM
        ) * math.sin(q)

    q = brentq(lambda q: gap(q) - (thickness + 0.0003), 0, 0.26)
    ctx.initial(joints={f"binder_clip.{s}_jaw_open": q for s in ["front", "rear"]})
    ctx.observe(
        "gauge.slip", lambda: float(g.body("gauge.body").state["position"][2]), unit="m"
    )
    ctx.force(
        "gauge_pull",
        g.body("gauge.body"),
        keyframes=[
            {"time": 0, "force": (0, 0, 0)},
            {"time": 0.8, "force": (0, 0, 0)},
            {"time": 1.2, "force": (0, 0, WEAK_PULL)},
            {"time": 2, "force": (0, 0, WEAK_PULL)},
            {"time": 2.2, "force": (0, 0, STRONG_PULL if strong else WEAK_PULL)},
            {"time": 3.4, "force": (0, 0, STRONG_PULL if strong else WEAK_PULL)},
            {"time": 3.6, "force": (0, 0, 0)},
            {"time": 4, "force": (0, 0, 0)},
        ],
    )
    if not strong:
        for side in ["front", "rear"]:
            _press(ctx, side, start=2.2, ramp=2.6, hold=3.2, release=3.4, end=4.0)
    ctx.plots(
        "gauge_pull.force.z",
        "gauge.slip",
        "mouth.gap",
        *([] if strong else ["front_press.force.y"]),
    )
    with ctx.fixed(ctx.asset.body("binder_clip.back")):
        # Native ideal axial guide locks lateral motion and rotation only.
        # It supplies no axial holding force and is an explicit test fixture.
        j = UsdPhysics.PrismaticJoint.Define(
            ctx.session.stage, "/World/Fixtures/gauge_axial_guide"
        )
        j.CreateBody1Rel().SetTargets([g.body("gauge.body").path])
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(-0.02 * SCALE)
        j.CreateUpperLimitAttr(0.012 * SCALE)
        j.CreateLocalPos0Attr(Gf.Vec3f(0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        ctx.run(4)
        ctx.session.stage.RemovePrim(j.GetPath())
    ctx.experiment_details = {
        "scope": "1 ms ideal rigid gauge only, not real paper or standard 4 ms",
        "thickness_m": thickness,
        "gauge_dimensions_m": [0.012 * SCALE, thickness, 0.010 * SCALE],
        "gauge_mass_kg": 0.001 * SCALE**3,
        "gauge_static_dynamic_friction": [0.5, 0.4],
        "average_pair_friction": [0.425, 0.325],
        "initial_q_rad": q,
        "weak_pull_N": WEAK_PULL,
        "strong_pull_N": STRONG_PULL if strong else None,
        "gravity_m_s2": [0, 0, 0],
        "asset_runtime_pose_writes": False,
        "clamping_asset_only": True,
        "guide": "native prismatic Z, no axial drive/friction; catch stops outside retention zone",
        "guide_limits_m": [-0.02 * SCALE, 0.012 * SCALE],
        "strong_pull": strong,
    }

    weak = [r for r in ctx.rows if 0.8 <= r["time"] <= 2.0]
    reference = weak[0]["gauge.slip"]
    weak_slip = max(abs(r["gauge.slip"] - reference) for r in weak)
    weak_gap_error = max(abs(r["mouth.gap"] - thickness) for r in weak)
    later = [r for r in ctx.rows if r["time"] >= 2.2]
    release_slip = max(abs(r["gauge.slip"] - reference) for r in later)
    ctx.experiment_details.update(
        {
            "weak_slip_m": weak_slip,
            "weak_gap_error_m": weak_gap_error,
            "release_or_strong_pull_slip_m": release_slip,
            "automatic_checks": {
                "weak_holds": weak_slip <= 0.0002,
                "gauge_contact_error": weak_gap_error <= 0.00015,
                "release_or_strong_pull": release_slip >= 0.002,
            },
            "reaction_forces": "not sampled; force columns are applied external forces",
        }
    )
    if not strong:
        ctx.experiment_details["automatic_checks"]["opened_for_release"] = (
            max(r["mouth.gap"] for r in later) >= thickness + 0.0005
        )


def gauge(ctx: TestContext, thickness=0.001, strong=False):
    # Only the ideal test fixture is temporary; it is not an asset dependency.
    import tempfile

    with tempfile.TemporaryDirectory(
        prefix="gauge-fixture-", dir=ctx.asset_path.parent
    ) as directory:
        _gauge(ctx, Path(directory) / "gauge.usda", thickness, strong)


def _anchor_error(instance):
    from table_1000.physics.joints import frames

    return max(
        float(np.linalg.norm(f["anchors"][1]["position"] - f["anchors"][0]["position"]))
        for f in (frames(instance, n) for n in instance.joint_names)
    )


def free_eccentric(ctx: TestContext):
    """A force on one handle moves and rotates the entire unanchored assembly."""
    ctx.simulation(dt=0.001, gravity=(0, 0, 0))
    _setup(ctx)
    ctx.camera(
        position=(0.19, -0.135, 0.105),
        target=(0, 0.0325, 0.005 * SCALE),
        focal_length=28,
    )
    ctx.video_annotation = lambda: "Free assembly | eccentric handle-tail force | 1 ms"
    ctx.ensure_initialized()
    masses = ctx.asset.mass.astype(float)
    total = float(masses.sum())
    for i, axis in enumerate("xyz"):
        ctx.observe(
            f"momentum.{axis}",
            lambda i=i: sum(
                m * ctx.asset.body(n).state["velocity"][i]
                for m, n in zip(masses, ctx.asset.body_names)
            ),
            unit="kg*m/s",
        )
    ctx.observe("anchor.error", lambda: _anchor_error(ctx.asset), unit="m")
    force = 0.5 * total
    ctx.force(
        "eccentric",
        ctx.asset.body("binder_clip.front_handle"),
        point_frame="body",
        keyframes=[
            {"time": 0, "force": (0, 0, 0), "point": (0, -TAIL_Y, TAIL_Z)},
            {"time": 0.2, "force": (0, 0, 0)},
            {"time": 0.3, "force": (0, force, 0)},
            {"time": 0.4, "force": (0, force, 0)},
            {"time": 0.5, "force": (0, 0, 0)},
            {"time": 1, "force": (0, 0, 0)},
        ],
    )
    ctx.plots(
        "eccentric.force.y",
        "momentum.y",
        "anchor.error",
        "asset.binder_clip.back.angular_velocity.x",
    )
    ctx.run(1)
    impulse = sum(r["eccentric.force.y"] * ctx.dt for r in ctx.rows[1:])
    change = ctx.rows[-1]["momentum.y"] - ctx.rows[0]["momentum.y"]
    error = abs(change - impulse) / abs(impulse)
    anchor = max(r["anchor.error"] for r in ctx.rows)
    spin = max(abs(r["asset.binder_clip.back.angular_velocity.x"]) for r in ctx.rows)
    ctx.experiment_details = {
        "fixture": None,
        "force_on": "front handle tail",
        "relative_momentum_error": error,
        "maximum_anchor_error_m": anchor,
        "maximum_back_angular_speed_rad_s": spin,
        "asset_runtime_pose_writes": False,
        "automatic_checks": {
            "momentum": error <= 0.05,
            "anchors": anchor <= 0.0002,
            "free_rotation": spin >= 0.1,
        },
    }


def low_drop(ctx: TestContext):
    """Sideways low drop, with no fixture and no post-initialization pose writes."""
    import json

    from table_1000.physics.test import ground_clearance

    ctx.simulation(dt=0.001)
    _setup(ctx)
    ctx.camera(
        position=tuple(SCALE * x for x in (0.16, -0.16, 0.13)),
        target=(0, 0, 0.023 * SCALE),
        focal_length=28,
    )
    ctx.ground(
        contact_offset=0.0001,
        static_friction=0.35,
        dynamic_friction=0.25,
        restitution=0,
    )
    ctx.initial(
        position=(0, 0, 0.03 * SCALE), rotation=(math.sqrt(0.5), math.sqrt(0.5), 0, 0)
    )
    ctx.observe("anchor.error", lambda: _anchor_error(ctx.asset), unit="m")
    ctx.observe(
        "assembly.max_speed",
        lambda: max(
            float(np.linalg.norm(ctx.asset.body(n).state["velocity"][:3]))
            for n in ctx.asset.body_names
        ),
        unit="m/s",
    )
    ctx.plots("asset.binder_clip.back.position.z", "assembly.max_speed", "anchor.error")
    ctx.video_annotation = lambda: "Free sideways low drop | gravity | 1 ms"
    ctx.run(2)
    model = json.loads((ctx.asset_path.parent / "model.json").read_text())
    settled = [r for r in ctx.rows if r["time"] >= 1.5]
    contact = ground_clearance(model, settled, [ctx.asset.name], 0)
    anchor = max(r["anchor.error"] for r in ctx.rows)
    speed = max(r["assembly.max_speed"] for r in settled)
    ctx.experiment_details = {
        "fixture": None,
        "initial_origin_height_m": 0.03 * SCALE,
        "initial_rotation_wxyz": [math.sqrt(0.5), math.sqrt(0.5), 0, 0],
        "maximum_anchor_error_m": anchor,
        "settled_contact": contact,
        "maximum_settled_body_speed_m_s": speed,
        "asset_runtime_pose_writes": False,
        "automatic_checks": {
            "anchors": anchor <= 0.0002,
            "settled_no_persistent_penetration": contact[
                "maximum_geometry_penetration_m"
            ]
            <= 0.00015,
            "settled": speed <= 0.05,
        },
    }


def two_instances(ctx: TestContext):
    """Alternating handle loading of two separate copies with two back fixtures."""
    ctx.simulation(dt=0.001, gravity=(0, 0, 0))
    _setup(ctx)
    second = ctx.spawn(ctx.asset_path, name="second", position=(0.045 * SCALE, 0, 0))
    ctx.camera(
        position=tuple(SCALE * x for x in (0.13, -0.10, 0.065)),
        target=(0.0225 * SCALE, 0, 0.005 * SCALE),
        focal_length=28,
    )
    for name in second.body_names:
        ctx.observe_body(second.body(name))
    _observe_joints(ctx, second)
    ctx.observe(
        "second.mouth.gap",
        lambda: float(
            (
                _point(second.body("binder_clip.rear_jaw"), (0, 0, HEIGHT))
                - _point(second.body("binder_clip.front_jaw"), (0, 0, HEIGHT))
            )
            @ second.body("binder_clip.back").state["rotation"][:, 1]
        ),
        unit="m",
    )
    for side in ["front", "rear"]:
        _press(ctx, side, end=5)
        _press(
            ctx,
            side,
            start=2.4,
            ramp=3.4,
            hold=3.8,
            release=4,
            end=5,
            instance=second,
            prefix="second_",
        )
    ctx.plots(
        "front_press.force.y",
        "second_front_press.force.y",
        "mouth.gap",
        "second.mouth.gap",
    )
    ctx.video_annotation = lambda: (
        "Two independent instances | alternating handle forces | 1 ms"
    )
    with (
        ctx.fixed(ctx.asset.body("binder_clip.back")),
        ctx.fixed(second.body("binder_clip.back")),
    ):
        ctx.run(5)
    first_open = max(r["mouth.gap"] for r in ctx.rows if 0.4 <= r["time"] <= 2)
    second_open = max(r["second.mouth.gap"] for r in ctx.rows if 2.4 <= r["time"] <= 4)
    idle_second = max(abs(r["second.mouth.gap"]) for r in ctx.rows if r["time"] <= 2)
    idle_first = max(abs(r["mouth.gap"]) for r in ctx.rows if r["time"] >= 2.4)
    ctx.experiment_details = {
        "instances": 2,
        "back_fixtures": 2,
        "first_loaded_gap_m": first_open,
        "second_loaded_gap_m": second_open,
        "idle_gaps_m": [idle_first, idle_second],
        "asset_runtime_pose_writes": False,
        "automatic_checks": {
            "each_opens": min(first_open, second_open) >= OPENING_TARGET,
            "no_cross_talk": max(idle_first, idle_second) <= 0.00015,
            "both_close": max(
                abs(ctx.rows[-1][c]) for c in ("mouth.gap", "second.mouth.gap")
            )
            <= 0.00015,
        },
    }


TESTS = {
    "free_impulse": free_impulse,
    "press_release": partial(press_release, variant="native_stop"),
    "handles_outward": partial(press_release, variant="outward_initial"),
    "asymmetric_press": partial(press_release, variant="asymmetric"),
    "asymmetric_rear": partial(press_release, variant="asymmetric_rear"),
    "stop_removed_control": partial(press_release, variant="stop_removed_control"),
    "gauge_thin": partial(gauge, thickness=THIN_GAUGE),
    "gauge_thick": partial(gauge, thickness=THICK_GAUGE),
    "gauge_strong_thin": partial(gauge, thickness=THIN_GAUGE, strong=True),
    "gauge_strong_thick": partial(gauge, thickness=THICK_GAUGE, strong=True),
    "free_eccentric": free_eccentric,
    "low_drop": low_drop,
    "two_instances": two_instances,
}
