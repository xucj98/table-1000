"""Rigid plug support, low-height releases, and controlled tipping experiments."""

import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.physics.testing import TestContext

BODY = "power_plug.body"
MASS_KG = 0.065
RELEASE_HEIGHT_M = 0.012


def _column(instance, field, axis):
    return f"{instance}.{BODY}.{field}.{axis}"


def _collision_groups(ctx):
    model = json.loads((ctx.asset_path.parent / "model.json").read_text())
    worlds = {}
    groups = {}
    for node in model["nodes"]:
        worlds[node["name"]] = worlds.get(node["parent"], np.eye(4)) @ np.array(node["matrix"])
        if node.get("role") == "collision":
            part = node["parent"].rsplit(".", 1)[-1]
            transform = np.linalg.inv(worlds[BODY]) @ worlds[node["name"]]
            points = np.c_[node["vertices"], np.ones(len(node["vertices"]))] @ transform.T
            groups.setdefault(part, []).append(points[:, :3])
    return {name: np.concatenate(parts) for name, parts in groups.items()}


def _lowest_z(points, quaternion):
    rotation = Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]]).as_matrix()
    return float((points @ rotation.T)[:, 2].min())


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(
        instance.name + ".tilt",
        lambda b=body: math.acos(float(np.clip(b.state["rotation"][2, 2], -1, 1))),
        unit="rad",
    )
    return body


def _settled(ctx, instance, *, linear_limit=0.015, angular_limit=0.08):
    tail = [row for row in ctx.rows if row["time"] >= ctx.session.time - 0.5 - 1e-8]
    speed = np.array(
        [[row[_column(instance.name, "linear_velocity", axis)] for axis in "xyz"] for row in tail]
    )
    angular = np.array(
        [[row[_column(instance.name, "angular_velocity", axis)] for axis in "xyz"] for row in tail]
    )
    position = np.array(
        [[row[_column(instance.name, "position", axis)] for axis in "xyz"] for row in tail]
    )
    assert np.max(np.linalg.norm(speed, axis=1)) < linear_limit
    assert np.max(np.linalg.norm(angular, axis=1)) < angular_limit
    assert np.linalg.norm(np.ptp(position, axis=0)) < 0.002


def drop_orientations(ctx: TestContext):
    """Release shell-face, pin-end, and side orientations above a flat floor."""
    from table_1000.physics.test import ground_clearance

    ctx.simulation(dt=0.002)
    ctx.ground(static_friction=0.45, dynamic_friction=0.34, restitution=0.0,
               contact_offset=0.0001)
    groups = _collision_groups(ctx)
    orientations = {
        "asset": (1.0, 0.0, 0.0, 0.0),
        "pins_down": tuple(Rotation.from_euler("x", math.pi).as_quat()[[3, 0, 1, 2]]),
        "side": tuple(Rotation.from_euler("y", math.pi / 2).as_quat()[[3, 0, 1, 2]]),
    }
    instances = {"asset": ctx.asset}
    ctx.initial(position=(0.0, 0.0, RELEASE_HEIGHT_M))
    for name, quaternion in list(orientations.items())[1:]:
        points = np.concatenate(list(groups.values()))
        z = RELEASE_HEIGHT_M - _lowest_z(points, quaternion)
        instances[name] = ctx.spawn(
            ctx.asset_path,
            name=name,
            position=(0.0, 0.12 if name == "pins_down" else -0.12, z),
            rotation=quaternion,
        )
    bodies = {name: _body(ctx, instance) for name, instance in instances.items()}
    ctx.camera(position=(0.30, -0.42, 0.30), target=(0.0, 0.0, 0.035))
    ctx.video_annotation = lambda: "12 mm release | shell face, three pins, side"
    ctx.plots(*[_column(name, "position", "z") for name in instances],
              *[_column(name, "angular_velocity", "y") for name in instances])
    ctx.run(3.2)
    clearance = ground_clearance(
        json.loads((ctx.asset_path.parent / "model.json").read_text()),
        ctx.rows,
        ctx.session.instances,
        0,
    )
    assert clearance["maximum_geometry_penetration_m"] < 0.0015
    for instance in instances.values():
        _settled(ctx, instance)
    final_states = {
        name: {
            "position_m": bodies[name].state["position"].tolist(),
            "linear_velocity_m_s": bodies[name].state["velocity"][:3].tolist(),
            "angular_velocity_rad_s": bodies[name].state["velocity"][3:].tolist(),
        }
        for name in instances
    }
    ctx.experiment_details = {
        "contract": "12 mm gravity releases from shell-face, pin-end, and side orientations",
        "release_height_m": RELEASE_HEIGHT_M,
        "initial_lowest_point_clearance_m": RELEASE_HEIGHT_M,
        "materials": "shell 0.55/0.42; metal pins 0.32/0.24 static/dynamic friction; zero restitution",
        "maximum_geometry_penetration_m": clearance["maximum_geometry_penetration_m"],
        "final_states": final_states,
        "boundary": "rigid-body drop and support only; no impact damage, pin flex, outlet mating, or electrical behavior",
    }


def tip_and_release(ctx: TestContext):
    """Apply a brief top-edge horizontal load, then observe free settling."""
    ctx.ground(static_friction=0.45, dynamic_friction=0.34, restitution=0.0,
               contact_offset=0.0001)
    ctx.initial(position=(0.0, 0.0, 0.0))
    body = _body(ctx, ctx.asset)
    ctx.force(
        "edge_load",
        body,
        point_frame="body",
        keyframes=[
            {"time": 0.0, "force": (0.0, 0.0, 0.0), "point": (0.0, 0.0, 0.045)},
            {"time": 0.4, "force": (0.0, 0.0, 0.0), "point": (0.0, 0.0, 0.045)},
            {"time": 0.404, "force": (0.60, 0.0, 0.0), "point": (0.0, 0.0, 0.045)},
            {"time": 0.654, "force": (0.60, 0.0, 0.0), "point": (0.0, 0.0, 0.045)},
            {"time": 0.658, "force": (0.0, 0.0, 0.0), "point": (0.0, 0.0, 0.045)},
        ],
    )
    ctx.camera(position=(0.28, -0.36, 0.27), target=(0.0, 0.0, 0.03))
    ctx.video_annotation = lambda: "0.60 N top-edge push, then released"
    ctx.plots("edge_load.force.x", _column("asset", "position", "x"),
              "asset.tilt", _column("asset", "angular_velocity", "y"))
    ctx.run(3.2)
    _settled(ctx, ctx.asset)
    peak_tilt = max(row["asset.tilt"] for row in ctx.rows)
    displacement = ctx.rows[-1][_column("asset", "position", "x")] - ctx.rows[0][_column("asset", "position", "x")]
    assert peak_tilt > math.radians(20)
    ctx.experiment_details = {
        "contract": "brief horizontal edge load followed by free motion and settling",
        "applied_load_N": 0.60,
        "application_point_body_m": [0.0, 0.0, 0.045],
        "load_interval_s": [0.404, 0.658],
        "peak_tilt_x_rad": peak_tilt,
        "final_horizontal_displacement_m": displacement,
        "mass_kg": MASS_KG,
        "boundary": "ideal flat floor and estimated parameters; not a calibrated human push or outlet insertion test",
    }


TESTS = {"drop_orientations": drop_orientations, "tip_and_release": tip_and_release}
