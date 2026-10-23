"""Free rigid-bottle experiments; forces and geometry use SI units."""

import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.physics.testing import TestContext

BODY = "glue.bottle"


def collision_groups(ctx):
    model = json.loads((ctx.asset_path.parent / "model.json").read_text())
    worlds, groups = {}, {}
    for node in model["nodes"]:
        worlds[node["name"]] = worlds.get(node["parent"], np.eye(4)) @ np.asarray(node["matrix"])
        if node.get("role") == "collision":
            transform = np.linalg.inv(worlds[BODY]) @ worlds[node["name"]]
            groups[node["parent"].rsplit(".", 1)[-1]] = (
                np.c_[node["vertices"], np.ones(len(node["vertices"]))] @ transform.T)[:, :3]
    assert set(groups) == {"base_rim", "container", "neck", "screw_collar", "nozzle_cap"}
    return groups


def setup(ctx, gravity=(0, 0, -9.81), dt=.004):
    ctx.simulation(dt=dt, gravity=gravity)
    if np.linalg.norm(gravity):
        ctx.ground(static_friction=.45, dynamic_friction=.35, restitution=0., contact_offset=.0001)
    ctx.experiment_details = {"contract": "glue-000000-v1", "dt_s": ctx.dt,
                              "force_source": "commanded external force, not contact reaction",
                              "calibration": "uncalibrated engineering estimates"}


def observe(ctx, instance, groups):
    """Read each environment's own native tensors, including mass properties."""
    cache = {}

    def state():
        if cache.get("step") != ctx.session.steps:
            current = ctx.session.instances.get(instance.name)
            if current is None:
                cache.update(step=ctx.session.steps, position=np.full((ctx.num_envs, 3), np.nan),
                    rotation=np.full((ctx.num_envs, 3, 3), np.nan), com=np.full((ctx.num_envs, 3), np.nan),
                    velocity=np.full((ctx.num_envs, 6), np.nan), energy=np.full(ctx.num_envs, np.nan),
                    clearance=np.full(ctx.num_envs, np.nan), tilt=np.full(ctx.num_envs, np.nan),
                    parts={n: np.full(ctx.num_envs, np.nan) for n in groups})
                return cache
            instance_view = current
            view = instance_view.view.view
            positions, rotations, centers = instance_view.environment_states()
            speeds = view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 1, 6)[:, 0]
            masses = view.get_masses().detach().cpu().numpy().reshape(ctx.num_envs)
            inertias = view.get_inertias().detach().cpu().numpy().reshape(ctx.num_envs, 3, 3)
            r = rotations[:, 0]
            world_i = r @ inertias @ r.transpose(0, 2, 1)
            energy = (.5*masses*np.sum(speeds[:, :3]**2, axis=1)
                      + .5*np.einsum("ei,eij,ej->e", speeds[:, 3:], world_i, speeds[:, 3:])
                      - masses*(centers[:, 0] @ np.asarray(ctx.gravity)))
            clearances = {n: positions[:, 0, 2] + np.min(
                np.einsum("eij,pj->epi", r, points)[:, :, 2], axis=1) for n, points in groups.items()}
            cache.update(step=ctx.session.steps, position=positions[:, 0], rotation=r, com=centers[:, 0],
                         velocity=speeds, energy=energy, clearance=np.min(list(clearances.values()), axis=0),
                         tilt=np.arccos(np.clip(r[:, 2, 2], -1, 1)), parts=clearances)
        return cache

    # Standard env0 pose columns are also consumed by the runner's independent
    # ground-clearance audit. Removed lifecycle instances have missing samples.
    for j, a in enumerate("xyz"):
        ctx.observe(f"{instance.name}.{BODY}.position.{a}", lambda j=j: state()["position"][0, j], unit="m")
    for j, a in enumerate("wxyz"):
        def orientation(j=j):
            r = state()["rotation"][0]
            return Rotation.from_matrix(r).as_quat()[[3, 0, 1, 2]][j] if np.isfinite(r).all() else np.nan
        ctx.observe(f"{instance.name}.{BODY}.quaternion.{a}", orientation, unit="1")
    for e in range(ctx.num_envs):
        p = f"{instance.name}.e{e}"
        for name, unit in (("clearance", "m"), ("energy", "J"), ("tilt", "rad")):
            ctx.observe(p+"."+name, lambda e=e, n=name: state()[n][e], unit=unit)
        ctx.observe(p+".speed", lambda e=e: np.linalg.norm(state()["velocity"][e, :3]), unit="m/s")
        ctx.observe(p+".spin", lambda e=e: np.linalg.norm(state()["velocity"][e, 3:]), unit="rad/s")
        for name in groups:
            ctx.observe(p+".gap."+name, lambda e=e, n=name: state()["parts"][n][e], unit="m")
        for j, a in enumerate("xyz"):
            for field in ("position", "com"):
                ctx.observe(p+"."+field+"."+a, lambda e=e, j=j, f=field: state()[f][e, j], unit="m")
            ctx.observe(p+".v."+a, lambda e=e, j=j: state()["velocity"][e, j], unit="m/s")
            ctx.observe(p+".w."+a, lambda e=e, j=j: state()["velocity"][e, j+3], unit="rad/s")
        for j in range(3):
            for k in range(3):
                ctx.observe(p+f".r{j}{k}", lambda e=e, j=j, k=k: state()["rotation"][e, j, k], unit="1")
    return instance.body(BODY)


def properties(ctx, instances):
    from pxr import PhysxSchema
    result = {}
    for instance in instances:
        native = instance.view.view
        api = PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.body(BODY).path))
        result[instance.name] = {
            "mass_kg": native.get_masses().detach().cpu().numpy().reshape(ctx.num_envs).tolist(),
            "local_com_m_xyzw": native.get_coms().detach().cpu().numpy().reshape(ctx.num_envs, 7).tolist(),
            "local_inertia_kg_m2": native.get_inertias().detach().cpu().numpy().reshape(ctx.num_envs, 3, 3).tolist(),
            "linear_damping_s_inv": api.GetLinearDampingAttr().Get(),
            "angular_damping_s_inv": api.GetAngularDampingAttr().Get(),
            "rigid_bodies": instance.body_names, "joints": instance.joint_names}
        assert np.max(np.abs(np.asarray(result[instance.name]["mass_kg"])-.02)) < 1e-7
    ctx.experiment_details["mass_properties"] = result
    return result


def column(ctx, name):
    return np.asarray([row[name] for row in ctx.rows])


def motion_checks(ctx, instances, *, settle=False):
    results = {}
    ctx.experiment_details["motion_checks"] = results
    time = column(ctx, "time")
    for instance in instances:
        for e in range(ctx.num_envs):
            p = f"{instance.name}.e{e}"
            tail = time >= time[-1]-.4-1e-9
            result = {"penetration_m": max(0., -float(column(ctx, p+".clearance").min())),
                      "peak_speed_m_s": float(column(ctx, p+".speed").max()),
                      "peak_spin_rad_s": float(column(ctx, p+".spin").max()),
                      "tail_speed_m_s": float(column(ctx, p+".speed")[tail].max()),
                      "tail_spin_rad_s": float(column(ctx, p+".spin")[tail].max())}
            results[p] = result
            assert all(np.isfinite(list(result.values()))), (p, result)
            assert result["penetration_m"] < .0015, (p, result)
            assert result["peak_speed_m_s"] < 2 and result["peak_spin_rad_s"] < 80, (p, result)
            if settle:
                assert result["tail_speed_m_s"] < .02 and result["tail_spin_rad_s"] < .15, (p, result)
    ctx.metric("penetration", max(v["penetration_m"] for v in results.values())*1000,
               unit="mm", label="全环境最大碰撞几何穿透")
    return results


def upright_push_tip(ctx: TestContext):
    setup(ctx, dt=.002)
    groups = collision_groups(ctx)
    low = ctx.asset
    # A face-normal push avoids balancing on a 16-gon vertex. The first
    # vertex-aligned diagnostic remains archived, including its residual roll.
    yaw = math.pi/16
    quaternion = (math.cos(yaw/2), 0, 0, math.sin(yaw/2))
    ctx.initial(position=(0, -.14, .0005), rotation=quaternion)
    high = ctx.spawn(ctx.asset_path, name="high", position=(0, 0, .0005), rotation=quaternion)
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(0, .14, .0005), rotation=quaternion)
    for instance in (low, high, idle):
        observe(ctx, instance, groups)
    def face_point(radius, height):
        return (-radius*math.cos(yaw)**2, radius*math.cos(yaw)*math.sin(yaw), height)

    for name, instance, point, maximum in (
            ("low_push", low, face_point(.012, .008), .11),
            ("high_push", high, face_point(.00495, .075), .06)):
        ctx.force(name, instance.body(BODY), point_frame="body", keyframes=[
            {"time": 0., "force": (0, 0, 0), "point": point},
            {"time": .5, "force": (0, 0, 0)},
            {"time": .7, "force": (.02, 0, 0)},
            {"time": .8, "force": (.02, 0, 0)},
            {"time": 1.8, "force": (maximum, 0, 0)},
            {"time": 1.9, "force": (maximum, 0, 0)},
            {"time": 1.92, "force": (0, 0, 0)},
            {"time": 3., "force": (0, 0, 0)}])
    ctx.camera(position=(.35, -.95, .60), target=(.09, 0, .035), focal_length=32)
    ctx.video_annotation = lambda: "low 8mm / high 75mm / idle | ramp and release | all bodies free"
    ctx.plots("low_push.force.x", "high_push.force.x", "asset.e0.position.x", "high.e0.tilt",
              "idle.e0.position.x", "asset.e0.speed", "high.e0.clearance")
    ctx.ensure_initialized()
    properties(ctx, (low, high, idle))
    ctx.experiment_details.update({"support_radius_x_m": float(groups["base_rim"][:, 0].max())*math.cos(yaw),
        "initial_yaw_rad": yaw, "push_alignment": "normal to a base support facet",
        "nominal_slide_threshold_N": .02*9.81*.45,
        "nominal_tip_thresholds_N": {"low": .02*9.81*.0124/.008, "high": .02*9.81*.0124/.075},
        "release_time_s": 1.92, "instances": ["asset=low8mm", "high=75mm", "idle"]})
    ctx.run(3.)
    checks = motion_checks(ctx, (low, high, idle), settle=True)
    time = column(ctx, "time")
    outcomes = []
    for e in range(ctx.num_envs):
        lp, hp, ip = (f"{n}.e{e}" for n in ("asset", "high", "idle"))
        displacement = float(np.ptp(column(ctx, lp+".position.x")))
        peak_low = float(column(ctx, lp+".tilt").max())
        peak_high = float(column(ctx, hp+".tilt").max())
        idle_shift = float(np.ptp(column(ctx, ip+".position.x")))
        hold_shift = float(np.ptp(column(ctx, lp+".position.x")[time <= .8+1e-9]))
        moving = np.flatnonzero((time > .8) & (column(ctx, lp+".speed") > .005))
        onset = None if not len(moving) else int(moving[0])
        outcome = {"env": e, "low_slide_m": displacement, "low_peak_tilt_rad": peak_low,
                   "high_peak_tilt_rad": peak_high, "idle_shift_m": idle_shift,
                   "hold_shift_m": hold_shift, "slide_onset_time_s": None if onset is None else float(time[onset]),
                   "slide_onset_force_N": None if onset is None else float(ctx.rows[onset]["low_push.force.x"])}
        outcomes.append(outcome)
        assert displacement > .003 and peak_low < math.radians(20), outcome
        assert peak_high > math.radians(60), outcome
        assert idle_shift < .001 and column(ctx, ip+".tilt").max() < math.radians(3), outcome
        assert hold_shift < .001, outcome
    ctx.experiment_details.update(per_environment=outcomes, motion_checks=checks)
    ctx.metric("low_slide", max(r["low_slide_m"] for r in outcomes)*1000, unit="mm", label="低位推力滑移")
    ctx.metric("high_tilt", min(r["high_peak_tilt_rad"] for r in outcomes)*180/math.pi, unit="deg", label="高位推力最小峰值倾角")
    ctx.metric("slide_onset", max(r["slide_onset_force_N"] for r in outcomes), unit="N", label="低位起动推力")


def quaternion(axis, angle):
    return tuple(Rotation.from_euler(axis, angle).as_quat()[[3, 0, 1, 2]])


def placed(ctx, name, xy, rotation, clearance, groups):
    points = np.concatenate(list(groups.values()))
    r = Rotation.from_quat(np.asarray(rotation)[[1, 2, 3, 0]]).as_matrix()
    z = clearance-float((points @ r.T)[:, 2].min())
    if name == "asset":
        instance = ctx.asset
        ctx.initial(instance, position=(*xy, z), rotation=rotation)
    else:
        instance = ctx.spawn(ctx.asset_path, name=name, position=(*xy, z), rotation=rotation)
    observe(ctx, instance, groups)
    return instance


def side_roll_release(ctx: TestContext):
    setup(ctx)
    groups = collision_groups(ctx)
    x_side = placed(ctx, "asset", (-.08, -.09), quaternion("y", math.pi/2), .0005, groups)
    y_side = placed(ctx, "cross", (.08, -.09), quaternion("x", -math.pi/2), .0005, groups)
    idle_x = placed(ctx, "idle_x", (-.08, .09), quaternion("y", math.pi/2), .0005, groups)
    idle_y = placed(ctx, "idle_y", (.08, .09), quaternion("x", -math.pi/2), .0005, groups)
    for name, instance, force, point, torque in (
            ("collar", x_side, (0, .008, 0), (-.009, 0, .051), (.000006, 0, 0)),
            ("nozzle", y_side, (.008, 0, 0), (0, -.00495, .075), (0, -.000006, 0))):
        ctx.force(name+"_push", instance.body(BODY), point_frame="body", keyframes=[
            {"time": 0, "force": (0, 0, 0), "point": point},
            {"time": .5, "force": (0, 0, 0)}, {"time": .54, "force": force},
            {"time": .74, "force": force}, {"time": .78, "force": (0, 0, 0)},
            {"time": 2., "force": (0, 0, 0)}])
        ctx.torque(name+"_twist", instance.body(BODY), keyframes=[
            {"time": 0, "torque": (0, 0, 0)}, {"time": .5, "torque": (0, 0, 0)},
            {"time": .54, "torque": torque}, {"time": .74, "torque": torque},
            {"time": .78, "torque": (0, 0, 0)}, {"time": 2., "torque": (0, 0, 0)}])
    ctx.camera(position=(.5, -.65, .60), target=(.025, .015, .025), focal_length=32)
    ctx.video_annotation = lambda: "front: collar/nozzle loaded | rear: idle controls | release at 0.78s"
    ctx.plots("collar_push.force.y", "nozzle_push.force.x", "asset.e0.spin", "cross.e0.spin",
              "asset.e0.energy", "cross.e0.energy", "idle_x.e0.speed")
    ctx.ensure_initialized()
    properties(ctx, (x_side, y_side, idle_x, idle_y))
    ctx.experiment_details.update(release_time_s=.78,
        rolling_model="faceted compound bottle; native angular damping only, no rolling friction added",
        contact_parts="per-part geometric gaps, not a contact force sensor")
    ctx.run(2.)
    checks = motion_checks(ctx, (x_side, y_side, idle_x, idle_y))
    time = column(ctx, "time")
    outcomes = []
    ctx.experiment_details["per_environment"] = outcomes
    for e in range(ctx.num_envs):
        item = {"env": e, "instances": {}}
        outcomes.append(item)
        for name, active in (("asset", True), ("cross", True), ("idle_x", False), ("idle_y", False)):
            p = f"{name}.e{e}"
            pos = np.column_stack([column(ctx, p+".com."+a) for a in "xyz"])
            start = np.flatnonzero(time >= .5-1e-9)[0]
            displacement = float(np.max(np.linalg.norm(pos[start:, :2]-pos[start, :2], axis=1)))
            released = column(ctx, p+".energy")[time >= .78-1e-9]
            spin = float(column(ctx, p+".spin")[time >= .5-1e-9].max())
            observation = {"horizontal_displacement_after_settling_m": displacement,
                "peak_spin_after_settling_rad_s": spin, "released_energy_J": float(released[0]),
                "max_released_energy_J": float(released.max()), "final_energy_J": float(released[-1]),
                "final_speed_m_s": float(column(ctx, p+".speed")[-1]),
                "final_spin_rad_s": float(column(ctx, p+".spin")[-1]),
                "closest_parts_final_m": {n: float(column(ctx, p+".gap."+n)[-1]) for n in groups}}
            item["instances"][name] = observation
            assert released.max() <= released[0]*1.05+2e-5, (p, observation)
            if active:
                assert displacement > .001 or spin > .3, (p, observation)
            else:
                assert displacement < .001 and spin < .15, (p, observation)
    ctx.metric("released_spin", max(v["instances"][n]["final_spin_rad_s"] for v in outcomes for n in ("asset", "cross")),
               unit="rad/s", label="撤力后末态最大余转")
    ctx.experiment_details["motion_checks"] = checks


def drop_orientations(ctx: TestContext):
    setup(ctx, dt=.001)
    groups = collision_groups(ctx)
    facet = Rotation.from_euler("z", math.pi/16)
    side_x = tuple((Rotation.from_euler("y", math.pi/2)*facet).as_quat()[[3, 0, 1, 2]])
    side_y = tuple((Rotation.from_euler("x", -math.pi/2)*facet).as_quat()[[3, 0, 1, 2]])
    nozzle = tuple((Rotation.from_euler("y", math.pi-.05)*facet).as_quat()[[3, 0, 1, 2]])
    cases = (("asset", (-.24, -.24), (1, 0, 0, 0)),
             ("side_x", (-.24, .24), side_x),
             ("side_y", (.24, -.24), side_y),
             ("nozzle_down", (.24, .24), nozzle))
    instances = [placed(ctx, name, xy, q, .01, groups) for name, xy, q in cases]
    ctx.camera(position=(.70, -1.10, 1.25), target=(.025, .015, .04), focal_length=24)
    ctx.video_annotation = lambda: "bottom / side-X / side-Y / nozzle-down | 10mm collision clearance"
    ctx.plots("asset.e0.clearance", "side_x.e0.clearance", "nozzle_down.e0.gap.nozzle_cap",
              "nozzle_down.e0.tilt", "nozzle_down.e0.energy", "side_y.e0.speed")
    ctx.ensure_initialized()
    properties(ctx, instances)
    ctx.experiment_details.update(release_clearance_m=.01,
        expected_first_parts={"asset": "base_rim", "side_x": "base_rim", "side_y": "base_rim", "nozzle_down": "nozzle_cap"},
        nozzle_final_pose="tipping is allowed; settling is checked, not upright retention")
    ctx.run(3.)
    ctx.experiment_details["instance_separation"] = separation_checks(ctx, instances, groups)
    motion_checks(ctx, instances, settle=True)
    outcomes = []
    ctx.experiment_details["per_environment"] = outcomes
    time = column(ctx, "time")
    for instance in instances:
        for e in range(ctx.num_envs):
            p = f"{instance.name}.e{e}"
            gap = column(ctx, p+".clearance")
            impact = np.flatnonzero(gap < .0003)
            initial_energy = float(column(ctx, p+".energy")[0])
            peak_energy = float(column(ctx, p+".energy").max())
            observation = {"instance": instance.name, "env": e, "initial_clearance_m": float(gap[0]),
                "first_contact_s": None if not len(impact) else float(time[impact[0]]),
                "initial_energy_J": initial_energy, "peak_energy_J": peak_energy,
                "final_tilt_rad": float(column(ctx, p+".tilt")[-1]),
                "nozzle_min_gap_m": float(column(ctx, p+".gap.nozzle_cap").min())}
            outcomes.append(observation)
            assert abs(gap[0]-.01) < 1e-6 and len(impact), observation
            assert peak_energy <= initial_energy*1.15+2e-5, observation
            if instance.name == "nozzle_down":
                assert column(ctx, p+".gap.nozzle_cap")[impact[0]] < .0003, observation
    ctx.metric("tip_contact", min(r["nozzle_min_gap_m"] for r in outcomes if r["instance"] == "nozzle_down")*1000,
               unit="mm", label="尖嘴朝下尖嘴最小间隙")


def separation_checks(ctx, instances, groups):
    """A positive AABB distance is a conservative no-contact certificate."""
    points = np.concatenate(list(groups.values()))
    results = []
    for e in range(ctx.num_envs):
        bounds = {}
        for instance in instances:
            prefix = f"{instance.name}.e{e}"
            rotations = np.asarray([[[row[prefix+f".r{i}{j}"] for j in range(3)] for i in range(3)] for row in ctx.rows])
            positions = np.column_stack([column(ctx, prefix+".position."+a) for a in "xyz"])
            world_points = np.einsum("tij,pj->tpi", rotations, points)+positions[:, None]
            bounds[instance.name] = (world_points.min(axis=1), world_points.max(axis=1))
        for i, a in enumerate(instances):
            for b in instances[i+1:]:
                amin, amax = bounds[a.name]
                bmin, bmax = bounds[b.name]
                distance = np.linalg.norm(np.maximum(np.maximum(amin-bmax, bmin-amax), 0), axis=1)
                minimum = float(distance.min())
                results.append({"env": e, "pair": [a.name, b.name], "minimum_aabb_separation_m": minimum})
                assert minimum > .005, results[-1]
    return results


def force_pulse(ctx, instance, name, point, start):
    tick = round(start/ctx.dt)
    ctx.force(name, instance.body(BODY), point_frame="body", keyframes=[
        {"time": tick*ctx.dt, "force": (.004, 0, 0), "point": tuple(point)},
        {"time": (tick+round(.04/ctx.dt))*ctx.dt, "force": (.004, 0, 0)}])


def free_checks(ctx, names, start=0., end=.4):
    outcomes = []
    ctx.experiment_details.setdefault("free_segments", []).append(outcomes)
    times = column(ctx, "time")
    selected = np.flatnonzero((times >= start-1e-9) & (times <= end+1e-9))
    boundary = np.flatnonzero(np.abs(times-start) < 1e-9)
    if len(boundary):
        selected = selected[selected >= boundary[-1]]
    for name, action in names:
        instance = ctx.session.instances[name]
        props = ctx.experiment_details["mass_properties"][name]
        for e in range(ctx.num_envs):
            p = f"{name}.e{e}"
            r = np.asarray([[[row[p+f".r{i}{j}"] for j in range(3)] for i in range(3)] for row in ctx.rows])[selected]
            v = np.column_stack([column(ctx, p+".v."+a) for a in "xyz"])[selected]
            w = np.column_stack([column(ctx, p+".w."+a) for a in "xyz"])[selected]
            mass = props["mass_kg"][e]
            inertia = np.asarray(props["local_inertia_kg_m2"][e])
            angular = np.einsum("tij,jk,tlk,tl->ti", r, inertia, r, w)
            expected_p, expected_l = mass*v[0].copy(), angular[0].copy()
            p_errors, l_errors = [], []
            impulse = np.zeros(3)
            angular_impulse = np.zeros(3)
            for local_index, row_index in enumerate(selected[1:], 1):
                row, previous = ctx.rows[row_index], ctx.rows[row_index-1]
                force = np.zeros(3) if action is None else np.array([row[action+".force."+a] for a in "xyz"])
                point = np.zeros(3) if action is None else np.array([row[action+".point."+a] for a in "xyz"])
                if action is not None:
                    own_rotation = np.asarray([[previous[p+f".r{i}{j}"] for j in range(3)] for i in range(3)])
                    own_origin = np.array([previous[p+".position."+a] for a in "xyz"])
                    point = own_origin+own_rotation @ np.asarray(ctx.actions[action]["frames"][0]["point"])
                center = np.array([previous[p+".com."+a] for a in "xyz"])
                moment = np.cross(point-center, force)
                expected_p = (expected_p+force*ctx.dt)*(1-props["linear_damping_s_inv"]*ctx.dt)
                expected_l = (expected_l+moment*ctx.dt)*(1-props["angular_damping_s_inv"]*ctx.dt)
                impulse += force*ctx.dt
                angular_impulse += moment*ctx.dt
                p_errors.append(float(np.linalg.norm(mass*v[local_index]-expected_p)))
                l_errors.append(float(np.linalg.norm(angular[local_index]-expected_l)))
            observation = {"env": e, "instance": name, "start_s": start, "end_s": end,
                "impulse_N_s": impulse.tolist(), "angular_impulse_N_m_s": angular_impulse.tolist(),
                "momentum_max_error_kg_m_s": max(p_errors), "angular_momentum_max_error_kg_m2_s": max(l_errors),
                "final_velocity_m_s": v[-1].tolist(), "final_angular_velocity_rad_s": w[-1].tolist(),
                "peak_speed_m_s": float(np.linalg.norm(v, axis=1).max()),
                "peak_spin_rad_s": float(np.linalg.norm(w, axis=1).max())}
            outcomes.append(observation)
            assert max(p_errors) < max(1e-9, .05*np.linalg.norm(impulse)), observation
            assert max(l_errors) < max(1e-10, .05*np.linalg.norm(angular_impulse)), observation
            if action is None:
                assert np.linalg.norm(v-v[0], axis=1).max() < 1e-5 and observation["peak_spin_rad_s"] < 1e-5, observation
            else:
                assert v[-1, 0] > .0075, observation
                if name == "central":
                    assert observation["peak_spin_rad_s"] < 1e-4, observation
                else:
                    assert w[-1, 1] > .15, observation
    return outcomes


def free_eccentric(ctx: TestContext):
    setup(ctx, gravity=(0, 0, 0))
    groups = collision_groups(ctx)
    ctx.initial(position=(-.03, 0, .12))
    central = ctx.spawn(ctx.asset_path, name="central", position=(-.03, -.065, .12))
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(-.03, .065, .12))
    for instance in (ctx.asset, central, idle):
        observe(ctx, instance, groups)
    ctx.camera(position=(.32, -.60, .38), target=(-.02, 0, .16), focal_length=40)
    ctx.video_annotation = lambda: "central / eccentric 10mm / idle | zero gravity | pulse 0.08-0.12s"
    ctx.ensure_initialized()
    props = properties(ctx, (ctx.asset, central, idle))
    com = np.asarray(props["asset"]["local_com_m_xyzw"][0][:3])
    force_pulse(ctx, ctx.asset, "eccentric_push", com+np.array([0, 0, .01]), .08)
    force_pulse(ctx, central, "central_push", com, .08)
    ctx.plots("eccentric_push.force.x", "eccentric_push.point.z", "asset.e0.v.x", "asset.e0.w.y",
              "central.e0.spin", "idle.e0.speed")
    ctx.experiment_details.update(pulse_force_N=[.004, 0, 0], pulse_interval_s=[.08, .12],
        eccentric_offset_from_com_m=[0, 0, .01],
        angular_prediction="integrate each step's world moment and damping; compare L=R I_body R^T omega, not constant world inertia")
    ctx.run(.4)
    outcomes = free_checks(ctx, (("asset", "eccentric_push"), ("central", "central_push"), ("idle", None)))
    ctx.metric("impulse_error", max(v["momentum_max_error_kg_m_s"] for v in outcomes), unit="kg*m/s", label="全环境冲量动量最大误差")
    ctx.metric("eccentric_spin", min(np.linalg.norm(v["final_angular_velocity_rad_s"]) for v in outcomes if v["instance"] == "asset"),
               unit="rad/s", label="偏心脉冲最小末态角速")


def lifecycle_reset_reload(ctx: TestContext):
    """Keep one stage and sibling through native reset and two true reloads."""
    import torch
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    setup(ctx, gravity=(0, 0, 0))
    groups = collision_groups(ctx)
    ctx.initial(position=(-.03, -.065, .12))
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(-.03, .065, .12))
    ctx.initial(idle, position=(-.03, .065, .12), linear_velocity=(.01, 0, 0))
    for instance in (ctx.asset, idle):
        observe(ctx, instance, groups)
    ctx.observe("loaded_present", lambda: float("asset" in ctx.session.instances), unit="1")
    ctx.camera(position=(.32, -.60, .38), target=(-.02, 0, .16), focal_length=40)
    ctx.video_annotation = lambda: "same stage | native stop/reset + two actual removals/references | moving sibling retained"
    ctx.plots("loaded_present", "asset.e0.v.x", "asset.e0.w.y", "idle.e0.speed", "idle.e0.position.x")
    ctx.ensure_initialized()
    properties(ctx, (ctx.asset, idle))
    session, stage = ctx.session, ctx.session.stage
    stage_id = stage.GetRootLayer().identifier
    subscriptions = (session.prepare_subscription, session.flush_subscription)

    def snapshot(instance):
        return instance.view.view.get_transforms().clone(), instance.view.view.get_velocities().clone()

    def restore(instance, saved):
        view = instance.view.view
        indices = torch.arange(len(saved[0]), device=saved[0].device, dtype=torch.int32)
        view.set_transforms(saved[0].clone(), indices)
        view.set_velocities(saved[1].clone(), indices)
        ctx.world.forward()
        session.refresh()
        return max(float((a-b).abs().max().item()) for a, b in zip(snapshot(instance), saved))

    def stop():
        session.active = False
        ctx.world._disable_app_control_on_stop_handle = True
        ctx.world.stop()

    def relative_structure(instance):
        cache = UsdGeom.XformCache()
        result = {}
        for e in range(ctx.num_envs):
            root = stage.GetPrimAtPath(session.replica_path(instance.body(BODY).path, e))
            inverse = cache.GetLocalToWorldTransform(root).GetInverse()
            for prim in Usd.PrimRange(root):
                if prim.GetCustomDataByKey("table1000:role") in ("part", "visual", "collision"):
                    result[(e, str(prim.GetPath()).split("/glue/bottle", 1)[-1])] = np.asarray(
                        cache.GetLocalToWorldTransform(prim)*inverse)
        return result

    original_asset, original_idle = snapshot(ctx.asset), snapshot(idle)
    structure = relative_structure(ctx.asset)
    events = []
    ctx.experiment_details["lifecycle_events"] = events

    def segment(index):
        # Old actions keep body handles even outside their active interval.
        # Rebind the intended segment before stepping a reset/reloaded stage.
        ctx.actions.clear()
        ctx.action_values.clear()
        start = session.time
        com = np.asarray(ctx.asset.local_com[0])
        action = "lifecycle_push"
        force_pulse(ctx, ctx.asset, action, com+np.array([0, 0, .01]), start+.08)
        ctx.run(.4)
        checks = free_checks(ctx, (("asset", action), ("idle", None)), start, start+.4)
        current = relative_structure(ctx.asset)
        error = max(float(np.abs(structure[n]-current[n]).max()) for n in structure)
        assert error < 1e-6 and len(current) == len(structure), error
        assert session.callback_steps == session.steps
        return {"segment": index, "start_s": start, "end_s": session.time,
                "checks": checks, "relative_structure_max_error": error}

    events.append(segment(0))
    stop()
    ctx.world.reset()
    session.initialize()
    errors = {"asset": restore(ctx.asset, original_asset), "idle": restore(idle, original_idle)}
    assert max(errors.values()) < 1e-6, errors
    events.append({"native_stop_reset": True, "state_errors": errors})
    # Segment checks start at the first post-reset sample; capture the restored
    # boundary explicitly so impulse integration starts from the true state.
    observe(ctx, ctx.asset, groups)
    observe(ctx, idle, groups)
    ctx.sample()
    events.append(segment(1))
    for cycle in range(2):
        saved_idle = snapshot(idle)
        old = ctx.asset
        stop()
        ctx.actions.clear()
        ctx.action_values.clear()
        paths = [session.replica_path(old.path, e) for e in range(ctx.num_envs)]
        for path in paths:
            stage.RemovePrim(path)
        session.unregister(old.path)
        assert all(not stage.GetPrimAtPath(p) for p in paths)
        assert all(stage.GetPrimAtPath(session.replica_path(idle.path, e)) for e in range(ctx.num_envs))
        ctx.world.reset()
        session.initialize()
        retain_error = restore(idle, saved_idle)
        assert retain_error < 1e-6, retain_error
        absent_start = session.time
        ctx.run(.2)
        after_absent = snapshot(idle)
        displacement = (after_absent[0][:, :3]-saved_idle[0][:, :3]).detach().cpu().numpy()
        absent_error = float(np.abs(displacement-np.array([.002, 0, 0])).max())
        assert absent_error < 1e-6, absent_error
        stop()
        replacement = ctx.spawn(ctx.asset_path, name="asset", position=(-.03, -.065, .12))
        ctx.asset = replacement
        for e in range(1, ctx.num_envs):
            assert Sdf.CopySpec(stage.GetRootLayer(), replacement.path, stage.GetRootLayer(), session.replica_path(replacement.path, e))
        ctx.world.reset()
        session.initialize()
        native_initial_error = max(float((a-b).abs().max().item()) for a, b in zip(snapshot(replacement), original_asset))
        assert native_initial_error < 1e-6, native_initial_error
        errors = {"asset": restore(replacement, original_asset), "idle": restore(idle, after_absent)}
        assert max(errors.values()) < 1e-6, errors
        assert replacement is not old and replacement.body_names == [BODY] and not replacement.joint_names
        assert stage.GetRootLayer().identifier == stage_id
        assert subscriptions == (session.prepare_subscription, session.flush_subscription)
        counts = [sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(stage.GetPrimAtPath(session.environment(e))))
                  for e in range(ctx.num_envs)]
        assert counts == [2]*ctx.num_envs, counts
        observe(ctx, replacement, groups)
        observe(ctx, idle, groups)
        ctx.sample()
        properties(ctx, (replacement, idle))
        events.append({"reload_cycle": cycle+1, "unloaded_at_s": absent_start,
            "reloaded_at_s": session.time, "sibling_prims_retained": True,
            "sibling_state_error_during_absence": absent_error, "state_errors": errors,
            "native_reference_initial_error": native_initial_error,
            "bodies_per_environment": counts})
        events.append(segment(cycle+2))
    ctx.experiment_details.update(same_stage=True, same_subscriptions=True,
        state_writes="stopped/reset initialization only; force-driven segments never write poses",
        absent_channels="NaN for the removed asset; loaded_present=0; idle remains sampled",
        media_scope="full native reset, two absent/reloaded intervals and all force segments",
        performance_scope="lifecycle only; excluded from steady-state benchmark")
    ctx.metric("reload_cycles", 2, unit="1", label="同stage真正卸载重载次数")


TESTS = {"upright_push_tip": upright_push_tip, "side_roll_release": side_roll_release,
         "drop_orientations": drop_orientations, "free_eccentric": free_eccentric,
         "lifecycle_reset_reload": lifecycle_reset_reload}
