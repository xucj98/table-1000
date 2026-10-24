"""Single rigid glue-stick experiments, with every environment checked in SI."""

import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.physics.testing import TestContext

BODY = "glue.stick"
MASS = .025


def geometry(ctx):
    model = json.loads((ctx.asset_path.parent / "model.json").read_text())
    worlds, groups = {}, {}
    for node in model["nodes"]:
        worlds[node["name"]] = worlds.get(node["parent"], np.eye(4)) @ np.asarray(node["matrix"])
        if node.get("role") == "collision":
            transform = np.linalg.inv(worlds[BODY]) @ worlds[node["name"]]
            groups[node["parent"].rsplit(".", 1)[-1]] = (
                np.c_[node["vertices"], np.ones(len(node["vertices"]))] @ transform.T)[:, :3]
    assert set(groups) == {"cap", "cap_rim", "container", "shoulder"}
    return groups


def setup(ctx, gravity=(0, 0, -9.81), dt=.004, gpu_solver=None):
    ctx.simulation(dt=dt, gravity=gravity, gpu_solver=gpu_solver)
    if np.linalg.norm(gravity):
        ctx.ground(static_friction=.45, dynamic_friction=.35, restitution=0., contact_offset=.0001)
    ctx.experiment_details = {"contract": "glue-000001-v5", "dt_s": ctx.dt,
        "force_source": "commanded external force, not contact reaction",
        "calibration": "uncalibrated engineering estimates"}


def observe(ctx, instance, groups):
    ctx.observe_body(instance.body(BODY))
    cache = {}

    def state():
        if cache.get("step") != ctx.session.steps:
            view = instance.view.view
            positions, rotations, centers = instance.environment_states()
            velocity = view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 6).astype(float)
            inertia = view.get_inertias().detach().cpu().numpy().reshape(ctx.num_envs, 3, 3)
            mass = view.get_masses().detach().cpu().numpy().reshape(ctx.num_envs)
            r = rotations[:, 0]
            world_i = r @ inertia @ r.transpose(0, 2, 1)
            gaps = {n: positions[:, 0, 2] + np.min(
                np.einsum("eij,pj->epi", r, points)[:, :, 2], axis=1) for n, points in groups.items()}
            cache.update(step=ctx.session.steps, position=positions[:, 0], com=centers[:, 0], r=r,
                v=velocity[:, :3], w=velocity[:, 3:],
                momentum=mass[:, None]*velocity[:, :3],
                angular_momentum=np.einsum("eij,ej->ei", world_i, velocity[:, 3:]),
                clearance=np.min(list(gaps.values()), axis=0), gaps=gaps,
                tilt=np.arccos(np.clip(r[:, 2, 2], -1, 1)),
                energy=.5*mass*np.sum(velocity[:, :3]**2, axis=1)
                + .5*np.einsum("ei,eij,ej->e", velocity[:, 3:], world_i, velocity[:, 3:])
                - mass*(centers[:, 0] @ np.asarray(ctx.gravity)))
        return cache

    for e in range(ctx.num_envs):
        prefix = f"{instance.name}.e{e}"
        for n, unit in (("clearance", "m"), ("energy", "J"), ("tilt", "rad")):
            ctx.observe(prefix+"."+n, lambda e=e, n=n: state()[n][e], unit=unit)
        for n, unit in (("v", "m/s"), ("w", "rad/s")):
            ctx.observe(prefix+(".speed" if n == "v" else ".spin"),
                        lambda e=e, n=n: np.linalg.norm(state()[n][e]), unit=unit)
        for n in groups:
            ctx.observe(prefix+".gap."+n, lambda e=e, n=n: state()["gaps"][n][e], unit="m")
        for n, unit in (("position", "m"), ("com", "m"), ("v", "m/s"), ("w", "rad/s"),
                        ("momentum", "kg*m/s"), ("angular_momentum", "kg*m2/s")):
            for j, axis in enumerate("xyz"):
                ctx.observe(prefix+"."+n+"."+axis, lambda e=e, n=n, j=j: state()[n][e, j], unit=unit)
        for j in range(3):
            for k in range(3):
                ctx.observe(prefix+f".r{j}{k}", lambda e=e, j=j, k=k: state()["r"][e, j, k], unit="1")
    return state


def properties(ctx, instances):
    from pxr import PhysxSchema
    result = {}
    for instance in instances:
        view = instance.view.view
        api = PhysxSchema.PhysxRigidBodyAPI(ctx.session.stage.GetPrimAtPath(instance.body(BODY).path))
        result[instance.name] = {
            "mass_kg": view.get_masses().detach().cpu().numpy().reshape(ctx.num_envs).tolist(),
            "local_com_m_xyzw": view.get_coms().detach().cpu().numpy().reshape(ctx.num_envs, 7).tolist(),
            "local_inertia_kg_m2": view.get_inertias().detach().cpu().numpy().reshape(ctx.num_envs, 3, 3).tolist(),
            "linear_damping_s_inv": api.GetLinearDampingAttr().Get(),
            "angular_damping_s_inv": api.GetAngularDampingAttr().Get(),
            "rigid_bodies": instance.body_names, "joints": instance.joint_names}
        assert instance.body_names == [BODY] and not instance.joint_names
        assert np.max(np.abs(np.asarray(result[instance.name]["mass_kg"])-MASS)) < 1e-7
    ctx.experiment_details["mass_properties"] = result
    return result


def column(ctx, name):
    return np.asarray([row[name] for row in ctx.rows])


def motion_checks(ctx, instances, *, settle=False):
    results = {}
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
            assert np.all(np.isfinite(list(result.values()))), (p, result)
            assert result["penetration_m"] < .0015, (p, result)
            assert result["peak_speed_m_s"] < 2 and result["peak_spin_rad_s"] < 80, (p, result)
            if settle:
                assert result["tail_speed_m_s"] < .02 and result["tail_spin_rad_s"] < .2, (p, result)
    ctx.metric("penetration", max(v["penetration_m"] for v in results.values())*1000,
               unit="mm", label="全环境最大碰撞几何穿透")
    return results


def upright_push_tip(ctx: TestContext):
    dt = .001 if ctx.physics == "cpu" else .002
    setup(ctx, dt=dt, gpu_solver="PGS" if ctx.physics == "gpu" else None)
    ctx.experiment_details["step_size_exception"] = {
        "baseline_dt_s": .004, "candidate_dt_s": dt,
        "baseline_CPU_tip_impact_penetration_m": .0036913070871812323,
        "baseline_GPU_tip_impact_penetration_m": .003940227292404758,
        "CPU_2ms_tip_impact_penetration_m": .0018930067089008417,
        "GPU_2ms_TGS_failure": "low-push env1 remains unsettled after controlled braking",
        "GPU_solver_candidate": "PGS, also verified with the same object's 50 mm drop",
        "reason": "resolve high-push fall impact; controlled low-push braking already stable at 4 ms"}
    groups = geometry(ctx)
    ctx.initial(position=(0, -.15, .0005))
    high = ctx.spawn(ctx.asset_path, name="high", position=(0, 0, .0005))
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(0, .15, .0005))
    instances = (ctx.asset, high, idle)
    for instance in instances:
        observe(ctx, instance, groups)
    for name, instance, point, maximum, ramp_end, hold_end, release in (
            ("low_push", ctx.asset, (-.0125, 0, .008), .115, 1.4, 1.42, 1.44),
            ("high_push", high, (-.0136, 0, .09), .065, 1.6, 1.7, 1.72)):
        frames = [
            {"time": 0., "force": (0, 0, 0), "point": point},
            {"time": .5, "force": (0, 0, 0)},
            {"time": .8, "force": (.03, 0, 0)},
            {"time": ramp_end, "force": (maximum, 0, 0)},
            {"time": hold_end, "force": (maximum, 0, 0)}]
        if name == "low_push":
            # Keep the sliding reaction inside the base during deceleration.
            frames += [{"time": 1.52, "force": (.075, 0, 0)},
                       {"time": 2.3, "force": (.075, 0, 0)},
                       {"time": 2.4, "force": (0, 0, 0)}]
        else:
            frames += [{"time": release, "force": (0, 0, 0)}]
        frames += [{"time": 3.4, "force": (0, 0, 0)}]
        ctx.force(name, instance.body(BODY), point_frame="body", keyframes=frames)
    ctx.camera(position=(.29, -.67, .32), target=(.07, 0, .04), focal_length=28)
    ctx.plots("low_push.force.x", "high_push.force.x", "asset.e0.position.x", "high.e0.tilt",
              "idle.e0.position.x", "asset.e0.speed", "high.e0.clearance")
    ctx.ensure_initialized()
    properties(ctx, instances)
    radius = float(groups["container"][:, 0].max())
    ctx.experiment_details.update(support_radius_x_m=radius, nominal_slide_threshold_N=MASS*9.81*.45,
        nominal_tip_thresholds_N={"low": MASS*9.81*radius/.008, "high": MASS*9.81*radius/.09},
        release_time_s={"low": 2.4, "high": 1.72}, instances=["asset=low8mm", "high=90mm", "idle"],
        low_braking_force_N=.075, low_braking_interval_s=[1.52, 2.3],
        braking_basis="x_COP=[F*(h-zCOM)+mu*m*g*zCOM]/(m*g); use static friction upper bound .45")
    ctx.run(3.4)
    checks = motion_checks(ctx, instances, settle=True)
    time = column(ctx, "time")
    outcomes = []
    for e in range(ctx.num_envs):
        lp, hp, ip = (f"{n}.e{e}" for n in ("asset", "high", "idle"))
        moving = np.flatnonzero((time > .8) & (np.abs(column(ctx, lp+".v.x")) > .005)
                               & (np.abs(column(ctx, lp+".position.x")-column(ctx, lp+".position.x")[0]) > .0002))
        onset = None if not len(moving) else int(moving[0])
        row = {"env": e, "low_slide_m": float(np.ptp(column(ctx, lp+".position.x"))),
            "low_peak_tilt_rad": float(column(ctx, lp+".tilt").max()),
            "high_peak_tilt_rad": float(column(ctx, hp+".tilt").max()),
            "idle_shift_m": float(np.ptp(column(ctx, ip+".position.x"))),
            "weak_force_shift_m": float(np.ptp(column(ctx, lp+".position.x")[time <= .8+1e-9])),
            "slide_onset_time_s": None if onset is None else float(time[onset]),
            "slide_onset_force_N": None if onset is None else float(ctx.rows[onset]["low_push.force.x"])}
        outcomes.append(row)
        assert row["low_slide_m"] > .003 and row["low_peak_tilt_rad"] < math.radians(20), row
        assert row["high_peak_tilt_rad"] > math.radians(60), row
        assert row["idle_shift_m"] < .001 and column(ctx, ip+".tilt").max() < math.radians(3), row
        assert row["weak_force_shift_m"] < .001, row
    ctx.experiment_details.update(per_environment=outcomes, motion_checks=checks)
    ctx.metric("low_slide", max(r["low_slide_m"] for r in outcomes)*1000, unit="mm", label="低位滑移")
    ctx.metric("high_tilt", min(r["high_peak_tilt_rad"] for r in outcomes)*180/math.pi, unit="deg", label="高位最小峰值倾角")
    ctx.metric("slide_onset", max(r["slide_onset_force_N"] for r in outcomes), unit="N", label="低位起动推力")


def quaternion(matrix):
    return tuple(Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]])


def descendants(ctx, instances):
    from pxr import Usd, UsdGeom
    cache = UsdGeom.XformCache()
    result = {}
    for instance in instances:
        for e in range(ctx.num_envs):
            root = ctx.session.replica_path(instance.body(BODY).path, e)
            inv = cache.GetLocalToWorldTransform(ctx.session.stage.GetPrimAtPath(root)).GetInverse()
            for prim in Usd.PrimRange(ctx.session.stage.GetPrimAtPath(root)):
                if prim.GetCustomDataByKey("table1000:role") in ("part", "visual", "collision"):
                    result[str(prim.GetPath())] = np.asarray(cache.GetLocalToWorldTransform(prim)*inv)
    return result


def structure_check(ctx, before, instances):
    after = descendants(ctx, instances)
    assert before.keys() == after.keys()
    error = max(float(np.max(np.abs(before[k]-after[k]))) for k in before)
    assert error < 1e-6, error
    for instance in instances:
        assert instance.body_names == [BODY] and not instance.joint_names
    ctx.experiment_details["structure"] = {"descendant_nodes_checked": len(before),
        "max_relative_transform_error": error, "rigid_bodies_per_instance": 1,
        "joints_per_instance": 0, "behavior": False}
    return error


def side_roll_release(ctx: TestContext):
    setup(ctx)
    groups = geometry(ctx)
    r = Rotation.from_euler("y", math.pi/2).as_matrix()
    q = quaternion(r)
    points = np.concatenate(list(groups.values()))
    z = .0005-float((points @ r.T)[:, 2].min())
    ctx.initial(position=(-.25, -.15, z), rotation=q)
    axial = ctx.spawn(ctx.asset_path, name="axial", position=(.03, -.15, z), rotation=q)
    oblique = ctx.spawn(ctx.asset_path, name="oblique", position=(-.25, .14, z), rotation=q)
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(.03, .14, z), rotation=q)
    instances = (ctx.asset, axial, oblique, idle)
    for instance in instances:
        state = observe(ctx, instance, groups)
        for e in range(ctx.num_envs):
            p = f"{instance.name}.e{e}"
            ctx.observe(p+".axis_elevation", lambda e=e, s=state:
                        np.arcsin(np.clip(s()["r"][e, 2, 2], -1, 1)), unit="rad")
            ctx.observe(p+".axis_heading", lambda e=e, s=state:
                        np.arctan2(s()["r"][e, 1, 2], s()["r"][e, 0, 2]), unit="rad")
            ctx.observe(p+".axial_speed", lambda e=e, s=state:
                        np.dot(s()["v"][e], s()["r"][e, :, 2]), unit="m/s")
            ctx.observe(p+".roll_spin", lambda e=e, s=state:
                        np.dot(s()["w"][e], s()["r"][e, :, 2]), unit="rad/s")

            def slip(e=e, s=state):
                value = s()
                world = points @ value["r"][e].T + value["position"][e]
                support = world[world[:, 2] < world[:, 2].min()+.0001].mean(axis=0)
                return np.linalg.norm((value["v"][e]+np.cross(value["w"][e], support-value["com"][e]))[:2])

            ctx.observe(p+".geometric_support_slip_speed", slip, unit="m/s")
    forces = (("roll", ctx.asset, (0, .07, 0)), ("axial_push", axial, (.14, 0, 0)),
              ("oblique_push", oblique, (.12, .07, 0)))
    for name, instance, force in forces:
        ctx.force(name, instance.body(BODY), keyframes=[
            {"time": 0, "force": (0, 0, 0)}, {"time": .5, "force": (0, 0, 0)},
            {"time": .6, "force": force}, {"time": .9, "force": force},
            {"time": 1., "force": (0, 0, 0)}, {"time": 2.4, "force": (0, 0, 0)}])
    ctx.camera(position=(.03, .28, 1.5), target=(.03, .28, 0), up=(0, 1, 0), focal_length=14)
    ctx.plots("roll.force.y", "asset.e0.roll_spin", "asset.e0.geometric_support_slip_speed",
              "axial.e0.axial_speed", "oblique.e0.axis_heading", "asset.e0.axis_elevation",
              "asset.e0.gap.cap_rim", "asset.e0.gap.container", "idle.e0.speed")
    ctx.ensure_initialized()
    properties(ctx, instances)
    before = descendants(ctx, instances)
    ctx.run(2.4)
    checks = motion_checks(ctx, instances)
    outcomes = []
    time = column(ctx, "time")
    for e in range(ctx.num_envs):
        for instance in instances:
            p = f"{instance.name}.e{e}"
            xyz = np.array([column(ctx, p+".com."+a) for a in "xyz"]).T
            late = time >= 1.0
            row = {"env": e, "instance": instance.name,
                "displacement_m": float(np.linalg.norm(xyz[-1]-xyz[0])),
                "peak_roll_spin_rad_s": float(np.abs(column(ctx, p+".roll_spin")).max()),
                "released_heading_range_rad": float(np.ptp(np.unwrap(column(ctx, p+".axis_heading")[late]))),
                "released_axis_elevation_range_rad": float(np.ptp(column(ctx, p+".axis_elevation")[late])),
                "released_support_slip_peak_m_s": float(column(ctx, p+".geometric_support_slip_speed")[late].max()),
                "end_part_gaps_m": {n: float(column(ctx, p+".gap."+n)[-1]) for n in groups},
                "energy_at_release_J": float(column(ctx, p+".energy")[np.flatnonzero(late)[0]]),
                "energy_end_J": float(column(ctx, p+".energy")[-1])}
            outcomes.append(row)
            if instance is not idle:
                assert row["displacement_m"] > .001, row
            if instance is ctx.asset:
                assert row["peak_roll_spin_rad_s"] > .2, row
            if instance is idle:
                assert row["displacement_m"] < .0015, row
                assert row["peak_roll_spin_rad_s"] < .05, row
            assert row["energy_end_J"] < row["energy_at_release_J"]+.0005, row
    structure_check(ctx, before, instances)
    ctx.experiment_details.update(per_environment=outcomes, motion_checks=checks, release_time_s=1.,
        loading="world transverse/axial/oblique forces at actual COM; idle has no force", duration_s=2.4,
        slip_method="velocity of geometric lowest-vertex centroid; not a contact-force sensor",
        nominal_radii_m={"cap": .0136, "cap_rim": .014, "container": .0125},
        acceptance="no required stopping time or straight-line rolling; no rolling damping added")
    ctx.metric("roll_spin", min(r["peak_roll_spin_rad_s"] for r in outcomes if r["instance"] == "asset"),
               unit="rad/s", label="横向加载最小峰值滚转角速")
    ctx.metric("roll_sway", max(r["released_axis_elevation_range_rad"] for r in outcomes if r["instance"] == "asset")*180/math.pi,
               unit="deg", label="撤力后轴线摆动范围")


def drop_orientations(ctx: TestContext):
    # The original 4 ms impacts exceed the unchanged 1.5 mm overlap contract.
    setup(ctx, dt=.002, gpu_solver="PGS" if ctx.physics == "gpu" else None)
    ctx.experiment_details["step_size_exception"] = {
        "baseline_dt_s": .004, "candidate_dt_s": ctx.dt,
        "baseline_CPU_side_penetration_m": .0017973055000946037,
        "baseline_GPU_bottom_penetration_m": .0030848942697048187,
        "GPU_2ms_failure": "penetration 1.047 mm but late bottom-down angular growth and unsettled motion",
        "GPU_1ms_failure": "penetration 0.135 mm but late bottom-down angular growth and unsettled motion",
        "GPU_solver_candidate": "PGS; TGS at 4/2/1 ms retained as failed evidence",
        "reason": "same 50 mm drop, orientations, materials and thresholds; resolve impact at a smaller step"}
    groups = geometry(ctx)
    points = np.concatenate(list(groups.values()))
    configs = (("asset", (-.18, -.15), np.eye(3)),
               ("cap_down", (.08, -.15), Rotation.from_euler("x", math.pi).as_matrix()),
               ("side", (-.18, .15), Rotation.from_euler("y", math.pi/2).as_matrix()),
               ("oblique", (.08, .15), Rotation.from_euler("xy", [math.pi/6, math.pi/4]).as_matrix()))
    instances = []
    for name, xy, r in configs:
        position = (*xy, .05-float((points @ r.T)[:, 2].min()))
        if name == "asset":
            instance = ctx.asset
            ctx.initial(position=position, rotation=quaternion(r))
        else:
            instance = ctx.spawn(ctx.asset_path, name=name, position=position, rotation=quaternion(r))
        instances.append(instance)
        observe(ctx, instance, groups)
    ctx.camera(position=(.43, -.82, .58), target=(0, 0, .04), focal_length=35)
    ctx.plots("asset.e0.clearance", "cap_down.e0.clearance", "side.e0.clearance", "oblique.e0.clearance",
              "cap_down.e0.speed", "side.e0.spin", "oblique.e0.energy")
    ctx.ensure_initialized()
    properties(ctx, instances)
    before = descendants(ctx, instances)
    ctx.run(3.)
    checks = motion_checks(ctx, instances, settle=True)
    time = column(ctx, "time")
    outcomes = []
    for e in range(ctx.num_envs):
        for instance in instances:
            p = f"{instance.name}.e{e}"
            gap = column(ctx, p+".clearance")
            impact = np.flatnonzero(gap < .0002)
            assert len(impact), p
            index = int(impact[0])
            energy = column(ctx, p+".energy")
            row = {"env": e, "instance": instance.name, "first_impact_s": float(time[index]),
                "rebound_clearance_m": max(0., float(gap[index:].max())),
                "end_part_gaps_m": {n: float(column(ctx, p+".gap."+n)[-1]) for n in groups},
                "final_tilt_rad": float(column(ctx, p+".tilt")[-1]),
                "initial_energy_J": float(energy[0]), "final_energy_J": float(energy[-1])}
            outcomes.append(row)
            assert row["rebound_clearance_m"] < .015, row
            assert energy[index:].max() < energy[0]+.0005, row
            assert abs(gap[-1]) < .0015, row
    structure_check(ctx, before, instances)
    ctx.experiment_details.update(per_environment=outcomes, motion_checks=checks, drop_clearance_m=.05)
    ctx.metric("rebound", max(r["rebound_clearance_m"] for r in outcomes)*1000,
               unit="mm", label="落放最大几何离地反弹")


def free_eccentric(ctx: TestContext):
    from table_1000.physics.testing import interpolate
    setup(ctx, gravity=(0, 0, 0))
    groups = geometry(ctx)
    ctx.initial(position=(-.16, 0, .18))
    eccentric = ctx.spawn(ctx.asset_path, name="eccentric", position=(0, 0, .18))
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(.16, 0, .18))
    instances = (ctx.asset, eccentric, idle)
    for instance in instances:
        observe(ctx, instance, groups)
    ctx.camera(position=(.36, -.7, .46), target=(0, .04, .23), focal_length=38)
    ctx.ensure_initialized()
    props = properties(ctx, instances)
    local_com = np.asarray(props["eccentric"]["local_com_m_xyzw"])[0, :3]
    force = np.array([0., .02, 0.])
    frames = [{"time": 0, "force": (0, 0, 0)}, {"time": .2, "force": (0, 0, 0)},
              {"time": .204, "force": tuple(force)}, {"time": .404, "force": tuple(force)},
              {"time": .408, "force": (0, 0, 0)}, {"time": .8, "force": (0, 0, 0)}]
    point = local_com+np.array([0, 0, .006])
    ctx.force("center_pulse", ctx.asset.body(BODY), keyframes=frames)
    ctx.force("eccentric_pulse", eccentric.body(BODY), point_frame="body",
              keyframes=[{**f, "point": tuple(point)} for f in frames])
    ctx.plots("center_pulse.force.y", "asset.e0.v.y", "eccentric.e0.w.x",
              "eccentric.e0.angular_momentum.x", "idle.e0.speed", "eccentric.e0.energy")
    before = descendants(ctx, instances)
    expected_p = np.zeros((ctx.num_envs, 3))
    expected_h = np.zeros((ctx.num_envs, 3))
    peak_p_error = peak_h_error = 0.
    angular_damping = props["eccentric"]["angular_damping_s_inv"]
    for _ in range(round(.8/ctx.dt)):
        value = interpolate(frames, ctx.session.time)
        applied = np.asarray(value["force"] if value is not None else (0, 0, 0))
        positions, rotations, centers = eccentric.environment_states()
        world_point = positions[:, 0]+np.einsum("eij,j->ei", rotations[:, 0], point)
        torque = np.cross(world_point-centers[:, 0], applied)
        expected_p += applied*ctx.dt
        expected_h = expected_h*(1-angular_damping*ctx.dt)+torque*ctx.dt
        ctx.step()
        for e in range(ctx.num_envs):
            row = ctx.rows[-1]
            p = np.array([row[f"eccentric.e{e}.momentum.{a}"] for a in "xyz"])
            h = np.array([row[f"eccentric.e{e}.angular_momentum.{a}"] for a in "xyz"])
            peak_p_error = max(peak_p_error, float(np.linalg.norm(p-expected_p[e])))
            peak_h_error = max(peak_h_error, float(np.linalg.norm(h-expected_h[e])))
    outcomes = []
    impulse = float(np.linalg.norm(expected_p[0]))
    angular_impulse = float(np.linalg.norm(expected_h[0]))
    for e in range(ctx.num_envs):
        row = {"env": e, "expected_momentum_kg_m_s": expected_p[e].tolist(),
            "expected_angular_momentum_kg_m2_s": expected_h[e].tolist(),
            "center_peak_spin_rad_s": float(column(ctx, f"asset.e{e}.spin").max()),
            "eccentric_peak_spin_rad_s": float(column(ctx, f"eccentric.e{e}.spin").max()),
            "idle_peak_speed_m_s": float(column(ctx, f"idle.e{e}.speed").max())}
        center_p = np.array([ctx.rows[-1][f"asset.e{e}.momentum.{a}"] for a in "xyz"])
        row["center_momentum_relative_error"] = float(np.linalg.norm(center_p-expected_p[e])/impulse)
        outcomes.append(row)
        assert row["center_momentum_relative_error"] < .01, row
        assert row["center_peak_spin_rad_s"] < .001 and row["idle_peak_speed_m_s"] < 1e-6, row
        assert row["eccentric_peak_spin_rad_s"] > .2, row
    structure_check(ctx, before, instances)
    ctx.experiment_details.update(per_environment=outcomes,
        peak_momentum_relative_error=peak_p_error/impulse,
        peak_angular_momentum_relative_error=peak_h_error/angular_impulse,
        angular_damping_s_inv=angular_damping, eccentric_local_point_m=point.tolist(),
        prediction="discrete pre-step world torque about actual COM; native angular damping retained")
    ctx.metric("momentum_error", peak_p_error/impulse*100, unit="%", label="线动量最大相对误差")
    ctx.metric("angular_momentum_error", peak_h_error/angular_impulse*100, unit="%", label="角动量最大相对误差")
    assert peak_p_error/impulse < .01
    assert peak_h_error/angular_impulse < .03


def lifecycle_reset_reload(ctx: TestContext):
    """Three full segments; state writes happen only at stopped initialization."""
    import torch
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils
    asset_stage = Usd.Stage.Open(str(ctx.asset_path))
    rigids = [p for p in asset_stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    colliders = [p for p in asset_stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(rigids) == 1 and len(colliders) == 4
    assert not any(p.IsA(UsdPhysics.Joint) for p in asset_stage.Traverse())
    assert not UsdUtils.ComputeAllDependencies(str(ctx.asset_path))[2]
    assert UsdGeom.GetStageMetersPerUnit(asset_stage) == 1
    assert UsdGeom.GetStageUpAxis(asset_stage) == "Z"
    for collider in colliders:
        material, _ = UsdShade.MaterialBindingAPI(collider).ComputeBoundMaterial("physics")
        api = UsdPhysics.MaterialAPI(material)
        assert np.allclose([api.GetStaticFrictionAttr().Get(), api.GetDynamicFrictionAttr().Get(),
                            api.GetRestitutionAttr().Get()], [.45, .35, 0.])
    setup(ctx, gravity=(0, 0, 0))
    groups = geometry(ctx)
    ctx.initial(position=(-.08, 0, .16))
    idle = ctx.spawn(ctx.asset_path, name="idle", position=(.08, 0, .16))
    stage = ctx.session.stage
    stage_id = stage.GetRootLayer().identifier
    phase = [1]
    ctx.observe("lifecycle.phase", lambda: phase[0], unit="1")
    ctx.camera(position=(.28, -.5, .38), target=(0, .025, .21), focal_length=38)
    ctx.ensure_initialized()
    properties(ctx, (ctx.asset, idle))

    def snapshot():
        return {n: (a.view.view.get_transforms().clone(), a.view.view.get_velocities().clone())
                for n, a in ctx.session.instances.items()}

    def restore(saved):
        for n, (poses, speeds) in saved.items():
            a = ctx.session.instances[n]
            ids = torch.arange(len(poses), dtype=torch.int32, device=poses.device)
            a.view.view.set_transforms(poses.clone(), ids)
            a.view.view.set_velocities(speeds.clone(), ids)
        ctx.world.forward()
        ctx.session.refresh()

    def difference(a, b):
        return max(float(torch.max(torch.abs(a[n][i]-b[n][i])).cpu()) for n in a for i in (0, 1))

    def configure():
        ctx.actions.clear()
        ctx.action_values.clear()
        for a in (ctx.asset, idle):
            observe(ctx, a, groups)
        t = ctx.session.time
        ctx.force("lifecycle_pulse", ctx.asset.body(BODY), keyframes=[
            {"time": t, "force": (0, 0, 0)}, {"time": t+.2, "force": (0, 0, 0)},
            {"time": t+.204, "force": (0, .012, 0)},
            {"time": t+.404, "force": (0, .012, 0)},
            {"time": t+.408, "force": (0, 0, 0)}, {"time": t+1.2, "force": (0, 0, 0)}])
        ctx.plots("lifecycle.phase", "lifecycle_pulse.force.y", "asset.e0.v.y", "idle.e0.speed",
                  "asset.e0.spin")

    def segment():
        start = len(ctx.rows)
        count = ctx.session.callback_steps
        configure()
        before = descendants(ctx, (ctx.asset, idle))
        ctx.run(1.2)
        assert ctx.session.callback_steps-count == round(1.2/ctx.dt)
        rows = ctx.rows[start:]
        impulse = sum(r["lifecycle_pulse.force.y"] for r in rows)*ctx.dt
        result = []
        for e in range(ctx.num_envs):
            momentum = rows[-1][f"asset.e{e}.momentum.y"]
            row = {"environment": e, "relative_momentum_error": abs(momentum-impulse)/impulse,
                   "idle_max_speed_m_s": max(r[f"idle.e{e}.speed"] for r in rows),
                   "center_max_spin_rad_s": max(r[f"asset.e{e}.spin"] for r in rows)}
            assert row["relative_momentum_error"] < .01, row
            assert row["idle_max_speed_m_s"] < 1e-6 and row["center_max_spin_rad_s"] < .001, row
            result.append(row)
        structure_check(ctx, before, (ctx.asset, idle))
        return {"phase": phase[0], "start_row": start, "rows": len(rows),
                "per_environment": result, "callback_steps": ctx.session.callback_steps-count}

    saved = snapshot()
    results = [segment()]
    first_final = snapshot()
    ctx.session.active = False
    ctx.world._disable_app_control_on_stop_handle = True
    ctx.world.stop()
    ctx.world.reset()
    ctx.session.initialize()
    restore(saved)
    reset_error = difference(saved, snapshot())
    assert reset_error < 1e-6
    phase[0] = 2
    results.append(segment())
    repeat_error = difference(first_final, snapshot())
    assert repeat_error < 1e-6
    ctx.session.active = False
    ctx.world._disable_app_control_on_stop_handle = True
    ctx.world.stop()
    old = ctx.asset
    paths = [ctx.session.replica_path(old.path, e) for e in range(ctx.num_envs)]
    sibling_paths = [ctx.session.replica_path(idle.path, e) for e in range(ctx.num_envs)]
    for path in paths:
        stage.RemovePrim(path)
    ctx.session.unregister(old.path)
    assert all(not stage.GetPrimAtPath(p) for p in paths)
    assert all(stage.GetPrimAtPath(p) for p in sibling_paths)
    assert ctx.session.instances == {"idle": idle}
    for path in paths:
        UsdGeom.Xform.Define(stage, path).GetPrim().GetReferences().AddReference(str(ctx.asset_path))
    ctx.asset = ctx.session.register(old.path, "asset")
    assert ctx.session.instances["idle"] is idle
    ctx.world.reset()
    ctx.session.initialize()
    restore(saved)
    reload_error = difference(saved, snapshot())
    assert reload_error < 1e-6
    for a in (ctx.asset, idle):
        assert len(a.view.view.get_transforms()) == ctx.num_envs
        assert a.body_names == [BODY] and not a.joint_names
    phase[0] = 3
    results.append(segment())
    reload_repeat_error = difference(first_final, snapshot())
    assert reload_repeat_error < 1e-6
    assert stage.GetRootLayer().identifier == stage_id
    ctx.experiment_details["lifecycle"] = {"same_stage": True, "segments": results,
        "reset_state_max_scalar_error": reset_error, "reset_response_max_scalar_error": repeat_error,
        "reload_state_max_scalar_error": reload_error, "reload_response_max_scalar_error": reload_repeat_error,
        "sibling_never_removed": True, "target_references_recreated": len(paths),
        "media_scope": "all three segments, numbered phase 1 original / 2 reset / 3 reload",
        "state_writes": "only during native stopped/reset initialization; no integration pose writes",
        "benchmark_scope": "excluded from steady-state performance"}
    ctx.experiment_details["usd_reopen"] = {"rigid_bodies": 1, "colliders": 4, "joints": 0,
        "behavior": False, "unresolved_dependencies": [], "mass_kg": MASS, "friction": [.45, .35]}
    ctx.metric("reset_error", reset_error, unit="1", label="全环境复位状态最大标量差")
    ctx.metric("reload_error", reload_error, unit="1", label="全环境重载状态最大标量差")


TESTS = {"upright_push_tip": upright_push_tip, "side_roll_release": side_roll_release,
         "drop_orientations": drop_orientations, "free_eccentric": free_eccentric,
         "lifecycle_reset_reload": lifecycle_reset_reload}
