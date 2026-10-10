"""Validate asset geometry and generate previews from Blender joints and preview.json.

CLI: uv run python scripts/assets/validate_and_preview.py object.blend [--views preview.json] [--output DIR]
Requires Blender; MP4 output additionally requires ffmpeg. Source .blend is not saved.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

from table_1000.assets.references import expand_mapping


DEFAULT_VIEWS = {
    "front.jpg": {"camera": [0, -1, 0, 0, 0, 1]},
    "right.jpg": {"camera": [1, 0, 0, 0, 0, 1]},
    "top.jpg": {"camera": [0, 0, 1, 0, 1, 0]},
    "three_quarter.jpg": {"camera": [0.82, -0.82, 0.72, 0, 0, 1]},
}


def finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def camera_values(value):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError("camera must contain [x, y, z, upx, upy, upz]")
    values = [finite_number(item, "camera value") for item in value]
    direction, up = values[:3], values[3:]
    cross = (direction[1] * up[2] - direction[2] * up[1],
             direction[2] * up[0] - direction[0] * up[2],
             direction[0] * up[1] - direction[1] * up[0])
    if math.sqrt(sum(x * x for x in cross)) < 1e-8:
        raise ValueError("camera direction and up vector must not be parallel")
    return values


def pose_values(value):
    if not isinstance(value, dict):
        raise ValueError("joints must be an object")
    return {name: finite_number(q, f"joint {name}") for name, q in expand_mapping(value).items()}


def transform_values(value):
    result = {}
    for name, pose in expand_mapping(value).items():
        if len(pose) != 7:
            raise ValueError(f"transform {name} requires [x,y,z,w,qx,qy,qz]")
        pose = [finite_number(v, f"transform {name}") for v in pose]
        length = math.sqrt(sum(v * v for v in pose[3:]))
        if length == 0:
            raise ValueError(f"transform {name} has a zero quaternion")
        result[name] = pose[:3] + [v / length for v in pose[3:]]
    return result


IDENTITY_TRANSFORM = [0, 0, 0, 1, 0, 0, 0]


def interpolate_transform(a, b, t):
    qa, qb = a[3:], b[3:]
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0:
        qb, dot = [-v for v in qb], -dot
    if dot > 0.9995:
        q = [(1 - t) * x + t * y for x, y in zip(qa, qb)]
    else:
        angle = math.acos(min(1, dot))
        q = [(math.sin((1 - t) * angle) * x + math.sin(t * angle) * y) / math.sin(angle)
             for x, y in zip(qa, qb)]
    length = math.sqrt(sum(v * v for v in q))
    return [(1 - t) * x + t * y for x, y in zip(a[:3], b[:3])] + [v / length for v in q]


def read_views(path):
    entries = json.loads(path.read_text(encoding="utf-8")) if path else DEFAULT_VIEWS
    if not isinstance(entries, dict) or not entries:
        raise ValueError("views must be a nonempty JSON object")
    result = {}
    for name, entry in entries.items():
        if not isinstance(name, str) or Path(name).name != name or name in {".", ".."}:
            raise ValueError(f"invalid output name: {name}")
        if not isinstance(entry, dict):
            raise ValueError(f"{name} must be an object")
        if name.endswith(".jpg"):
            if set(entry) - {"camera", "joints", "transforms"}:
                raise ValueError(f"unsupported fields in {name}")
            result[name] = [{"frame": 0, "camera": camera_values(entry.get("camera")),
                             "joints": pose_values(entry.get("joints", {})),
                             "transforms": transform_values(entry.get("transforms", {}))}]
        elif name.endswith(".mp4"):
            if set(entry) != {"fps", "frames"}:
                raise ValueError(f"{name} requires fps and frames")
            fps = entry["fps"]
            if isinstance(fps, bool) or not isinstance(fps, int) or not 1 <= fps <= 120:
                raise ValueError(f"{name}: fps must be an integer in [1, 120]")
            frames = entry["frames"]
            if not isinstance(frames, list) or len(frames) < 2:
                raise ValueError(f"{name}: at least two keyframes are required")
            expanded = []
            camera = None
            joints = {}
            transforms = {}
            previous = -1
            for keyframe in frames:
                if not isinstance(keyframe, dict) or set(keyframe) - {"frame", "camera", "joints", "transforms"}:
                    raise ValueError(f"{name}: invalid keyframe")
                frame = keyframe.get("frame")
                if isinstance(frame, bool) or not isinstance(frame, int) or frame <= previous:
                    raise ValueError(f"{name}: frame indices must strictly increase")
                if previous == -1 and frame != 0:
                    raise ValueError(f"{name}: first frame must be 0")
                camera = camera_values(keyframe["camera"]) if "camera" in keyframe else camera
                if camera is None:
                    raise ValueError(f"{name}: first keyframe needs camera")
                joints.update(pose_values(keyframe.get("joints", {})))
                transforms.update(transform_values(keyframe.get("transforms", {})))
                expanded.append({"frame": frame, "camera": camera, "joints": joints.copy(),
                                 "transforms": transforms.copy()})
                previous = frame
            result[name] = (fps, expanded)
        else:
            raise ValueError(f"unsupported preview format: {name}")
    return result


def interpolate(keys, frame):
    before, after = next((a, b) for a, b in zip(keys, keys[1:]) if a["frame"] <= frame <= b["frame"])
    t = (frame - before["frame"]) / (after["frame"] - before["frame"])
    camera = [(1 - t) * a + t * b for a, b in zip(before["camera"], after["camera"])]
    camera_values(camera)
    names = before["joints"].keys() | after["joints"].keys()
    joints = {name: (1 - t) * before["joints"].get(name, 0.0) + t * after["joints"].get(name, 0.0)
              for name in names}
    names = before.get("transforms", {}).keys() | after.get("transforms", {}).keys()
    transforms = {name: interpolate_transform(before.get("transforms", {}).get(name, IDENTITY_TRANSFORM),
                                             after.get("transforms", {}).get(name, IDENTITY_TRANSFORM), t)
                  for name in names}
    return {"frame": frame, "camera": camera, "joints": joints, "transforms": transforms}


def configure_cycles(scene, requested):
    """Use one visible GPU; CPU remains available on machines without GPU support."""
    import bpy
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    selected = "CPU"
    if requested != "cpu":
        preferences = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
            try:
                preferences.compute_device_type = backend
                preferences.get_devices()
                devices = [device for device in preferences.devices if device.type == backend]
                if not devices:
                    continue
                chosen = devices[0]
                for device in preferences.devices:
                    device.use = device == chosen
                scene.cycles.device = "GPU"
                selected = f"{backend}: {chosen.name}"
                break
            except (TypeError, RuntimeError):
                continue
        if selected == "CPU" and requested == "gpu":
            raise RuntimeError("No supported Cycles GPU found; use --device cpu or auto")
    scene.render.use_persistent_data = True
    print("PREVIEW_DEVICE", selected, flush=True)
    return selected


def run_worker(blend, views, output, check_only=False, device="auto"):
    started = time.perf_counter()
    import bpy
    import numpy as np
    from table_1000.modeling.geometry_checks import GeometryChecks
    from mathutils import Matrix, Quaternion, Vector

    output.mkdir(parents=True, exist_ok=True)
    (output / "acceptance.json").unlink(missing_ok=True)
    bpy.ops.wm.open_mainfile(filepath=str(blend), load_ui=False)
    scene = bpy.context.scene
    scene.frame_set(0)
    if scene.rigidbody_world:
        scene.rigidbody_world.enabled = False
    bpy.context.view_layer.update()

    checks = GeometryChecks()
    meshes = checks.visuals + checks.colliders
    if not meshes:
        raise ValueError("asset contains no visible meshes")
    joints = {}
    for obj in scene.objects:
        c = obj.rigid_body_constraint
        if c and obj.get("preview_joint"):
            if obj.name in joints or c.type not in {"SLIDER", "HINGE"} or not c.object1 or not c.object2:
                raise ValueError(f"invalid preview joint: {obj.name}")
            limits = ((c.limit_lin_x_lower, c.limit_lin_x_upper) if c.type == "SLIDER"
                      else (c.limit_ang_z_lower, c.limit_ang_z_upper))
            enabled = c.use_limit_lin_x if c.type == "SLIDER" else c.use_limit_ang_z
            joints[obj.name] = (obj, c, limits if enabled else None)
    rest = {obj.name: obj.matrix_world.copy() for obj in scene.objects}

    bodies = {obj.name: obj for obj in checks.bodies}

    def pose(values, transforms=None):
        values = expand_mapping(values, joints)
        transforms = expand_mapping(transforms or {}, bodies)
        for name, (joint, c, limits) in joints.items():
            q = values.get(name, 0.0)
            if limits and not limits[0] - 1e-6 <= q <= limits[1] + 1e-6:
                raise ValueError(f"{name}={q} outside [{limits[0]}, {limits[1]}]")
        for obj in bodies.values():
            obj.matrix_world = rest[obj.name]
        bpy.context.view_layer.update()
        pending = set(joints)
        while pending:
            progressed = False
            for name in tuple(pending):
                joint, c, _ = joints[name]
                parent_joint = next((other for other in pending if joints[other][1].object2 == c.object1), None)
                if parent_joint:
                    continue
                parent_delta = c.object1.matrix_world @ rest[c.object1.name].inverted()
                joint_matrix = parent_delta @ rest[joint.name]
                q = values.get(name, 0.0)
                if c.type == "SLIDER":
                    motion = Matrix.Translation((q, 0, 0))
                else:
                    motion = Matrix.Rotation(q, 4, "Z")
                c.object2.matrix_world = (joint_matrix @ motion @ rest[joint.name].inverted()
                                          @ rest[c.object2.name])
                bpy.context.view_layer.update()
                pending.remove(name)
                progressed = True
            if not progressed:
                raise ValueError("joint dependency cycle")
        for name, transform in transforms.items():
            pivot = rest[name].translation
            rotation = Quaternion(transform[3:]).to_matrix().to_4x4()
            delta = (Matrix.Translation(Vector(transform[:3]) + pivot)
                     @ rotation @ Matrix.Translation(-pivot))
            bodies[name].matrix_world = delta @ bodies[name].matrix_world
        bpy.context.view_layer.update()
        return values, transforms

    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "acceptance.json"
    for output_name, entry in views.items():
        states = entry if output_name.endswith(".jpg") else [interpolate(entry[1], i) for i in range(entry[1][-1]["frame"]+1)]
        for state in states:
            values, transforms = pose(state["joints"], state.get("transforms"))
            checks.sample(output_name, state["frame"], values, transforms)
    checks.report["timings_seconds"] = {"validation": round(time.perf_counter() - started, 3)}
    checks.report["status"] = "failed" if checks.report["failures"] else "passed"
    report_path.write_text(json.dumps(checks.report, indent=2)+"\n", encoding="utf-8")
    if checks.report["failures"]:
        raise ValueError(f"Collision coarse check failed; see {report_path}")
    if check_only:
        print("GEOMETRY_CHECK_PASSED", report_path, flush=True)
        return
    pose({})

    def corners():
        depsgraph = bpy.context.evaluated_depsgraph_get()
        return [obj.evaluated_get(depsgraph).matrix_world @ Vector(corner)
                for obj in meshes for corner in obj.evaluated_get(depsgraph).bound_box]

    def box(points):
        minimum = Vector(tuple(min(point[i] for point in points) for i in range(3)))
        maximum = Vector(tuple(max(point[i] for point in points) for i in range(3)))
        return minimum, maximum

    bpy.context.view_layer.update()
    low, high = box(corners())
    default_center = (low + high) / 2
    default_span = max(high - low)
    for obj in list(scene.objects):
        if obj.type in {"LIGHT", "CAMERA"}:
            bpy.data.objects.remove(obj, do_unlink=True)
    selected_device = configure_cycles(scene, device)
    scene.cycles.samples = 16
    scene.cycles.use_denoising = True
    scene.cycles.denoising_use_gpu = scene.cycles.device == "GPU"
    if selected_device.startswith("OPTIX:"):
        scene.cycles.denoiser = "OPTIX"
    scene.cycles.transmission_bounces = 12
    scene.render.threads_mode = "FIXED"
    scene.render.threads = 8
    scene.render.resolution_x = scene.render.resolution_y = 480
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "BMP"
    scene.render.image_settings.color_mode = "RGB"
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.world = bpy.data.worlds.new("Tutorial studio")
    scene.world.use_nodes = True
    background = scene.world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = (0.35, 0.35, 0.35, 1)
    background.inputs["Strength"].default_value = 0.45
    camera_data = bpy.data.cameras.new("Tutorial camera")
    camera_data.type = "ORTHO"
    camera = bpy.data.objects.new("Tutorial camera", camera_data)
    scene.collection.objects.link(camera)
    scene.camera = camera
    for name, offset, power in [("Key", (-1, -1.5, 2), 80), ("Fill", (1.5, -0.5, 1), 35),
                                ("Rim", (0, 1.5, 1.5), 60), ("Lower fill", (0, -0.5, -1.5), 45)]:
        data = bpy.data.lights.new(name, "AREA")
        data.energy = power * default_span * default_span
        data.shape = "DISK"
        data.size = default_span * 1.5
        light = bpy.data.objects.new(name, data)
        scene.collection.objects.link(light)
        light.location = default_center + Vector(offset) * default_span
        light.rotation_euler = (-Vector(offset)).to_track_quat("-Z", "Y").to_euler()

    def camera_rotation(values):
        direction = Vector(values[:3]).normalized()
        right = (-direction).cross(Vector(values[3:])).normalized()
        up = direction.cross(right).normalized()
        return Matrix((right, up, direction)).transposed()

    def frame_camera(values, center, scale, span):
        rotation = camera_rotation(values)
        direction = Vector(values[:3]).normalized()
        camera.matrix_world = Matrix.Translation(center + direction * span * 3) @ rotation.to_4x4()
        camera.data.ortho_scale = scale
        camera.data.clip_start = max(span * 0.001, 1e-5)
        camera.data.clip_end = max(span * 20, 1)

    palette = [(0.28,0.48,0.72,1),(0.95,0.42,0.16,1),(0.24,0.68,0.42,1),(0.65,0.35,0.76,1)]
    materials = {}
    for i, body in enumerate(checks.bodies):
        material = bpy.data.materials.new("Collision_"+body.name)
        material.use_nodes = True
        nodes = material.node_tree.nodes
        shader = nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = palette[i % len(palette)]
        shader.inputs["Roughness"].default_value = 0.65
        materials[body.name] = material
    for obj in checks.colliders:
        obj.data.materials.clear()
        obj.data.materials.append(materials[checks.owner[obj.name]])
        for face in obj.data.polygons:
            face.use_smooth = False

    def render_pair(destination):
        arrays = []
        with tempfile.TemporaryDirectory(prefix="panels-", dir=output) as folder:
            for collision in (False, True):
                for obj in checks.visuals:
                    obj.hide_render = collision
                for obj in checks.colliders:
                    obj.hide_render = not collision
                scene.render.filepath = str(Path(folder) / "panel.bmp")
                bpy.ops.render.render(write_still=True)
                panel = bpy.data.images.load(scene.render.filepath, check_existing=False)
                width,height = panel.size
                pixels = np.empty(width*height*4, dtype=np.float32)
                panel.pixels.foreach_get(pixels)
                arrays.append(pixels.reshape(height,width,4))
                bpy.data.images.remove(panel)
            combined = bpy.data.images.new("Visual | Collision", width=width*2, height=height, alpha=True)
            combined.pixels.foreach_set(np.concatenate(arrays, axis=1).ravel())
            combined.filepath_raw = str(destination)
            combined.file_format = "JPEG" if destination.suffix == ".jpg" else "BMP"
            combined.save()
            bpy.data.images.remove(combined)

    for name, entry in views.items():
        if name.endswith(".jpg"):
            state = entry[0]
            pose(state["joints"], state.get("transforms"))
            points = corners()
            low, high = box(points)
            center = (low + high) / 2
            span = max(high - low)
            rotation = camera_rotation(state["camera"])
            projected = [rotation.transposed() @ (point - center) for point in points]
            a, b = box(projected)
            frame_camera(state["camera"], center, max(b.x - a.x, b.y - a.y) * 1.28, span)
            render_pair(output / name)
            print("PREVIEW_RENDERED", output / name, flush=True)
        else:
            fps, keys = entry
            frames = [interpolate(keys, index) for index in range(keys[-1]["frame"] + 1)]
            all_points = []
            for state in frames:
                pose(state["joints"], state.get("transforms"))
                all_points.extend(corners())
            low, high = box(all_points)
            center = (low + high) / 2
            span = max(high - low)
            scale = 0.0
            for state in frames:
                pose(state["joints"], state.get("transforms"))
                rotation = camera_rotation(state["camera"])
                projected = [rotation.transposed() @ (point - center) for point in corners()]
                a, b = box(projected)
                scale = max(scale, 2*max(abs(a.x), abs(b.x), abs(a.y), abs(b.y)))
            if not shutil.which("ffmpeg"):
                raise FileNotFoundError("ffmpeg is required for MP4 previews")
            with tempfile.TemporaryDirectory(prefix="preview-frames-", dir=output) as temp:
                temporary = Path(temp)
                for state in frames:
                    pose(state["joints"], state.get("transforms"))
                    frame_camera(state["camera"], center, scale * 1.28, span)
                    render_pair(temporary / f"{state['frame']:06d}.bmp")
                subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(fps),
                                "-i", str(temporary / "%06d.bmp"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                                "-crf", "18", "-movflags", "+faststart", str(output / name)], check=True)
            print("PREVIEW_RENDERED", output / name, flush=True)

    checks.report["preview"] = {"status": "completed", "device": selected_device, "samples": 16,
                                "panel_size": 480}
    checks.report["timings_seconds"]["total"] = round(time.perf_counter() - started, 3)
    report_path.write_text(json.dumps(checks.report, indent=2)+"\n", encoding="utf-8")

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("object", type=Path)
    parser.add_argument("--views", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", choices=("auto", "cpu", "gpu"), default="auto",
                        help="Cycles device: auto selects one visible GPU, otherwise CPU")
    parser.add_argument("--check-only", action="store_true", help="Validate geometry and sampled poses without rendering")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)
    blend = args.object.resolve()
    views_path = args.views.resolve() if args.views else None
    output = (args.output or blend.parent / "preview").resolve()
    # A failed rerun must not leave a previous successful report behind.
    (output / "acceptance.json").unlink(missing_ok=True)
    views = read_views(views_path)
    if args.worker:
        run_worker(blend, views, output, args.check_only, args.device)
    else:
        subprocess.run(["blender", "-b", "--python-exit-code", "1", "-P", str(Path(__file__).resolve()),
                        "--", str(blend), "--output", str(output), "--worker", "--device", args.device,
                        *(["--views", str(views_path)] if views_path else []),
                        *(["--check-only"] if args.check_only else [])], check=True)


if __name__ == "__main__":
    # Executed directly by Blender; only the package location is shared with
    # the host, not the host Python environment or its dependencies.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    main()
