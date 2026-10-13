"""Parameterized binder clips with straight planar loops and rigid-jaw hinges.

Sheet/loop proportions and colors retain the existing scene-002 clip geometry.
The articulated model approximates sheet bending with hinges; it does not
simulate spring elasticity, clamping force or automatic return.
"""
from dataclasses import dataclass, replace
import math

import bpy
from mathutils import Vector
from table_1000.modeling.asset_builders import (
    asset, body, box, collider, cylinder, generate, group_part, joint, material, mesh,
)


COLORS = {
    "blue": (0.045, 0.26, 0.48),
    "gold": (0.58, 0.43, 0.13),
    "pink": (0.58, 0.23, 0.34),
    "pale_yellow": (0.73, 0.69, 0.36),
}


@dataclass(frozen=True)
class ClipProfile:
    """Metre dimensions, independent sheet/wire gauges and handle centerline."""

    width: float
    back_half_depth: float
    sheet_thickness: float
    jaw_height: float
    jaw_top_outer: float
    jaw_top_inner: float
    lip_radius: float
    wire_radius: float
    handle_points: tuple


PROFILES = {
    # Preserve every default point and gauge of the original 22 mm assets.
    "small": ClipProfile(
        0.022, 0.00635, 0.0007, 0.0155, 0.0007, 0.00005, 0.00075, 0.00065,
        ((-0.0075, 0.001, 0.0155), (-0.0085, 0.0105, 0.022),
         (-0.004, 0.01235, 0.02635), (0.004, 0.01235, 0.02635),
         (0.0085, 0.0105, 0.022), (0.0075, 0.001, 0.0155)),
    ),
    "large": ClipProfile(
        0.041, 0.012, 0.00085, 0.030, 0.00095, 0.00010, 0.0010, 0.0009,
        ((-0.0144, 0.0015, 0.030), (-0.016, 0.020, 0.043),
         (-0.0075, 0.0231, 0.0511), (0.0075, 0.0231, 0.0511),
         (0.016, 0.020, 0.043), (0.0144, 0.0015, 0.030)),
    ),
    "mini": ClipProfile(
        0.015, 0.0044, 0.0005, 0.0105, 0.0005, 0.00005, 0.00055, 0.00045,
        ((-0.0048, 0.0007, 0.0105), (-0.0055, 0.0067, 0.0163),
         (-0.0024, 0.00855, 0.01955), (0.0024, 0.00855, 0.01955),
         (0.0055, 0.0067, 0.0163), (0.0048, 0.0007, 0.0105)),
    ),
}


def wire_segment(name, start, end, surface, parent, radius=0.00065, segments=10):
    start, end = Vector(start), Vector(end)
    direction = end - start
    obj = cylinder(name, radius, direction.length, (start + end) / 2,
                   surface, parent, axis="Z", segments=segments, collision=False)
    rotation = direction.to_track_quat("Z", "Y").to_matrix()
    # Bake rotation so both visual and convex proxy have unit object scale.
    for vertex in obj.data.vertices:
        vertex.co = rotation @ vertex.co
    collider(obj)


def build_storage(color, size="small"):
    profile = PROFILES[size]
    paint = material(f"{color.title()} enamel steel", COLORS[color], 0.28, 0.65)
    steel = material("Plated steel wire", (0.66, 0.70, 0.73), 0.2, 0.98)
    root = asset("binder_clip", "center of flat spring back at its underside",
                 "+X jaw width; +Y rear handle; +Z toward closed lips and extended handles",
                 "front handle at -Y")
    whole = body("binder_clip.clip", root)
    whole["interaction_scope"] = "closed storage; no jaw opening or handle articulation"
    # Three gauges are specified explicitly, rather than uniformly scaling steel.
    box("Spring back", (profile.width, 2 * profile.back_half_depth, profile.sheet_thickness),
        (0, 0, profile.sheet_thickness / 2), paint, whole)
    for sign, side in ((-1, "front"), (1, "rear")):
        section = [(sign * profile.back_half_depth, profile.sheet_thickness),
                   (sign * profile.jaw_top_outer, profile.jaw_height),
                   (sign * profile.jaw_top_inner, profile.jaw_height),
                   (sign * (profile.back_half_depth - profile.sheet_thickness), profile.sheet_thickness)]
        vertices = [(x, y, z) for x in (-profile.width / 2, profile.width / 2) for y, z in section]
        faces = [(3, 2, 1, 0), (4, 5, 6, 7)]
        faces += [(i, (i+1)%4, (i+1)%4+4, i+4) for i in range(4)]
        sheet = mesh(f"{side} folded jaw", vertices, faces, whole, paint)
        collider(sheet)
        # Closed contacting lips and sockets connect the two ends of each loop.
        cylinder(f"{side} rolled lip", profile.lip_radius, profile.width,
                 (0, sign * profile.lip_radius, profile.jaw_height), paint, whole,
                 axis="X", segments=12)
        points = [(x, sign * y, z) for x, y, z in profile.handle_points]
        for index, (start, end) in enumerate(zip(points, points[1:])):
            wire_segment(f"{side} wire {index}", start, end, steel, whole, profile.wire_radius)
    components = {
        "back_plate": ["Spring back"],
        "front_jaw": ["front folded jaw", "front rolled lip"],
        "rear_jaw": ["rear folded jaw", "rear rolled lip"],
        "front_loop": [f"front wire {i}" for i in range(5)],
        "rear_loop": [f"rear wire {i}" for i in range(5)],
    }
    for name, names in components.items():
        visuals = [bpy.data.objects[name] for name in names]
        proxies = [bpy.data.objects["Collision_" + name] for name in names]
        component = group_part(whole.name + "." + name, whole, visuals, proxies)
        join_visuals(component)
    lock_geometry(root)


def join_visuals(component):
    """Keep one selectable surface per semantic component, not per wire segment."""
    visuals = [obj for obj in component.children if obj.get("geometry_role") == "visual"]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in visuals:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = visuals[0]
    if len(visuals) > 1:
        bpy.ops.object.join()
    bpy.context.object.name = component.name + ".visual"


def lock_geometry(root):
    for obj in root.children_recursive:
        if obj.get("semantic_part") or obj.get("geometry_role"):
            obj.lock_location = obj.lock_rotation = obj.lock_scale = (True, True, True)
        if obj.get("geometry_role") == "collision":
            obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    root.select_set(True)
    bpy.context.view_layer.objects.active = root


JAW_LIMIT = math.radians(15)
# Negative turns a handle outward for the independent hinge demonstration.
# During jaw opening both handle coordinates stay at zero: no extra folding.
HANDLE_LIMITS = (math.radians(-30), 0.0)

# A straight planar loop needs the shell to clear its plane at the B opening
# endpoint. Each approved size uses its own back depth and wire/socket gauges;
# jaw widths and the existing color variants remain unchanged.
MECHANISM_PROFILES = {
    "small": replace(PROFILES["small"], back_half_depth=0.0044),
    "large": replace(PROFILES["large"], back_half_depth=0.0084),
    "mini": replace(PROFILES["mini"], back_half_depth=0.00305),
}
MECHANISM_SEAMS = {"small": 0.0003, "large": 0.0004, "mini": 0.00022}
MECHANISM_HANDLE_AXIS_Y = {"small": 0.0015, "large": 0.002, "mini": 0.0011}
# Each gauge, socket and rear bend is specified per size, not scaled uniformly.
MECHANISM_LOOP_SHAPES = {
    "small": (0.0075, 0.0085, 0.004, 0.0075, 0.010),
    "large": (0.0144, 0.016, 0.0075, 0.015, 0.0195),
    "mini": (0.0048, 0.0055, 0.0024, 0.0052, 0.007),
}


def mechanism_handle_points(size):
    profile = MECHANISM_PROFILES[size]
    tip, outer, bridge, shoulder_z, tail_z = MECHANISM_LOOP_SHAPES[size]
    return tuple(
        (x, MECHANISM_HANDLE_AXIS_Y[size] + (profile.jaw_height - z) * math.tan(JAW_LIMIT), z)
        for x, z in ((-tip, profile.jaw_height), (-outer, -shoulder_z),
                     (-bridge, -tail_z), (bridge, -tail_z),
                     (outer, -shoulder_z), (tip, profile.jaw_height))
    )


MECHANISM_HANDLE_POINTS = {size: mechanism_handle_points(size) for size in MECHANISM_PROFILES}


def _component(owner, name, builder):
    previous = set(owner.children)
    builder()
    created = set(owner.children) - previous
    visuals = sorted((obj for obj in created if obj.get("geometry_role") == "visual"), key=lambda obj: obj.name)
    proxies = sorted((obj for obj in created if obj.get("geometry_role") == "collision"), key=lambda obj: obj.name)
    result = group_part(owner.name + "." + name, owner, visuals, proxies)
    join_visuals(result)
    return result


def build_articulated(color, size="small"):
    """A free back plate, two rigid jaws and two intact straight planar loops.

    This tree approximates spring-sheet deformation with ordinary hinges. Handle
    and jaw coordinates remain independent; preview keyframes coordinate them.
    The two jaw/loop pairs ignore contact only for their simplified seated pivots;
    the per-asset audit also inspects those pairs beyond the local pivot region.
    """
    profile = MECHANISM_PROFILES[size]
    lever_points = MECHANISM_HANDLE_POINTS[size]
    paint = material(f"{color.title()} enamel steel", COLORS[color], 0.28, 0.65)
    steel = material("Plated steel wire", (0.66, 0.70, 0.73), 0.2, 0.98)
    root = asset("binder_clip", "center of flat back plate at its underside",
                 "+X jaw width and front positive hinge axis; +Y rear jaw; +Z toward mouth; handles extend toward -Z beyond back",
                 "front jaw at -Y")
    root["mechanism_scope"] = "rigid-sheet approximation; no elasticity, clamping force or automatic return"
    base = body("binder_clip.back", root)
    base["free_assembly_base"] = True
    _component(base, "plate", lambda: box(
        "back_plate", (profile.width, 2 * profile.back_half_depth, profile.sheet_thickness),
        (0, 0, profile.sheet_thickness / 2), paint, base))
    # A small clearance seam keeps both jaw hinges free of the back proxy.
    seam = MECHANISM_SEAMS[size]
    bottom = profile.sheet_thickness + seam
    for sign, side in ((-1, "front"), (1, "rear")):
        jaw = body(f"binder_clip.{side}_jaw", root)
        pivot = (0, sign * profile.back_half_depth, bottom)
        section = [(sign * profile.back_half_depth, bottom),
                   (sign * profile.jaw_top_outer, profile.jaw_height),
                   (sign * profile.jaw_top_inner, profile.jaw_height),
                   (sign * (profile.back_half_depth - profile.sheet_thickness), bottom)]
        vertices = [(x, y, z) for x in (-profile.width / 2, profile.width / 2) for y, z in section]
        faces = [(3, 2, 1, 0), (4, 5, 6, 7)]
        faces.extend((i, (i + 1) % 4, (i + 1) % 4 + 4, i + 4) for i in range(4))

        def make_sheet():
            surface = mesh(side + "_sheet", vertices, faces, jaw, paint)
            collider(surface)
        _component(jaw, "sheet", make_sheet)
        lip_center = (0, sign * profile.lip_radius, profile.jaw_height)
        handle_anchor = (0, sign * MECHANISM_HANDLE_AXIS_Y[size], profile.jaw_height)
        _component(jaw, "rolled_contact", lambda: cylinder(
            side + "_lip", profile.lip_radius, profile.width, lip_center,
            paint, jaw, axis="X", segments=8))
        loop = body(f"binder_clip.{side}_handle", root)
        points = [(x, sign * y, z) for x, y, z in lever_points]

        def make_loop():
            for index, (start, end) in enumerate(zip(points, points[1:]), 1):
                wire_segment(f"{side}_wire{index}", start, end, steel, loop,
                             profile.wire_radius, segments=6)
        _component(loop, "wire_loop", make_loop)
        # The constraint local Z is +X for front and -X for rear. Equal positive
        # jaw coordinates thus open both sheets symmetrically, without a cycle.
        rotation = (0, -sign * math.pi / 2, 0)
        jaw_joint = joint(f"binder_clip.{side}_jaw_open", "HINGE", base, jaw, pivot,
                          (0, JAW_LIMIT), rotation)
        handle_joint = joint(f"binder_clip.{side}_handle_turn", "HINGE", jaw, loop,
                             handle_anchor, HANDLE_LIMITS, rotation)
        handle_joint.rigid_body_constraint.disable_collisions = True
        handle_joint["collision_exclusion_reason"] = (
            "Wire tips seat inside a simplified solid rolled lip; only the connected jaw/loop pair. "
            "Trial checks must separately verify no interference beyond the pivot neighborhood.")
        # The organization root must carry the anchors as well as all bodies.
        for constraint in (jaw_joint, handle_joint):
            bpy.context.view_layer.update()
            world = constraint.matrix_world.copy()
            constraint.parent = root
            constraint.matrix_world = world
    lock_geometry(root)


def generate_clip(color, source_file, argv=None, *, size="small", articulated=True):
    builder = build_articulated if articulated else build_storage
    generate(lambda: builder(color, size), source_file, argv)
