"""Shared modeling helpers for complete, non-articulated small props.

Dimensions below are approved engineering estimates. Unseen ports, branding,
extra buttons and unrequested moving mechanisms are deliberately omitted.
"""

import math

import bpy
import bmesh

from table_1000.modeling.asset_builders import (
    asset, body, box, collider, cylinder, generate, material, mesh, part, tube,
)


def rounded_box(name, dimensions, position, surface, parent, radius, segments=2):
    """Apply rounded visual edges before copying the matching convex proxy."""
    obj = box(name, dimensions, position, surface, parent, collision=False)
    bpy.context.view_layer.objects.active = obj
    bevel = obj.modifiers.new("Rounded solid edges", "BEVEL")
    bevel.width = radius
    bevel.segments = segments
    bpy.ops.object.modifier_apply(modifier=bevel.name)
    proxy = collider(obj)
    # Bevel corner patches can be nonplanar; the detector requires an explicit
    # closed convex surface. Build that surface from the same rounded vertices.
    hull = bmesh.new()
    for vertex in obj.data.vertices:
        hull.verts.new(vertex.co)
    result = bmesh.ops.convex_hull(hull, input=list(hull.verts))
    bmesh.ops.delete(hull, geom=result["geom_interior"] + result["geom_unused"],
                     context="VERTS")
    hull.to_mesh(proxy.data)
    hull.free()
    return obj


def profiled_solid(name, rings, position, surface, parent, segments=12):
    """Build an X-axis solid and convex proxies for its concave profile runs.

    Decreasing profile slopes describe a convex run. Split where the slope
    increases, preserving grip recesses without a collider for every ring span.
    """
    vertices = [(x, r * math.cos(i * math.tau / segments),
                 r * math.sin(i * math.tau / segments))
                for x, r in rings for i in range(segments)]
    faces = [tuple(range(segments - 1, -1, -1)),
             tuple((len(rings) - 1) * segments + i for i in range(segments))]
    for k in range(len(rings) - 1):
        a, b = k * segments, (k + 1) * segments
        faces.extend((a + i, a + (i + 1) % segments,
                      b + (i + 1) % segments, b + i) for i in range(segments))
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    slopes = [(r1-r0)/(end-start) for (start,r0),(end,r1) in zip(rings,rings[1:])]
    breaks = [0] + [i for i in range(1, len(slopes)) if slopes[i] > slopes[i-1]+1e-9]
    breaks.append(len(rings)-1)
    for k, (first, last) in enumerate(zip(breaks, breaks[1:])):
        count = last-first+1
        proxy_faces = [tuple(range(segments-1,-1,-1)),
                       tuple((count-1)*segments+i for i in range(segments))]
        for j in range(count-1):
            a, b = j*segments, (j+1)*segments
            proxy_faces.extend((a+i, a+(i+1)%segments, b+(i+1)%segments, b+i)
                               for i in range(segments))
        proxy = mesh(f"{name} convex run {k}",
                     vertices[first*segments:(last+1)*segments], proxy_faces,
                     parent, shape="CONVEX_HULL")
        proxy.location = position
    return obj


def ellipsoid(name, dimensions, position, surface, parent, collision=True):
    """Low-segment closed ellipsoid and its matching convex proxy."""
    bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=6, radius=1,
                                        location=position)
    obj = bpy.context.object
    obj.name = name
    obj.scale = tuple(d / 2 for d in dimensions)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.parent = parent
    obj["geometry_role"] = "visual"
    obj.data.materials.append(surface)
    for face in obj.data.polygons:
        face.use_smooth = True
    if collision:
        collider(obj)
    return obj


def component(whole, name):
    """A material or functional part belonging to the complete rigid object."""
    return part(whole.name + "." + name, whole)


def screwdriver(whole):
    yellow = material("Yellow molded grip", (0.98, 0.55, 0.012), 0.30)
    black = material("Black neck", (0.015, 0.020, 0.025), 0.30)
    steel = material("Silver shaft", (0.58, 0.62, 0.66), 0.22, metallic=0.90)
    grip = component(whole, "grip")
    neck = component(whole, "neck")
    shaft = component(whole, "shaft")
    z = 0.011
    profiled_solid("Yellow grip", [(-0.075, 0.0015), (-0.074, 0.004),
                   (-0.070, 0.0085), (-0.064, 0.0095), (0.002, 0.0095),
                   (0.010, 0.0085), (0.012, 0.009), (0.015, 0.011),
                   (0.019, 0.010), (0.020, 0.007)], (0, 0, z), yellow, grip)
    # Long shallow molded ribs stay within the approved 22 mm grip envelope.
    for i in range(6):
        angle = i * math.tau / 6
        cylinder(f"Grip rib {i}", 0.0015, 0.062,
                 (-0.030, 0.0089 * math.cos(angle), z + 0.0089 * math.sin(angle)),
                 yellow, grip, segments=8)
    profiled_solid("Black neck", [(0.018, 0.0065), (0.027, 0.0065),
                   (0.030, 0.004)], (0, 0, z), black, neck)
    profiled_solid("Silver shaft and narrow tip", [(0.027, 0.0022), (0.070, 0.0022),
                   (0.075, 0.0009)], (0, 0, z), steel, shaft)


def power_plug(whole):
    white = material("White plug shell", (0.82, 0.84, 0.85), 0.26)
    metal = material("Flat pin metal", (0.57, 0.60, 0.63), 0.24, metallic=0.88)
    shell = component(whole, "shell")
    rounded_box("Rounded white shell", (0.055, 0.055, 0.030),
                (0, 0, 0.015), white, shell, radius=0.006)
    # Looking toward the pin face (+Z view, +Y at the top), the top blade is
    # vertical; the lower left slopes \ and the lower right slopes /.
    for name, x, y, angle in (("top_pin", 0, 0.009, math.pi / 2),
                              ("left_pin", -0.010, -0.008, -math.pi / 3),
                              ("right_pin", 0.010, -0.008, math.pi / 3)):
        pin = component(whole, name)
        obj = rounded_box(name, (0.0065, 0.0016, 0.018), (x, y, 0.039),
                          metal, pin, radius=0.0002, segments=1)
        obj.rotation_euler.z = angle
        bpy.data.objects["Collision_" + obj.name].rotation_euler.z = angle


def distractor_food(whole):
    rice = material("Ivory rice prop", (0.89, 0.85, 0.74), 0.65)
    orange = material("Orange shrimp prop", (0.91, 0.26, 0.09), 0.42)
    pale = material("Pale shrimp bands", (0.96, 0.76, 0.56), 0.45)
    red = material("Red shrimp tail", (0.55, 0.035, 0.065), 0.42)
    rice_part = component(whole, "rice")
    shrimp = component(whole, "shrimp")
    bands = component(whole, "pale_bands")
    tail = component(whole, "tail")
    rounded_box("Rice base", (0.045, 0.025, 0.012), (-0.0075, 0, 0.006),
                rice, rice_part, radius=0.005)
    # Overlapping lobes reproduce visible transverse bands as one rigid prop.
    for i, (x, width, z) in enumerate(((-0.024, 0.022, 0.016),
                                     (-0.015, 0.021, 0.017),
                                     (-0.006, 0.019, 0.017),
                                     (0.003, 0.0165, 0.016),
                                     (0.011, 0.013, 0.0145))):
        ellipsoid(f"Shrimp segment {i}", (0.012 if i == 0 else 0.013, width, 0.010),
                  (x, 0, z), orange, shrimp)
        if i < 4:
            ellipsoid(f"Pale band {i}", (0.0028, width + 0.0003, 0.0101),
                      (x + 0.005, 0, z - 0.0001), pale, bands, collision=False)
            # These thin colored bands describe the same rigid prop surface.
            # Their <=0.15 mm overhang is within the 0.2 mm shape tolerance;
            # the shrimp lobes supply contact rather than duplicating proxies.
    # Two closed triangular tail lobes; each receives a separate convex proxy.
    for sign in (-1, 1):
        triangle = [(0.015, 0), (0.030, sign * 0.002), (0.026, sign * 0.010)]
        vertices = [(x, y, z) for z in (0.012, 0.016) for x, y in triangle]
        faces = [(2, 1, 0), (3, 4, 5), (0, 1, 4, 3), (1, 2, 5, 4), (2, 0, 3, 5)]
        visual = mesh(f"Tail lobe {sign}", vertices, faces, tail, red)
        collider(visual)


def remote_control(whole):
    white = material("White remote shell", (0.86, 0.87, 0.86), 0.28)
    panel = material("Dark face area", (0.065, 0.075, 0.085), 0.32)
    black = material("Round black buttons", (0.008, 0.012, 0.016), 0.27)
    steel = material("Fixed end loop", (0.55, 0.58, 0.60), 0.22, metallic=0.88)
    shell = component(whole, "shell")
    face = component(whole, "face_panel")
    indicator = component(whole, "indicator")
    loop = component(whole, "end_loop")
    # 90 mm shell plus the fixed loop gives a 95 mm overall length.
    rounded_box("Rounded white body", (0.090, 0.035, 0.0135),
                (-0.0025, 0, 0.00675), white, shell, radius=0.0045)
    rounded_box("Plain dark face", (0.069, 0.025, 0.0008),
                (-0.004, 0, 0.0137), panel, face, radius=0.0003)
    for i, x in enumerate((0.010, 0.023)):
        button = component(whole, f"button{i+1}")
        cylinder(f"Round button {i}", 0.0043, 0.0009, (x, 0, 0.01455),
                 black, button, axis="Z", segments=12)
    ellipsoid("Small indicator", (0.003, 0.0018, 0.0004),
              (0.034, 0, 0.0135), black, indicator)
    # Ring sectors retain the visible hole; no solid global hull fills it.
    tube("Fixed metal end loop", 0.003, 0.0017, 0.0018,
         (0.0445, -0.010, 0.006), steel, loop, axis="Z", segments=8)


def usb_drive(whole):
    black = material("Black USB body", (0.012, 0.016, 0.022), 0.26)
    metal = material("Silver exterior connector", (0.58, 0.62, 0.66), 0.26,
                     metallic=0.88)
    housing = component(whole, "housing")
    connector = component(whole, "connector")
    rounded_box("Black rounded body", (0.044, 0.020, 0.010),
                (-0.008, 0, 0.005), black, housing, radius=0.003)
    rounded_box("Fixed exterior connector", (0.016, 0.012, 0.0045),
                (0.022, 0, 0.005), metal, connector, radius=0.00035, segments=1)


def build_small_prop(category):
    builders = {"screwdriver": screwdriver, "power_plug": power_plug,
                "distractor_food": distractor_food, "remote_control": remote_control,
                "usb_drive": usb_drive}
    axes = "+X length; +Y width; +Z up"
    if category == "power_plug":
        axes = "+X width; +Y depth; +Z from shell toward pins"
    root = asset(category, "center of overall XY envelope at bottom plane",
                 axes, "-Y is the preview front; remote face is +Z")
    whole = body(category + ".body", root)
    builders[category](whole)
    finish_whole(whole)


def finish_whole(whole):
    """Keep semantic parts; select and edit the complete Compound root."""
    bpy.context.view_layer.update()
    for node in whole.children:
        visuals = [o for o in node.children if o.get("geometry_role") == "visual"]
        bpy.ops.object.select_all(action="DESELECT")
        for obj in visuals:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = visuals[0]
        if len(visuals) > 1:
            bpy.ops.object.join()
        bpy.context.object.name = node.name + ".visual"
        proxies = sorted((o for o in node.children if o.get("geometry_role") == "collision"),
                         key=lambda o: o.name)
        for i, obj in enumerate(proxies, 1):
            obj.name = node.name + ".collision" + (str(i) if len(proxies) > 1 else "")
    for obj in whole.children_recursive:
        obj.hide_select = True
        obj.lock_location = obj.lock_rotation = obj.lock_scale = (True, True, True)
    whole.parent.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    whole.select_set(True)
    bpy.context.view_layer.objects.active = whole


def generate_small_prop(build, source_file, argv=None):
    """Generate using the same-version package, with no copied shared modules."""
    generate(build, source_file, argv)
