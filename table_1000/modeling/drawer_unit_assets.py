"""Desktop drawer units: a housing open at the front and drawers on SLIDER joints.

Rigid bodies ``<kind>.housing`` and ``<kind>.drawer<i>`` (1 at the top); each drawer has the parts
``front``, ``handle``, ``side1``, ``side2``, ``rear`` and ``bottom``; joints ``<kind>.drawer<i>.slide``
open toward -Y from 0 (closed) to the profile's travel. Scenes rely on these names (see
``table_1000.scenes.arrange.Drawer``). Origin at the bottom centre of the feet; +X width, +Y depth,
+Z up, the drawer fronts at -Y. Every wall is a box, so cavities stay open in collision.
"""
import math

import bpy

from table_1000.modeling.asset_builders import asset, body, box, cylinder, group_part, joint, material

PROFILES = {
    # Red powder-coated steel tool chest with chrome bar pulls (electronics bench).
    "tool_chest": {
        "width": .360, "depth": .240, "feet": .008, "wall": .006, "shelf": .004,
        "drawers": [.068, .068, .068],          # opening heights, top to bottom
        "travel": .170, "clearance": .002, "drawer_wall": .003, "front": .008,
        "housing": ("Red powder-coated steel", (.48, .025, .02), .38, 0., 0.),
        "drawer": ("Dark grey steel drawer", (.05, .055, .06), .45, 0., 0.),
        "fronts": ("Red powder-coated steel front", (.52, .03, .025), .36, 0., 0.),
        "pull": ("Chrome bar pull", (.80, .81, .82), .18, 1., 0.),
        "pull_style": "bar", "foot": ("Black rubber foot", (.02, .02, .02), .85, 0., 0.),
    },
    # Clear acrylic makeup organiser with three drawers of different depths (dressing table).
    "makeup_organizer": {
        "width": .320, "depth": .240, "feet": .004, "wall": .005, "shelf": .004,
        "drawers": [.050, .068, .090],
        "travel": .170, "clearance": .002, "drawer_wall": .003, "front": .005,
        "housing": ("Clear acrylic", (.95, .97, .98), .04, 0., .95),
        "drawer": ("Clear acrylic drawer", (.95, .97, .98), .05, 0., .92),
        "fronts": ("Frosted acrylic front", (.92, .90, .92), .25, 0., .7),
        "pull": ("Rose gold knob", (.78, .50, .42), .25, 1., 0.),
        "pull_style": "knob", "foot": ("Clear silicone foot", (.85, .86, .86), .6, 0., .5),
    },
}


def surface(spec):
    name, color, roughness, metallic, transmission = spec
    result = material(name, color, roughness, metallic=metallic, transmission=transmission)
    if transmission:
        result.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.49
    return result


def piece(owner, part, pieces):
    """Boxes ``[(name, dimensions, position, surface)]`` as part ``<owner>.<part>``."""
    visuals, colliders = [], []
    for name, size, position, look in pieces:
        before = set(owner.children)
        visuals.append(box(name, size, position, look, owner))
        colliders.extend(o for o in owner.children if o not in before and o.get("geometry_role") == "collision")
    return group_part(f"{owner.name}.{part}", owner, visuals, colliders)


def build_drawer_unit(kind):
    p = PROFILES[kind]
    w, d, f, t, s = (p[k] for k in ("width", "depth", "feet", "wall", "shelf"))
    heights, c, dt, ft = p["drawers"], p["clearance"], p["drawer_wall"], p["front"]
    height = f + 2 * t + sum(heights) + s * (len(heights) - 1)
    looks = {key: surface(p[key]) for key in ("housing", "drawer", "fronts", "pull", "foot")}

    root = asset(kind, "bottom centre of the feet", "+X width; +Y depth; +Z up", "drawer fronts at -Y")
    root["nominal_dimensions_m"] = [w, d, height]
    housing = body(f"{kind}.housing", root)
    z0 = f
    piece(housing, "bottom", [("Bottom panel", (w, d, t), (0, 0, z0 + t / 2), looks["housing"])])
    piece(housing, "top", [("Top panel", (w, d, t), (0, 0, height - t / 2), looks["housing"])])
    side_height = height - f - 2 * t
    piece(housing, "side1", [("Left side panel", (t, d, side_height), (-w / 2 + t / 2, 0, z0 + t + side_height / 2),
                              looks["housing"])])
    piece(housing, "side2", [("Right side panel", (t, d, side_height), (w / 2 - t / 2, 0, z0 + t + side_height / 2),
                              looks["housing"])])
    piece(housing, "back", [("Back panel", (w - 2 * t, t, side_height), (0, d / 2 - t / 2, z0 + t + side_height / 2),
                             looks["housing"])])
    # Openings from the top down: the bottom of opening i, and shelves between them.
    tops, z = [], height - t
    for i, h in enumerate(heights):
        tops.append(z)
        z -= h
        if i < len(heights) - 1:
            piece(housing, f"shelf{i + 1}", [(f"Shelf {i + 1}", (w - 2 * t, d - t, s), (0, -t / 2, z - s / 2),
                                               looks["housing"])])
            z -= s
    for i, (x, y) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)], 1):
        foot = cylinder(f"Foot {i}", .010, f, (x * (w / 2 - .02), y * (d / 2 - .02), f / 2), looks["foot"], housing,
                        axis="Z", segments=12)
        group_part(f"{housing.name}.foot{i}", housing, [foot],
                   [o for o in housing.children if o.name == "Collision_" + foot.name])

    inner_w = w - 2 * t - 2 * c                  # drawer box outside width
    inner_d = d - t - c                          # from the housing front to just before its back panel
    for i, (top, h) in enumerate(zip(tops, heights), 1):
        box_h = h - 2 * c
        centre = top - h / 2
        drawer = body(f"{kind}.drawer{i}", root)
        drawer.location = (0, -d / 2, centre)     # the drawer's frame: front face plane, opening centre
        front_w = w - 2 * t - 2 * c
        piece(drawer, "front", [("Drawer front", (front_w, ft, box_h), (0, ft / 2, 0), looks["fronts"])])
        depth = inner_d - ft
        y_mid = ft + depth / 2
        piece(drawer, "bottom", [("Drawer bottom", (inner_w, depth, dt), (0, y_mid, -box_h / 2 + dt / 2),
                                  looks["drawer"])])
        wall_h = box_h - dt
        for k, x in ((1, -inner_w / 2 + dt / 2), (2, inner_w / 2 - dt / 2)):
            piece(drawer, f"side{k}", [(f"Drawer side {k}", (dt, depth, wall_h), (x, y_mid, dt / 2),
                                        looks["drawer"])])
        piece(drawer, "rear", [("Drawer rear", (inner_w - 2 * dt, dt, wall_h), (0, ft + depth - dt / 2, dt / 2),
                                looks["drawer"])])
        if p["pull_style"] == "bar":
            pulls = [("Pull bar", (front_w * .6, .010, .010), (0, -.016, 0), looks["pull"]),
                     ("Pull post left", (.008, .012, .008), (-front_w * .28, -.006, 0), looks["pull"]),
                     ("Pull post right", (.008, .012, .008), (front_w * .28, -.006, 0), looks["pull"])]
            piece(drawer, "handle", pulls)
        else:
            knob = cylinder("Pull knob", .009, .014, (0, -.007, 0), looks["pull"], drawer, axis="Y", segments=16)
            group_part(f"{drawer.name}.handle", drawer, [knob],
                       [o for o in drawer.children if o.name == "Collision_" + knob.name])
        joint(f"{kind}.drawer{i}.slide", "SLIDER", housing, drawer, (0, -d / 2, centre), (0., p["travel"]),
              rotation=(0, 0, -math.pi / 2))      # slider +X = asset -Y, the drawer opens toward the front
    for obj in root.children_recursive:
        obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
