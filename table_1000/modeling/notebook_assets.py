"""Closed notebooks: one compound body, with fixed covers and accessories.

Photo-guided dimensions are estimates. Covers, page-block footprint, spine,
strap position and colors reuse the existing scene-two parametric geometry.
The closed stack removes the old cloth snapshot gaps; the strap is a complete
fixed loop. This chapter-one model has no opening or page-turning mechanism.
"""

import math

import bpy

from table_1000.modeling.asset_builders import (
    asset, body, box, collider, generate, group_part, material, mesh,
)


PROFILES = {
    "black": ((0.009, 0.009, 0.010), (0.028, 0.030, 0.035)),
    "navy": ((0.007, 0.012, 0.045), (0.019, 0.028, 0.058)),
}

# Body dimensions are engineering estimates. Accessories extend beyond them.
# Cover, page inset, loop and ribbon dimensions are set separately below.
BOOK_SIZES = {
    "large": (0.210, 0.297, 0.012),
    "medium": (0.148, 0.210, 0.012),
}
BOOK_COLORS = {
    "charcoal": ((0.022, 0.026, 0.029), "ribbon", (0.012, 0.014, 0.015)),
    "taupe": ((0.18, 0.14, 0.10), "ribbon", (0.68, 0.66, 0.56)),
    "slate": ((0.035, 0.060, 0.082), "loop", (0.034, 0.055, 0.075)),
    "tan": ((0.28, 0.20, 0.12), "ribbon", (0.68, 0.66, 0.56)),
    "rust": ((0.24, 0.055, 0.018), "loop", (0.21, 0.052, 0.022)),
}


def mottled_cover(color):
    surface = material("Mottled cover", color, 0.78)
    nodes, links = surface.node_tree.nodes, surface.node_tree.links
    coordinates = nodes.new("ShaderNodeTexCoord")
    patches = nodes.new("ShaderNodeTexNoise")
    patches.inputs["Scale"].default_value = 75
    patches.inputs["Detail"].default_value = 2
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.2
    ramp.color_ramp.elements[0].color = (*(v * 0.75 for v in color), 1)
    ramp.color_ramp.elements[1].position = 0.8
    ramp.color_ramp.elements[1].color = (*(v * 1.2 for v in color), 1)
    shader = nodes.get("Principled BSDF")
    links.new(coordinates.outputs["Object"], patches.inputs["Vector"])
    links.new(patches.outputs["Fac"], ramp.inputs[0])
    links.new(ramp.outputs["Color"], shader.inputs["Base Color"])
    grain = nodes.new("ShaderNodeTexNoise")
    grain.inputs["Scale"].default_value = 1600
    grain.inputs["Detail"].default_value = 1
    bump = nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.12
    bump.inputs["Distance"].default_value = 0.000025
    links.new(coordinates.outputs["Object"], grain.inputs["Vector"])
    links.new(grain.outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    return surface


def rounded_plate(name, dimensions, position, radius, surface, parent):
    """Low-sided convex rounded rectangle, with identical contact geometry."""
    width, length, thickness = dimensions
    outline = []
    for cx, cy, start in ((width / 2 - radius, length / 2 - radius, 0),
                          (-width / 2 + radius, length / 2 - radius, 90),
                          (-width / 2 + radius, -length / 2 + radius, 180),
                          (width / 2 - radius, -length / 2 + radius, 270)):
        for step in range(3):
            angle = math.radians(start + 45 * step)
            outline.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    n = len(outline)
    vertices = [(x, y, z) for z in (-thickness / 2, thickness / 2) for x, y in outline]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)]
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    collider(obj)
    return obj


def fixed_part(owner, name, visuals):
    """One semantic component; its contact meshes still belong to the book."""
    collisions = [bpy.data.objects["Collision_" + obj.name] for obj in visuals
                  if "Collision_" + obj.name in bpy.data.objects]
    return group_part(owner.name + "." + name, owner, visuals, collisions)


def page_edge_detail(owner, x, length, first_z, count, surface):
    """Visible fore-edge strokes, without tiny box backs or contact proxies."""
    vertices, faces = [], []
    for i in range(count):
        z = first_z + i * 0.001
        offset = len(vertices)
        vertices.extend((x, y, zz) for y, zz in
                        ((-length / 2, z - 0.00004), (length / 2, z - 0.00004),
                         (length / 2, z + 0.00004), (-length / 2, z + 0.00004)))
        faces.append(tuple(offset + j for j in range(4)))
    visual = mesh("page_edge_detail", vertices, faces, owner, surface)
    fixed_part(owner, "page_edge_detail", [visual])


def select_whole_book(root, notebook):
    for obj in notebook.children_recursive:
        obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = root
    root.select_set(True)


def build_slim_notebook(color, dimensions):
    width, length, thickness = dimensions
    cover_color, accessory, accessory_color = BOOK_COLORS[color]
    cover = mottled_cover(cover_color)
    paper = material("Paper", (0.87, 0.84, 0.74), 0.84)
    trim = material("Fixed accessory", accessory_color, 0.85)
    root = asset("notebook", "bottom center of the closed cover",
                 "+X toward fore-edge; +Y along spine; +Z toward top cover",
                 "short page edge faces -Y; spine is -X")
    root.empty_display_type = "CUBE"
    root.empty_display_size = max(width, length) / 2
    notebook = body("notebook.closed", root)
    notebook["model_scope"] = "closed storage model; fixed loop or ribbon; no page turning or deformation"
    notebook["body_dimensions_m"] = dimensions
    cover_thickness = 0.0012
    page_height = thickness - 2 * cover_thickness
    # Independent manufacturing dimensions, not uniform scaling of a pocket book.
    for name, z in (("cover_bottom", cover_thickness / 2),
                    ("cover_top", thickness - cover_thickness / 2)):
        fixed_part(notebook, name, [rounded_plate(
            name, (width, length, cover_thickness), (0, 0, z), 0.003, cover, notebook)])
    fixed_part(notebook, "pages", [rounded_plate(
        "pages", (width - 0.005, length - 0.006, page_height),
        (0.0005, 0, thickness / 2), 0.0015, paper, notebook)])
    fixed_part(notebook, "spine", [box(
        "spine", (0.003, length - 0.002, thickness - 0.0008),
        (-width / 2 + 0.0015, 0, thickness / 2), cover, notebook)])
    if accessory == "loop":
        # Four thin walls keep the side loop hollow; it remains part of this body.
        x = width / 2 + 0.005
        y = length * 0.28
        walls = []
        for name, z in (("side_loop1", 0.0034), ("side_loop2", 0.0086)):
            walls.append(box(name, (0.016, 0.010, 0.0008), (x, y, z), trim, notebook))
        for name, xx in (("side_loop3", x - 0.0076), ("side_loop4", x + 0.0076)):
            walls.append(box(name, (0.0008, 0.010, 0.0044), (xx, y, 0.006), trim, notebook))
        fixed_part(notebook, "side_loop", walls)
    else:
        # The fixed bookmark protrudes 35 mm beyond the -Y page edge.
        fixed_part(notebook, "bookmark", [box(
            "bookmark", (0.0035, 0.045, 0.0004),
            (width / 2 - 0.025, -length / 2 - 0.0125, 0.0003), trim, notebook)])
    edge = material("Page edge lines", (0.65, 0.63, 0.56), 0.9)
    page_edge_detail(notebook, width / 2 - 0.002 + 0.00004, length - 0.008,
                     cover_thickness + 0.0008, 9, edge)
    select_whole_book(root, notebook)


def build_notebook(color):
    cover_color, strap_color = PROFILES[color]
    cover = material("Cover", cover_color, 0.73)
    paper = material("Paper", (0.89, 0.87, 0.78), 0.84)
    strap = material("Woven strap", strap_color, 0.85)
    root = asset("notebook", "bottom center of the closed strap loop",
                 "+X toward fore-edge; +Y along spine; +Z toward top cover",
                 "short page edge faces -Y; spine is -X")
    root.empty_display_type = "CUBE"
    root.empty_display_size = 0.075
    notebook = body("notebook.closed", root)
    notebook["model_scope"] = "closed storage model; no page turning or elastic simulation"
    notebook["body_dimensions_m"] = (0.103, 0.143, 0.0214)
    # 0.7 mm strap + 2.2 mm cover + 15.6 mm pages + cover + strap = 21.4 mm.
    # The page block includes the old fixed top leaf, with no cloth or air gap.
    fixed_part(notebook, "cover_bottom", [box(
        "cover_bottom", (0.103, 0.143, 0.0022), (0, 0, 0.0018), cover, notebook, bevel=0.0006)])
    fixed_part(notebook, "cover_top", [box(
        "cover_top", (0.103, 0.143, 0.0022), (0, 0, 0.0196), cover, notebook, bevel=0.0006)])
    fixed_part(notebook, "pages", [box(
        "pages", (0.097, 0.136, 0.0156), (0.001, 0, 0.0107), paper, notebook, bevel=0.0003)])
    fixed_part(notebook, "spine", [box(
        "spine", (0.004, 0.140, 0.018), (-0.049, 0, 0.0107), cover, notebook, bevel=0.0005)])
    bands = []
    for name, z in (("strap1", 0.00035), ("strap2", 0.02105)):
        bands.append(box(name, (0.005, 0.143, 0.0007), (0.035, 0, z),
                         strap, notebook, bevel=0.0001))
    for name, y in (("strap3", -0.07115), ("strap4", 0.07115)):
        bands.append(box(name, (0.005, 0.0007, 0.020), (0.035, y, 0.0107),
                         strap, notebook, bevel=0.0001))
    fixed_part(notebook, "strap", bands)
    # Sub-millimetre page marks are visual only; the block provides contact.
    edge = material("Page edge lines", (0.65, 0.63, 0.56), 0.9)
    page_edge_detail(notebook, 0.04954, 0.134, 0.0035, 15, edge)
    # Selecting the asset frame or compound root moves all contact geometry.
    # Decorative pieces cannot be picked and dragged away independently.
    select_whole_book(root, notebook)


def generate_notebook(color, source_file, argv=None, *, size=None):
    # Existing black/navy entry points retain pocket dimensions and fixed contacts.
    build = (lambda: build_notebook(color)) if size is None else (
        lambda: build_slim_notebook(color, BOOK_SIZES[size]))
    generate(build, source_file, argv)
