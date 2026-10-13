"""24/6 staple strip: user dimensions and confirmed round-wire section.

Origin: centre of the two foot-tip planes. +X strip/feed length (rearward
in the target stapler); +Y crown width; +Z toward the crown. All 50 staples
belong to one Compound body. Separation, adhesive and deformation are absent.
"""
import math

import bpy
import bmesh

from table_1000.modeling.asset_builders import asset, body, generate, material, mesh, part


OUTER_WIDTH_M = 0.0127
LEG_LENGTH_M = 0.006
WIRE_DIAMETER_M = 0.00051
HEIGHT_BASIS = "outside_total_height_candidate"
WIRE_SECTION = "round_diameter_confirmed_by_user"
STAPLES_PER_STRIP = 50
PITCH_M = WIRE_DIAMETER_M  # Tangent round wires; no adhesive gap assumed.
BEND_RADIUS_M = WIRE_DIAMETER_M  # Centreline bend radius, modeling estimate.
RING_SEGMENTS = 8
BEND_SEGMENTS = 2


def wire_rings(x, overall_height_m):
    """Octagonal circular-wire sweep, with exact outside width/height extrema."""
    r = WIRE_DIAMETER_M / 2
    c = OUTER_WIDTH_M / 2 - r
    top = overall_height_m - r
    bend = BEND_RADIUS_M
    # Each tuple is centre Y/Z followed by the centreline tangent Y/Z.
    path = [(-c, 0, 0, 1), (-c, top - bend, 0, 1)]
    for step in range(1, BEND_SEGMENTS + 1):
        a = math.pi - step * math.pi / (2 * BEND_SEGMENTS)
        path.append((-c + bend + bend * math.cos(a), top - bend + bend * math.sin(a),
                     math.sin(a), -math.cos(a)))
    path.append((c - bend, top, 1, 0))
    for step in range(1, BEND_SEGMENTS + 1):
        a = math.pi / 2 - step * math.pi / (2 * BEND_SEGMENTS)
        path.append((c - bend + bend * math.cos(a), top - bend + bend * math.sin(a),
                     math.sin(a), -math.cos(a)))
    path.append((c, 0, 0, -1))
    return [[(x + r * math.cos(a), y - tz * r * math.sin(a), z + ty * r * math.sin(a))
             for a in (i * math.tau / RING_SEGMENTS for i in range(RING_SEGMENTS))]
            for y, z, ty, tz in path]


def hull(points):
    """Closed convex segment; never take a convex hull of an entire U."""
    bm = bmesh.new()
    for point in sorted({tuple(round(c, 12) for c in p) for p in points}):
        bm.verts.new(point)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts), use_existing_faces=False)
    unused = list({v for v in result["geom_unused"] + result["geom_interior"]
                   if isinstance(v, bmesh.types.BMVert)})
    if unused:
        bmesh.ops.delete(bm, geom=unused, context="VERTS")
    bm.verts.ensure_lookup_table()
    bm.verts.index_update()
    vertices = [tuple(v.co) for v in bm.verts]
    faces = [tuple(v.index for v in face.verts) for face in bm.faces]
    bm.free()
    return vertices, faces


def build(overall_height_m=LEG_LENGTH_M):
    steel = material("Galvanized steel candidate", (0.58, 0.62, 0.66), 0.32, metallic=0.82)
    root = asset("staple", "foot-tip plane centre", "+X strip length; +Y crown; +Z up",
                 "leading staple at -X; open legs toward -Z")
    root.empty_display_size = 0.003
    root.hide_select = True
    strip = body("staple.strip", root)
    strip["parameter_source"] = "user input; not measured or manufacturer-verified"
    strip["outer_width_m"] = OUTER_WIDTH_M
    strip["given_leg_length_m"] = LEG_LENGTH_M
    strip["overall_height_m"] = overall_height_m
    strip["height_basis"] = HEIGHT_BASIS if overall_height_m == LEG_LENGTH_M else "fixture_variant"
    strip["wire_section"] = WIRE_SECTION
    strip["wire_diameter_m"] = WIRE_DIAMETER_M
    strip["staple_count"] = STAPLES_PER_STRIP
    strip["pitch_m"] = PITCH_M
    strip["separation_supported"] = False
    strip["collision_approximation"] = "seven wire-section profiles extruded along strip; inter-wire grooves filled"
    # Crown includes both rounded shoulders; each row of legs remains separate.
    nodes = {name: part(strip.name + "." + name, strip)
             for name in ("crown", "leg_left", "leg_right")}
    vertices = []
    faces = {name: [] for name in nodes}
    n = RING_SEGMENTS
    for staple in range(STAPLES_PER_STRIP):
        x = (staple - (STAPLES_PER_STRIP - 1) / 2) * PITCH_M
        rings = wire_rings(x, overall_height_m)
        offset = len(vertices)
        vertices.extend(p for ring in rings for p in ring)
        faces["leg_left"].append(tuple(offset + i for i in range(n - 1, -1, -1)))
        faces["leg_right"].append(tuple(offset + (len(rings) - 1) * n + i for i in range(n)))
        for section, (a, b) in enumerate(zip(rings, rings[1:])):
            name = "leg_left" if section == 0 else "leg_right" if section == len(rings) - 2 else "crown"
            faces[name].extend((offset + section * n + i, offset + section * n + (i + 1) % n,
                               offset + (section + 1) * n + (i + 1) % n, offset + (section + 1) * n + i)
                              for i in range(n))
    # Partition the original surface at matching rings without adding seam caps
    # or duplicate faces. The union is the unchanged, closed 50-wire surface.
    for name, subset in faces.items():
        indices = sorted({index for face in subset for index in face})
        remap = {index: i for i, index in enumerate(indices)}
        visual = mesh(nodes[name].name + ".visual", [vertices[i] for i in indices],
                      [tuple(remap[i] for i in face) for face in subset], nodes[name], steel)
        visual.hide_select = True
    # Preserve the seven original Y/Z convex section profiles, but aggregate
    # their 50 repeated copies along X. This retains the U void and foot plane;
    # it intentionally omits sub-pitch grooves for whole-strip handling.
    rings = wire_rings(0, overall_height_m)
    half_length = ((STAPLES_PER_STRIP - 1) * PITCH_M + WIRE_DIAMETER_M) / 2
    crown_index = 0
    for section, (a, b) in enumerate(zip(rings, rings[1:])):
        name = "leg_left" if section == 0 else "leg_right" if section == len(rings) - 2 else "crown"
        suffix = ""
        if name == "crown":
            crown_index += 1
            suffix = str(crown_index)
        hv, hf = hull([(x, y, z) for x in (-half_length, half_length)
                       for _, y, z in a + b])
        proxy = mesh(nodes[name].name + ".collision" + suffix, hv, hf, nodes[name],
                     shape="CONVEX_HULL")
        proxy.hide_select = True
    for node in nodes.values():
        node.hide_select = True
    # The complete strip is the only selectable moving object.
    bpy.ops.object.select_all(action="DESELECT")
    strip.hide_select = False
    strip.select_set(True)
    bpy.context.view_layer.objects.active = strip


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
