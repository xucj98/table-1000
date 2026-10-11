"""24/6 staple strip: user dimensions and confirmed round-wire section.

Origin: centre of the two foot-tip planes. +X strip/feed length (rearward
in the target stapler); +Y crown width; +Z toward the crown. All 50 staples
belong to one Compound body. Separation, adhesive and deformation are absent.
"""
import math
from pathlib import Path
import sys

import bpy
import bmesh

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, generate, material, mesh


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
    for point in points:
        bm.verts.new(point)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts), use_existing_faces=False)
    unused = [v for v in result["geom_unused"] + result["geom_interior"]
              if isinstance(v, bmesh.types.BMVert)]
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
    root = asset("staple/000000", "foot-tip plane centre", "+X strip length; +Y crown; +Z up",
                 "leading staple at -X; open legs toward -Z")
    root.empty_display_size = 0.003
    strip = body("Staple strip", root)
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
    vertices, faces = [], []
    n = RING_SEGMENTS
    for staple in range(STAPLES_PER_STRIP):
        x = (staple - (STAPLES_PER_STRIP - 1) / 2) * PITCH_M
        rings = wire_rings(x, overall_height_m)
        offset = len(vertices)
        vertices.extend(p for ring in rings for p in ring)
        faces.extend([tuple(offset + i for i in range(n - 1, -1, -1)),
                      tuple(offset + (len(rings) - 1) * n + i for i in range(n))])
        for section, (a, b) in enumerate(zip(rings, rings[1:])):
            faces.extend((offset + section * n + i, offset + section * n + (i + 1) % n,
                          offset + (section + 1) * n + (i + 1) % n, offset + (section + 1) * n + i)
                         for i in range(n))
            hv, hf = hull(a + b)
            proxy = mesh(f"Collision_staple_{staple:02d}_{section}", hv, hf, strip,
                         shape="CONVEX_HULL")
            proxy.hide_select = True
    visual = mesh("Complete 50-staple strip", vertices, faces, strip, steel)
    # Select/move the Compound root; neither wire pieces nor proxies are loose.
    visual.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    strip.hide_select = False
    strip.select_set(True)
    bpy.context.view_layer.objects.active = strip


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
