"""Warm-white perforated basket, reconstructed parametrically from local geometry.

The retained 364 elliptical openings, tapered walls, four large rim openings,
corner posts and dimensions come from the earlier basket mesh. Lower skirts and
edge strips close its assembly seams. Small perforations are visual only in the
collision proxy; all four large openings and the container cavity stay open.
No scene, USD reader, external texture or extracted mesh is needed at build time.
"""
from pathlib import Path
import math
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, box, generate, material, mesh, BOX_FACES
import bpy

WALL_THICKNESS = 0.0022
WALL_SLOPE = 0.12
WALL_BOTTOM = 0.007
WALL_TOP = 0.093
ROWS = 7
HANDLE_BOTTOM = WALL_BOTTOM + 5 * (WALL_TOP - WALL_BOTTOM) / ROWS


def wall_point(axis, sign, base, u, z, offset):
    normal = sign * (base + WALL_SLOPE * z + offset)
    return (normal, u, z) if axis == "X" else (u, normal, z)


def wall_panel(name, axis, sign, base, u0, u1, z0, z1, parent,
               surface=None, shape=None):
    """Closed tapered prism; u bounds may differ between lower and upper edge."""
    lo = (u0, u0) if isinstance(u0, (int, float)) else u0
    hi = (u1, u1) if isinstance(u1, (int, float)) else u1
    vertices = [wall_point(axis, sign, base, (lo, hi)[i][k], z, offset)
                for i in range(2) for offset in (-WALL_THICKNESS/2, WALL_THICKNESS/2)
                for k, z in enumerate((z0, z1))]
    return mesh(name, vertices, BOX_FACES, parent, surface, shape)


def perforated_wall(axis, sign, base, half_span, columns, half_handle,
                    surface, parent):
    """16-sided elliptical holes in rectangular cells, matching recovered USD."""
    vertices, faces = [], []
    pitch = 2 * half_span / columns
    height = (WALL_TOP - WALL_BOTTOM) / ROWS
    segments = 16
    for row in range(ROWS):
        zc = WALL_BOTTOM + (row + .5) * height
        for col in range(columns):
            uc = -half_span + (col + .5) * pitch
            if row >= 5 and abs(uc) < half_handle:
                continue
            start = len(vertices)
            for offset in (-WALL_THICKNESS/2, WALL_THICKNESS/2):
                for inner in (False, True):
                    for i in range(segments):
                        c, s = math.cos(math.tau*i/segments), math.sin(math.tau*i/segments)
                        if inner:
                            du, dz = pitch * .29 * c, height * .25 * s
                        else:
                            factor = max(abs(c), abs(s))
                            du, dz = pitch/2*c/factor, height/2*s/factor
                        vertices.append(wall_point(axis, sign, base, uc+du, zc+dz, offset))
            for i in range(segments):
                j = (i+1) % segments
                faces.extend(tuple(start+k for k in f) for f in (
                    (i,j,16+j,16+i), (32+j,32+i,48+i,48+j),
                    (i,32+i,32+j,j), (16+j,48+j,48+i,16+i)))
    mesh(f"Perforated_{axis}_{sign}", vertices, faces, parent, surface)
    # Recovered cells began above the floor and stopped short of the corner
    # posts. Close those seams without changing any original cell or opening.
    wall_panel(f"Lower_skirt_{axis}_{sign}", axis, sign, base,
               -half_span, half_span, .0015, WALL_BOTTOM, parent, surface)
    adjacent_base = .102 if axis == "X" else .147
    extent = lambda z: max(half_span, adjacent_base + WALL_SLOPE*z + WALL_THICKNESS/2)
    outside = tuple(extent(z) for z in (.0015, WALL_TOP))
    wall_panel(f"Edge_positive_{axis}_{sign}", axis, sign, base,
               half_span-.0001, outside, .0015, WALL_TOP, parent, surface)
    wall_panel(f"Edge_negative_{axis}_{sign}", axis, sign, base,
               tuple(-v for v in outside), -half_span+.0001, .0015, WALL_TOP, parent, surface)
    # One low wall and two upper flanks preserve the large opening. Small
    # elliptical holes intentionally remain solid in this reduced proxy.
    wall_panel(f"Collision_low_{axis}_{sign}", axis, sign, base,
               (-extent(.0015), -extent(HANDLE_BOTTOM)),
               (extent(.0015), extent(HANDLE_BOTTOM)), .0015, HANDLE_BOTTOM,
               parent, shape="CONVEX_HULL")
    wall_panel(f"Collision_upper_negative_{axis}_{sign}", axis, sign, base,
               (-extent(HANDLE_BOTTOM), -extent(WALL_TOP)), -half_handle,
               HANDLE_BOTTOM, WALL_TOP, parent, shape="CONVEX_HULL")
    wall_panel(f"Collision_upper_positive_{axis}_{sign}", axis, sign, base,
               half_handle, (extent(HANDLE_BOTTOM), extent(WALL_TOP)),
               HANDLE_BOTTOM, WALL_TOP, parent, shape="CONVEX_HULL")


def build():
    pp = material("Warm white polypropylene", (.83, .85, .82), .30)
    frame = asset("file_basket/000000", "center of the bottom supporting plane",
                  "+X long dimension; +Y short dimension; +Z up", "long wall at -Y")
    basket = body("file_basket", frame)
    basket["collision_approximation"] = "Small perforations filled; cavity and large rim openings open"
    box("Floor", (.294, .204, .003), (0,0,.0015), pp, basket, bevel=.0007)
    for sign in (-1,1):
        perforated_wall("X", sign, .147, .102, 12, .051, pp, basket)
        perforated_wall("Y", sign, .102, .150, 18, 1/15, pp, basket)
        box(f"Long_rim_{sign}", (.328,.006,.009), (0,sign*.115,.096),
            pp, basket, bevel=.001)
        box(f"Short_rim_{sign}", (.006,.242,.009), (sign*.160,0,.096),
            pp, basket, bevel=.001)
        for other in (-1,1):
            box(f"Corner_{sign}_{other}", (.007,.007,.096),
                (sign*.155,other*.110,.050), pp, basket, bevel=.001)
    # Bake bevels and merge all fixed visual parts into one selectable shell.
    visuals = [obj for obj in bpy.context.scene.objects if obj.get("geometry_role") == "visual"]
    for obj in visuals:
        bpy.context.view_layer.objects.active = obj
        for modifier in list(obj.modifiers):
            if modifier.type == "BEVEL":
                modifier.segments = 3
            bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in visuals:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = visuals[0]
    bpy.ops.object.join()
    shell = bpy.context.object
    shell.name = "Basket_visual"
    # Select the compound root to move the complete visual/collision subtree.
    for obj in basket.children:
        obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    basket.select_set(True)
    bpy.context.view_layer.objects.active = basket


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
