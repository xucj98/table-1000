"""Dressing-table props (scene 000102): one complete rigid body per asset.

Shapes are authored in millimetres and converted to metres by the helpers here.
Every asset is built about a bottom-centre origin, +Z up, +X along the longest
horizontal side and -Y as the front. Dimensions are engineering estimates read
from the reference photo; no brands, text or logos are modelled.
"""

import math

import bpy
import bmesh
from mathutils import Matrix

from table_1000.modeling.asset_builders import asset, body, material, mesh, tube
from table_1000.modeling.small_props_assets import component, finish_whole

S = .001


# ---------------------------------------------------------------- primitives

def circle(r, n=24, cx=0., cy=0., ry=None, phase=0.):
    ry = r if ry is None else ry
    return [(cx + r * math.cos(phase + i * math.tau / n),
             cy + ry * math.sin(phase + i * math.tau / n)) for i in range(n)]


def star(r, inner, n=16, cx=0., cy=0., phase=0.):
    """Alternating radii: petal rim of a flower head."""
    return [(cx + (r if i % 2 == 0 else inner) * math.cos(phase + i * math.tau / n),
             cy + (r if i % 2 == 0 else inner) * math.sin(phase + i * math.tau / n))
            for i in range(n)]


def rrect(w, d, r, n=5, cx=0., cy=0.):
    """Rounded rectangle, counter-clockwise, 4*(n+1) points (n=1: chamfered)."""
    r = max(.05, min(r, w / 2 - .01, d / 2 - .01))
    points = []
    for sx, sy, start in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ccx, ccy = cx + sx * (w / 2 - r), cy + sy * (d / 2 - r)
        for k in range(n + 1):
            a = math.radians(start + 90 * k / n)
            points.append((ccx + r * math.cos(a), ccy + r * math.sin(a)))
    return points


HULL_VERTEX_LIMIT = 64  # PhysX convexHull cooking default; larger hulls get simplified


def _hull_vertices(points):
    bm = bmesh.new()
    for p in dict.fromkeys(tuple(round(c, 7) for c in p) for p in points):
        bm.verts.new(p)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts))
    unused = set(g for g in result["geom_interior"] + result["geom_unused"]
                 if isinstance(g, bmesh.types.BMVert))
    vertices = [tuple(v.co) for v in bm.verts if v not in unused]
    bm.free()
    return vertices


def hull_m(name, points, parent):
    """Closed convex collision mesh around points given in metres.

    Above the PhysX vertex limit, keep a farthest-point subset of the hull vertices
    so the cooked hull equals the authored one.
    """
    vertices = _hull_vertices(points)
    if len(vertices) > HULL_VERTEX_LIMIT:
        import numpy as np
        v = np.array(vertices)
        chosen = [int(np.argmin(v[:, 2]))]
        distance = np.linalg.norm(v - v[chosen[0]], axis=1)
        while len(chosen) < HULL_VERTEX_LIMIT:
            chosen.append(int(np.argmax(distance)))
            distance = np.minimum(distance, np.linalg.norm(v - v[chosen[-1]], axis=1))
        points = [vertices[i] for i in chosen]
    bm = bmesh.new()
    for p in dict.fromkeys(tuple(round(c, 7) for c in p) for p in points):
        bm.verts.new(p)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts))
    unused = list(dict.fromkeys(g for g in result["geom_interior"] + result["geom_unused"]
                                if isinstance(g, bmesh.types.BMVert)))
    bmesh.ops.delete(bm, geom=unused, context="VERTS")
    bm.verts.index_update()
    vertices = [tuple(v.co) for v in bm.verts]
    faces = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return mesh(name, vertices, faces, parent, shape="CONVEX_HULL")


def hull(name, points, parent):
    return hull_m(name, [(x * S, y * S, z * S) for x, y, z in points], parent)


def loft(name, sections, surface, parent, hulls="all", smooth=True, round_sections=False):
    """Closed solid through (z, polygon) sections; hulls = section index ranges."""
    n = len(sections[0][1])
    vertices = [(x * S, y * S, z * S) for z, pts in sections for x, y in pts]
    last = (len(sections) - 1) * n
    faces = [tuple(range(n - 1, -1, -1)), tuple(last + i for i in range(n))]
    for k in range(len(sections) - 1):
        a, b = k * n, (k + 1) * n
        faces.extend((a + i, a + (i + 1) % n, b + (i + 1) % n, b + i) for i in range(n))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = smooth and len(face.vertices) == 4
    if hulls == "all":
        hulls = [(0, len(sections) - 1)]
    for k, (first, end) in enumerate(hulls or []):
        section_hull(f"{name} hull {k}", sections[first:end + 1], parent, round_sections)
    return obj


def section_hull(name, sections, parent, round_sections=False):
    """Convex hull through lofted sections within the PhysX vertex limit.

    Sections are thinned (every k-th point, round sections pushed out so the facets
    still cover the visual polygon) and the flattest interior rings are dropped;
    the first and last sections are always kept.
    """
    n = len(sections[0][1])
    stride = 1
    while len(range(0, n, stride)) * 2 > HULL_VERTEX_LIMIT:
        stride += 1
    if len(range(0, n, stride)) * len(sections) > HULL_VERTEX_LIMIT and n >= 20:
        stride = max(stride, 2)
    m = len(range(0, n, stride))
    scale = math.cos(math.pi / n) / math.cos(math.pi / m) if round_sections and m < n else 1.
    keep = list(range(len(sections)))

    def radius(i):
        z, pts = sections[i]
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        return sum(math.hypot(p[0] - cx, p[1] - cy) for p in pts) / n

    profile = {i: (sections[i][0], radius(i)) for i in keep}
    while len(keep) * m > HULL_VERTEX_LIMIT and len(keep) > 2:
        def error(j):
            (z0, r0), (z1, r1), (z, r) = profile[keep[j - 1]], profile[keep[j + 1]], profile[keep[j]]
            return abs((r1 - r0) * (z - z0) - (z1 - z0) * (r - r0)) / math.hypot(r1 - r0, z1 - z0)
        keep.pop(min(range(1, len(keep) - 1), key=error))
    points = []
    for i in keep:
        z, pts = sections[i]
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        for x, y in pts[::stride]:
            points.append(((cx + (x - cx) * scale) * S, (cy + (y - cy) * scale) * S, z * S))
    return hull_m(name, points, parent)


def lathe(name, rings, surface, parent, n=24, hulls="all", cx=0., cy=0., smooth=True, phase=0.):
    """Convex hulls stay under PhysX's 255-vertex cooking limit: keep n * len(rings) small."""
    return loft(name, [(z, circle(r, n, cx, cy, phase=phase)) for z, r in rings], surface, parent,
                hulls, smooth, round_sections=True)


def block(name, w, d, z0, z1, surface, parent, r=1., bevel=.8, n=3, cx=0., cy=0.,
          collision=True):
    """Rounded-corner box with chamfered top and bottom edges."""
    b = min(bevel, (z1 - z0) / 2.01)
    sections = [(z0, rrect(w - 2 * b, d - 2 * b, r - b, n, cx, cy)),
                (z0 + b, rrect(w, d, r, n, cx, cy)),
                (z1 - b, rrect(w, d, r, n, cx, cy)),
                (z1, rrect(w - 2 * b, d - 2 * b, r - b, n, cx, cy))]
    return loft(name, sections, surface, parent, "all" if collision else None, smooth=False)


def rod(name, p0, p1, r, surface, parent, r1=None, n=6):
    r1 = r if r1 is None else r1
    a = [c * S for c in p0]
    b = [c * S for c in p1]
    axis = [b[i] - a[i] for i in range(3)]
    length = math.sqrt(sum(c * c for c in axis))
    w = [c / length for c in axis]
    helper = (1, 0, 0) if abs(w[0]) < .9 else (0, 1, 0)
    u = [helper[1] * w[2] - helper[2] * w[1], helper[2] * w[0] - helper[0] * w[2],
         helper[0] * w[1] - helper[1] * w[0]]
    norm = math.sqrt(sum(c * c for c in u))
    u = [c / norm for c in u]
    v = [w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0]]
    vertices = []
    for centre, radius in ((a, r * S), (b, r1 * S)):
        for i in range(n):
            c, s = math.cos(i * math.tau / n) * radius, math.sin(i * math.tau / n) * radius
            vertices.append(tuple(centre[k] + c * u[k] + s * v[k] for k in range(3)))
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces.extend((i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return obj


def glass(name, color, roughness=.04, transmission=.95):
    surface = material(name, color, roughness, transmission=transmission)
    surface.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.5
    return surface


# ---------------------------------------------------------------- post passes

def geometry(whole):
    return [o for o in whole.children_recursive if o.get("geometry_role")]


def transform_all(whole, matrix):
    for obj in geometry(whole):
        obj.data.transform(obj.matrix_basis)
        obj.matrix_basis = Matrix.Identity(4)
        obj.data.transform(matrix)


def ground(whole, center_xy=True):
    """Translate so the lowest vertex is on z=0 and (optionally) the XY box is centred."""
    bpy.context.view_layer.update()
    points = [obj.matrix_world @ v.co for obj in geometry(whole) for v in obj.data.vertices]
    dx = -(min(p.x for p in points) + max(p.x for p in points)) / 2 if center_xy else 0
    dy = -(min(p.y for p in points) + max(p.y for p in points)) / 2 if center_xy else 0
    dz = -min(p.z for p in points)
    transform_all(whole, Matrix.Translation((dx, dy, dz)))


def hull_centroid(whole):
    """Volume-weighted centroid of the collision hulls (uniform density, as PhysX)."""
    total, acc = 0., [0., 0., 0.]
    for obj in geometry(whole):
        if obj.get("geometry_role") != "collision":
            continue
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bm.transform(obj.matrix_world)
        volume = abs(bm.calc_volume())
        centre = [sum(v.co[i] for v in bm.verts) / len(bm.verts) for i in range(3)]
        bm.free()
        total += volume
        for i in range(3):
            acc[i] += volume * centre[i]
    return [c / total for c in acc]


def lay_down(whole):
    """Turn a part built along +Z onto +X, then rest it on the lower hull edge under its centroid."""
    transform_all(whole, Matrix.Rotation(math.pi / 2, 4, "Y"))
    bpy.context.view_layer.update()
    com_x = hull_centroid(whole)[0]
    points = sorted({(round(v.co.x, 7), round(v.co.z, 7)) for obj in geometry(whole)
                     if obj.get("geometry_role") == "collision" for v in obj.data.vertices})
    lower = []
    for p in points:
        while len(lower) >= 2 and ((lower[-1][0] - lower[-2][0]) * (p[1] - lower[-2][1])
                                   - (lower[-1][1] - lower[-2][1]) * (p[0] - lower[-2][0])) <= 0:
            lower.pop()
        lower.append(p)
    for a, b in zip(lower, lower[1:]):
        if a[0] <= com_x <= b[0]:
            angle = math.atan2(b[1] - a[1], b[0] - a[0])
            transform_all(whole, Matrix.Rotation(angle, 4, "Y"))
            break
    ground(whole)


def start(category, front="-Y front"):
    root = asset(category, "bottom centre of the resting pose", "+X longest side; +Z up", front)
    return body(category + ".body", root)


def finish(whole, center_xy=True):
    ground(whole, center_xy)
    finish_whole(whole)


# ---------------------------------------------------------------- assets

def vanity_tray(whole):
    white = material("White painted wood", (.86, .84, .79), .55)
    L, W, H, T, F = 360, 230, 55, 8, 6
    hole_w, hole_z0, hole_z1 = 84, 25, 44
    block("Tray floor", L, W, 0, F, white, component(whole, "floor"), r=4, bevel=.8)
    walls = component(whole, "long_walls")
    for y in (-1, 1):
        block(f"Long wall {y:+d}", L, T, F - .5, H, white, walls, r=2, bevel=.8,
              cy=y * (W - T) / 2)
    ends = component(whole, "handle_walls")
    inner = W / 2 - T
    for x in (-1, 1):
        cx = x * (L - T) / 2
        side = (inner - hole_w / 2)
        for y in (-1, 1):
            block(f"End wall side {x:+d}{y:+d}", T, side, F - .5, H, white, ends, r=1, bevel=.8,
                  cx=cx, cy=y * (hole_w / 2 + side / 2))
        block(f"End wall under hole {x:+d}", T, hole_w + .4, F - .5, hole_z0, white, ends,
              r=1, bevel=.6, cx=cx)
        block(f"End wall over hole {x:+d}", T, hole_w + .4, hole_z1, H, white, ends,
              r=1, bevel=.6, cx=cx)


def rect_perfume(whole):
    bottle = glass("Light blue glass", (.66, .84, .97))
    juice = glass("Pale blue perfume", (.42, .68, .93), .1, .7)
    silver = material("Brushed silver cap", (.80, .81, .83), .28, metallic=.95)
    body_part = component(whole, "bottle")
    block("Rectangular glass body", 62, 38, 0, 72, bottle, body_part, r=5, bevel=2.5)
    block("Perfume inside", 52, 28, 7, 60, juice, body_part, r=3, bevel=1, collision=False)
    lathe("Glass neck", [(71, 6), (77.5, 6)], silver, component(whole, "neck"), n=16)
    block("Square silver cap", 26, 26, 77, 95, silver, component(whole, "cap"), r=2, bevel=1.2)


def round_perfume(whole):
    bottle = glass("Amber glass", (.95, .55, .16), .05, .85)
    juice = glass("Orange perfume", (.88, .38, .05), .1, .6)
    gold = material("Polished gold", (.86, .64, .30), .22, metallic=1)
    rings = [(0, 29), (2, 32.5), (10, 35.5), (25, 36), (40, 34), (50, 26), (56, 14), (58.5, 9)]
    body_part = component(whole, "bottle")
    lathe("Round squat glass body", rings, bottle, body_part, n=28)
    lathe("Perfume inside", [(6, 28), (14, 31), (26, 31.5), (38, 29.5), (46, 22)], juice,
          body_part, n=32, hulls=None)
    lathe("Gold collar", [(58, 9), (62.5, 9)], gold, component(whole, "collar"), n=20)
    lathe("Gold cap", [(62, 12), (63.5, 13), (76.5, 13), (78, 11.5)], gold,
          component(whole, "cap"), n=24)


def tall_perfume(whole):
    bottle = material("Deep purple glass", (.16, .03, .26), .08)
    gold = material("Polished gold", (.86, .64, .30), .22, metallic=1)
    body_part = component(whole, "bottle")
    sections = [(0, rrect(35, 23, 3.5)), (2, rrect(38, 26, 4.5)), (77, rrect(38, 26, 4.5)),
                (85, rrect(22, 15, 4))]
    loft("Slim glass body", sections, bottle, body_part, smooth=False)
    lathe("Gold collar", [(84.5, 9), (93, 9)], gold, component(whole, "collar"), n=20)
    block("Gold cap", 22, 16, 92.5, 125, gold, component(whole, "cap"), r=3, bevel=1.5)


def lotion_bottle(whole):
    white = material("White plastic bottle", (.92, .92, .93), .32)
    navy = material("Navy flip cap", (.05, .09, .28), .3)
    blue = material("Pale blue label band", (.62, .74, .90), .45)
    bottle = component(whole, "bottle")
    lathe("Bottle body", [(0, 21), (2, 23.5), (5, 24), (140, 24), (145, 23)], white, bottle)
    tube("Label band", .02405, .0238, .060, (0, 0, .085), blue, bottle, axis="Z",
         segments=24, collision=False)
    cap = component(whole, "cap")
    lathe("Flip cap", [(144.5, 23), (146, 24), (168, 24), (173, 22.5), (175, 19)], navy, cap)
    tube("Lid seam", .02408, .0235, .0008, (0, 0, .1665), material("Seam", (.01, .01, .02), .5),
         cap, axis="Z", segments=24, collision=False)


def deodorant(whole):
    white = material("White plastic stick", (.93, .93, .92), .3)
    mint = material("Mint green cap", (.55, .85, .76), .28)
    grey = material("Grey cap seam", (.55, .57, .58), .4)
    stick = component(whole, "stick")
    body_rings = [(0, 19), (2, 21), (111, 21)]
    cap_rings = [(110.5, 21), (130, 21), (139, 18.5), (145, 14), (148.5, 8.5), (150, 2)]
    lathe("Stick body", body_rings, white, stick, hulls=None)
    tube("Cap seam ring", .02104, .0205, .0012, (0, 0, .1110), grey, stick, axis="Z",
         segments=24, collision=False)
    cap = component(whole, "cap")
    lathe("Dome cap", cap_rings, mint, cap, hulls=None)
    # One convex hull for the whole stick: no internal seam between two hulls.
    section_hull("Stick hull", [(z, circle(r, 24)) for z, r in body_rings[:2] + cap_rings[1:]],
                 stick, round_sections=True)


def serum_bottle(whole):
    black = material("Black matte bottle", (.015, .015, .018), .38)
    gloss = material("Black gloss pump", (.02, .02, .025), .15)
    silver = material("Silver band", (.75, .76, .78), .25, metallic=.9)
    lathe("Bottle body", [(0, 14.5), (2, 16), (72, 16), (78, 13), (82.5, 8)], black,
          component(whole, "bottle"))
    collar = component(whole, "collar")
    lathe("Pump collar", [(82, 8.5), (90, 8.5)], gloss, collar, n=20)
    tube("Collar band", .00855, .0083, .0015, (0, 0, .0845), silver, collar, axis="Z",
         segments=20, collision=False)
    pump = component(whole, "pump")
    lathe("Pump stem", [(89.5, 3), (101, 3)], gloss, pump, n=12, hulls=None)
    block("Pump head", 13, 10, 100, 115, gloss, pump, r=3, bevel=1.5, collision=False)
    rod("Pump nozzle", (0, -4.5, 109), (0, -11, 109), 1.6, gloss, pump, n=8)
    hull("Pump hull", [(x, y, z) for x in (-6.5, 6.5) for y in (-11.5, 5) for z in (89.5, 115)]
         + [(x, y, 89.5) for x, y in circle(3, 8)], pump)


def oil_bottle(whole):
    bottle = glass("Golden amber glass", (.97, .70, .22), .05, .8)
    oil = glass("Golden oil", (.95, .62, .10), .08, .55)
    black = material("Black pump", (.015, .015, .018), .3)
    body_part = component(whole, "bottle")
    sections = [(0, rrect(39, 39, 3)), (2, rrect(42, 42, 4.5)), (95, rrect(42, 42, 4.5)),
                (103, circle(11, 24, phase=0.)), (108, circle(10, 24))]
    loft("Square glass body", sections, bottle, body_part, smooth=False)
    block("Oil inside", 35, 35, 5, 92, oil, body_part, r=3, bevel=1, collision=False)
    lathe("Ribbed collar", [(107.5, 11.5), (118, 11.5)], black, component(whole, "collar"), n=24)
    pump = component(whole, "pump")
    lathe("Pump stem", [(117.5, 3.5), (126, 3.5)], black, pump, n=12, hulls=None)
    block("Pump head", 18, 13, 125, 140, black, pump, r=3.5, bevel=1.5, collision=False)
    rod("Pump nozzle", (0, -6, 134), (0, -15, 133), 2, black, pump, n=8)
    hull("Pump hull", [(x, y, z) for x in (-9, 9) for y in (-15.5, 6.5) for z in (117.5, 140)],
         pump)


def reed_diffuser(whole):
    clear = glass("Clear cut glass", (.92, .95, .96), .03, .97)
    scent = glass("Diffuser oil", (.95, .82, .55), .1, .6)
    gold = material("Polished gold", (.86, .64, .30), .22, metallic=1)
    reed_mat = material("Black rattan reeds", (.02, .02, .02), .6)
    vessel = component(whole, "bottle")
    rings = [(0, 31), (3, 34), (50, 35), (62, 30), (70, 18), (74, 11), (78.5, 10)]
    loft("Faceted glass bottle", [(z, circle(r / math.cos(math.pi / 12), 12, phase=math.pi / 12))
                                   for z, r in rings], clear, vessel, smooth=False)
    lathe("Oil inside", [(6, 28), (40, 30.5)], scent, vessel, n=24, hulls=None)
    lathe("Gold collar", [(78, 12), (84, 12), (85, 11)], gold, component(whole, "collar"), n=24)
    reeds = component(whole, "reeds")
    tips = []
    for i in range(6):
        a = math.radians(i * 60 + 15 + (8 if i % 2 else -6))
        spread = 24 + 6 * (i % 3)
        foot = (3 * math.cos(a), 3 * math.sin(a), 30)
        tip = (spread * math.cos(a), spread * math.sin(a), 240 - 1.6 - 6 * (i % 2))
        rod(f"Reed {i + 1}", foot, tip, 1.6, reed_mat, reeds, r1=1.3)
        tips.append((foot, tip))
    points = []
    for foot, tip in tips:
        t = (85 - foot[2]) / (tip[2] - foot[2])
        mid = tuple(foot[k] + t * (tip[k] - foot[k]) for k in range(3))
        for p in (mid, tip):
            points.extend((p[0] + x, p[1] + y, p[2] + dz) for x, y in circle(2.2, 6)
                          for dz in (-1.6, 1.6))
    hull("Reed bundle hull", points, reeds)


def laptop(whole):
    shell = material("Rose gold aluminium", (.90, .75, .70), .32, metallic=.75)
    dark = material("Dark seam", (.10, .09, .09), .5)
    rubber = material("Grey rubber feet", (.18, .18, .19), .8)
    L, W = 304, 212
    feet = component(whole, "feet")
    for x in (-1, 1):
        for y in (-1, 1):
            lathe(f"Foot {x:+d}{y:+d}", [(0, 4.5), (1.6, 5)], rubber, feet, n=12,
                  cx=x * (L / 2 - 22), cy=y * (W / 2 - 18))
    base = component(whole, "base")
    block("Base shell", L, W, 1.5, 10, shell, base, r=9, bevel=1.4)
    block("Seam shadow", L - 3, W - 3, 9.3, 10.7, dark, base, r=8, bevel=.2, collision=False)
    block("Hinge", 240, 3, 5, 14.5, dark, base, r=1, bevel=.5, cy=W / 2 - 1.6, collision=False)
    lid = component(whole, "lid")
    block("Lid", L, W, 10, 16, shell, lid, r=9, bevel=1.4)
    block("Thumb recess", 52, 3, 9.6, 11.6, dark, lid, r=1, bevel=.3, cy=-W / 2 + 1.3,
          collision=False)


def flower_vase(whole):
    pink = glass("Pink cut glass", (.92, .62, .74), .05, .85)
    water = glass("Vase water", (.80, .90, .82), .05, .8)
    stem_mat = material("Green stems", (.18, .42, .12), .6)
    leaf_mat = material("Leaf green", (.22, .46, .16), .55)
    vase = component(whole, "vase")
    sections = [(0, rrect(70, 70, 10, 1)), (3, rrect(75, 75, 11, 1))]
    for k in range(9):
        z = 3 + 8.5 * (k + 1)
        sections.append((z - 4.25, rrect(72.5, 72.5, 10.5, 1)))
        sections.append((z, rrect(75, 75, 11, 1)))
    sections += [(92, rrect(62, 62, 10, 1)), (98, rrect(52, 52, 8, 1)),
                 (105, rrect(56, 56, 9, 1))]
    loft("Faceted square vase", sections, pink, vase, hulls=None, smooth=False)
    hull("Vase hull", [(x, y, z) for z, pts in (sections[0], sections[1], sections[-4])
                       for x, y in pts] + [(x, y, 105) for x, y in rrect(75, 75, 11, 1)], vase)
    block("Water", 62, 62, 6, 70, water, vase, r=8, bevel=1, collision=False)
    flowers = component(whole, "bouquet")
    heads = [  # x, y, z, radius, colour
        (0, 0, 214, 17, "white"), (-26, 12, 205, 18, "pink"), (24, -14, 203, 17, "pink"),
        (-10, -30, 197, 15, "white"), (14, 28, 200, 16, "white"), (36, 14, 186, 15, "rose"),
        (-38, -12, 184, 16, "rose"), (-18, 32, 180, 14, "pink"), (22, -36, 178, 14, "white"),
        (6, 10, 190, 13, "rose"), (-44, 22, 168, 12, "white"), (42, -26, 166, 12, "pink"),
        (30, 34, 176, 13, "pink"), (-30, -36, 170, 13, "pink"), (0, -14, 206, 14, "rose"),
    ]
    heads = [(1.25 * x, 1.25 * y, z, 1.25 * r, c) for x, y, z, r, c in heads]
    colours = {"white": material("White petals", (.95, .94, .90), .6),
               "pink": material("Pink petals", (.86, .40, .60), .55),
               "rose": material("Deep rose petals", (.72, .20, .40), .55)}
    centre = material("Yellow flower centre", (.90, .72, .20), .6)
    head_points = []
    for i, (x, y, z, r, c) in enumerate(heads):
        h = .55 * r
        sections = [(z - h / 2, star(.35 * r, .3 * r, 16, x, y)),
                    (z - .25 * h, star(.85 * r, .72 * r, 16, x, y)),
                    (z + .05 * h, star(r, .8 * r, 16, x, y)),
                    (z + .35 * h, star(.8 * r, .68 * r, 16, x, y)),
                    (z + h / 2, star(.4 * r, .35 * r, 16, x, y))]
        loft(f"Flower head {i + 1}", sections, colours[c], flowers, hulls=None)
        lathe(f"Flower centre {i + 1}", [(z + .3 * h, .25 * r), (z + .55 * h, .2 * r)],
              centre, flowers, n=8, cx=x, cy=y, hulls=None)
        rod(f"Stem {i + 1}", (.15 * x, .15 * y, 20), (x, y, z - h / 2 + .5), 1.4, stem_mat,
            flowers)
        head_points.extend((px, py, pz) for pz, pts in sections for px, py in pts)
    for i, (x, y, z, rx, ry, a) in enumerate([(20, 8, 118, 18, 7, .5), (-18, -10, 122, 16, 6, 2.6),
                                              (-4, 22, 128, 15, 6, 1.6)]):
        ring = [(x + math.cos(a) * px - math.sin(a) * py, y + math.sin(a) * px + math.cos(a) * py)
                for px, py in circle(rx, 10, ry=ry)]
        loft(f"Leaf {i + 1}", [(z - 1, ring), (z + 1, ring)], leaf_mat, flowers, hulls=None)
    hull("Stem bundle hull", [(x, y, z) for z in (104, 160) for x, y in circle(16, 8)], flowers)
    hull("Flower heads hull", head_points, flowers)


def lipstick(whole, style):
    if style == "gold":
        case = material("Gold lipstick case", (.85, .65, .32), .22, metallic=1)
        band = material("Dark gold band", (.55, .38, .16), .3, metallic=1)
        cap_mat = case
    elif style == "rose":
        case = material("Rose gold lipstick case", (.90, .64, .56), .22, metallic=1)
        band = material("Light rose band", (.96, .80, .74), .25, metallic=1)
        cap_mat = case
    else:
        case = material("Black lipstick case", (.02, .02, .025), .2)
        band = material("Red ring", (.70, .03, .06), .3)
        cap_mat = case
    tube_part = component(whole, "base")
    lathe("Base tube", [(0, 9), (1, 10), (30, 10)], case, tube_part, n=20, hulls=None)
    tube("Band ring", .01005, .0094, .003, (0, 0, .0315), band, tube_part, axis="Z",
         segments=20, collision=False)
    cap = component(whole, "cap")
    lathe("Cap", [(30, 10), (74, 10), (75, 9)], cap_mat, cap, n=20, hulls=None)
    section_hull("Lipstick hull", [(z, circle(r, 20)) for z, r in ((0, 9), (1, 10), (74, 10), (75, 9))],
                 cap, round_sections=True)


def hairbrush(whole):
    pink = material("Pink brush body", (.93, .60, .70), .35)
    white = material("White cushion", (.95, .94, .93), .45)
    bristle = material("White bristles", (.92, .92, .90), .4)
    tip = material("Pink bristle tips", (.85, .40, .58), .4)
    back = component(whole, "paddle")
    cx = 57.5
    loft("Paddle back", [(0, rrect(118, 68, 26, 4, cx)), (3.5, rrect(125, 75, 30, 4, cx)),
                         (11, rrect(125, 75, 30, 4, cx)), (14, rrect(120, 70, 27, 4, cx))],
         pink, back)
    handle = component(whole, "handle")
    loft("Handle", [(0, rrect(125, 24, 9, 3, -57.5)), (3, rrect(125, 28, 11, 3, -57.5)),
                    (11, rrect(125, 28, 11, 3, -57.5)), (14, rrect(125, 24, 9, 3, -57.5))],
         pink, handle)
    bed = component(whole, "bristles")
    loft("Cushion", [(13.5, rrect(108, 62, 25, 4, cx)), (17, rrect(108, 62, 25, 4, cx))],
         white, bed, hulls=None)
    for i, x in enumerate(range(12, 104, 9)):
        for j, y in enumerate(range(-24, 25, 8)):
            if ((x - cx) / 50) ** 2 + (y / 27) ** 2 > 1:
                continue
            rod(f"Bristle {i}_{j}", (x, y, 16.5), (x, y, 26.5), .9, bristle, bed, n=5)
            rod(f"Bristle tip {i}_{j}", (x, y, 26.3), (x, y, 28.5), 1.5, tip, bed, r1=1.2, n=6)
    hull("Bristle bed hull", [(x, y, z) for z in (13.5, 28.5)
                              for x, y in rrect(108, 62, 25, 4, cx)], bed)


def compact_mirror(whole):
    rose = material("Rose gold compact", (.90, .66, .58), .25, metallic=.95)
    seam = material("Compact seam", (.20, .14, .12), .5)
    case = component(whole, "case")
    lathe("Compact case", [(0, 34.5), (1.5, 36.8), (6.5, 37.5), (8.5, 37.5), (13.5, 36.8),
                           (15, 33)], rose, case, n=32)
    tube("Hinge seam", .0375, .0365, .0009, (0, 0, .0075), seam, case, axis="Z",
         segments=40, collision=False)
    lathe("Top medallion", [(14.9, 22), (15.25, 21)], rose, case, n=40, hulls=None)
    block("Clasp", 10, 3, 5.5, 9.5, rose, component(whole, "clasp"), r=1, bevel=.6,
          cy=-37, collision=False)


def makeup_brush(whole, style):
    if style == "powder":
        handle_mat = material("Black lacquer handle", (.02, .02, .022), .2)
        ferrule_mat = material("Gold ferrule", (.86, .64, .30), .22, metallic=1)
        tuft_mat = material("Taupe brush hair", (.55, .40, .32), .75)
        handle = [(0, 4), (2, 5), (95, 7)]
        ferrule = [(94.5, 7), (100, 7.5), (120, 8)]
        tuft = [(119.5, 8), (125, 10), (135, 13.5), (148, 14), (160, 11), (166, 7), (170, 2.5)]
    else:
        handle_mat = material("White handle", (.94, .94, .93), .3)
        ferrule_mat = material("Silver ferrule", (.78, .79, .80), .25, metallic=.95)
        tuft_mat = material("Dark brush hair", (.12, .09, .08), .7)
        handle = [(0, 3), (2, 3.6), (100, 4.5)]
        ferrule = [(99.5, 4.5), (121, 3.6)]
        tuft = [(120.5, 3.6), (130, 4), (141, 3), (150, 1)]
    # Half-segment phase: once laid down the brush rests on flat facets, not on a vertex ridge.
    flat = math.pi / 16
    lathe("Handle", handle, handle_mat, component(whole, "handle"), n=16, phase=flat)
    lathe("Ferrule", ferrule, ferrule_mat, component(whole, "ferrule"), n=16, phase=flat)
    lathe("Tuft", tuft, tuft_mat, component(whole, "tuft"), n=16, phase=flat)


def cosmetic_cup(whole):
    acrylic = glass("Clear acrylic", (.93, .96, .98), .05, .95)
    floor = component(whole, "floor")
    lathe("Cup floor", [(0, 39.3), (1, 40), (4, 40)], acrylic, floor, n=24)
    walls = component(whole, "walls")
    tube("Cup wall", .040, .037, .106, (0, 0, .057), acrylic, walls, axis="Z", segments=24)
    tube("Rounded rim", .0401, .0369, .0012, (0, 0, .1094), acrylic, walls, axis="Z",
         segments=24, collision=False)


def nail_polish(whole, style):
    polish = {"red": (.70, .03, .06), "nude": (.86, .58, .55), "plum": (.30, .05, .18)}[style]
    cap_mat = (material("Gold cap", (.86, .64, .30), .22, metallic=1) if style == "nude"
               else material("Black gloss cap", (.02, .02, .022), .15))
    bottle = component(whole, "bottle")
    block("Square glass bottle", 30, 30, 0, 36, glass("Clear thick glass", (.94, .96, .97)),
          bottle, r=3, bevel=1.5)
    block("Polish inside", 25, 25, 4, 33, material("Nail polish", polish, .15), bottle, r=2,
          bevel=.8, collision=False)
    lathe("Bottle neck", [(35.5, 6), (39, 6)], cap_mat, component(whole, "neck"), n=16)
    lathe("Tall cap", [(38.5, 7.5), (69, 7.5), (70, 6.8)], cap_mat, component(whole, "cap"), n=20)


def cream_jar(whole, style):
    if style == "white":
        jar_mat = material("White jar", (.93, .93, .92), .3)
        lid_mat = material("Silver lid", (.80, .81, .83), .25, metallic=.95)
        r, h, lid_h = 31, 30, 15
    else:
        jar_mat = glass("Frosted pink glass", (.95, .70, .78), .45, .6)
        lid_mat = material("Gold lid", (.86, .64, .30), .22, metallic=1)
        r, h, lid_h = 27.5, 26, 14
    lathe("Jar body", [(0, r - 2), (2, r - .5), (h, r - .5)], jar_mat,
          component(whole, "jar"), n=32)
    lathe("Screw lid", [(h - .5, r), (h + lid_h - 1.2, r), (h + lid_h, r - 1.5)], lid_mat,
          component(whole, "lid"), n=32)


def faceted_perfume(whole):
    clear = glass("Clear faceted glass", (.93, .96, .97), .03, .97)
    juice = glass("Pale gold perfume", (.96, .86, .60), .1, .6)
    silver = material("Silver ball cap", (.80, .81, .83), .2, metallic=.95)
    bottle = component(whole, "bottle")
    rings = [(0, 21), (2, 24), (18, 25), (36, 22), (44, 12), (46, 6)]
    loft("Faceted glass body", [(z, circle(r / math.cos(math.pi / 10), 10, phase=math.pi / 10))
                                 for z, r in rings], clear, bottle, smooth=False)
    lathe("Perfume inside", [(5, 19), (18, 21), (32, 19)], juice, bottle, n=20, hulls=None)
    lathe("Silver collar", [(45.5, 6), (48.5, 6)], silver, component(whole, "collar"), n=16)
    ball = [(48 + 11 - 11 * math.cos(a), 11 * math.sin(a))
            for a in [math.pi * k / 8 for k in range(1, 8)]]
    lathe("Ball cap", [(48, 4)] + ball + [(70, 1)], silver, component(whole, "cap"), n=20)


def comb(whole):
    black = material("Black comb plastic", (.025, .025, .028), .35)
    spine = component(whole, "spine")
    block("Comb spine", 190, 12, 0, 6, black, spine, r=4, bevel=.8, cy=16.5)
    teeth = component(whole, "teeth")
    count = 19
    for i in range(count):
        x = -88 + i * (176 / (count - 1))
        loft(f"Tooth {i + 1}", [(0, rrect(4.2, 34, 1.2, 1, x, -5.5)),
                                (4.5, rrect(3.4, 34, 1, 1, x, -5.5))], black, teeth, hulls=None)
    for k, (x0, x1) in enumerate(((-90, -30), (-30, 30), (30, 90))):
        hull(f"Teeth slab {k}", [(x, y, z) for x in (x0, x1) for y in (-22.5, 11)
                                 for z in (0, 4)], teeth)


def trinket_dish(whole):
    ceramic = material("White glazed ceramic", (.94, .93, .91), .18)
    gold = material("Gold rim", (.86, .64, .30), .22, metallic=1)
    lathe("Dish floor", [(0, 33), (1, 35), (4, 38.5)], ceramic, component(whole, "floor"), n=24)
    walls = component(whole, "walls")
    tube("Bowl wall", .0385, .0355, .018, (0, 0, .0129), ceramic, walls, axis="Z",
         outer_end=.045, inner_end=.042, segments=24)
    tube("Gold rim", .04515, .04185, .0009, (0, 0, .02175), gold, walls, axis="Z",
         segments=24, collision=False)


def cotton_pad_jar(whole):
    acrylic = glass("Clear acrylic", (.93, .96, .98), .05, .95)
    rose = material("Rose gold lid", (.90, .66, .58), .25, metallic=.95)
    cotton = material("White cotton pads", (.97, .97, .96), .9)
    jar = component(whole, "jar")
    lathe("Jar floor", [(0, 34), (1, 35), (3, 35)], acrylic, jar, n=32, hulls=None)
    tube("Jar wall", .035, .0335, .077, (0, 0, .0415), acrylic, jar, axis="Z", segments=32,
         collision=False)
    section_hull("Jar hull", [(z, circle(r, 32)) for z, r in ((0, 34), (1, 35), (80, 35))], jar,
                 round_sections=True)
    pads = component(whole, "cotton_pads")
    for i in range(22):
        z = 3.2 + i * 3.4
        lathe(f"Cotton pad {i + 1}", [(z, 29.5), (z + .6, 30.5), (z + 2.6, 30.5), (z + 3.2, 29.5)],
              cotton, pads, n=20, hulls=None, cx=.4 * math.sin(i * 1.7), cy=.4 * math.cos(i * 2.3))
    lathe("Screw lid", [(79.5, 36), (93.5, 36), (95, 34.5)], rose, component(whole, "lid"), n=32)


def tissue_box(whole):
    card = material("White card box", (.95, .94, .92), .55)
    colours = [material("Pastel pink print", (.92, .55, .65), .55),
               material("Pastel mint print", (.50, .82, .70), .55),
               material("Pastel lilac print", (.68, .58, .88), .55)]
    slot = material("Slot shadow", (.30, .28, .28), .7)
    tissue_mat = material("White tissue", (.98, .98, .97), .85)
    box_part = component(whole, "box")
    block("Tissue box", 115, 115, 0, 130, card, box_part, r=2, bevel=1)
    block("Print band", 115.6, 115.6, 36, 94, material("Cream print band", (.98, .90, .86), .55),
          box_part, r=2.2, bevel=.1, n=2, collision=False)
    for side in range(4):
        for k in range(6):
            t = -47 + k * 18.8
            c = colours[(k + side) % 3]
            for z0 in (42, 70):
                zz = z0 + (14 if k % 2 else 0) * .5
                if side % 2 == 0:
                    x, y, w, d = t, (58 if side == 0 else -58), 12, .6
                else:
                    x, y, w, d = (58 if side == 1 else -58), t, .6, 12
                block(f"Print {side}_{k}_{z0}", w, d, zz, zz + 12, c, box_part, r=.25, bevel=.1,
                      n=1, cx=x, cy=y, collision=False)
    block("Top slot", 80, 14, 129.6, 130.3, slot, box_part, r=6.5, bevel=.1, n=4, collision=False)
    tissue = component(whole, "tissue")
    sections = [(126, rrect(70, 2.4, 1, 2)), (145, rrect(86, 3.5, 1.5, 2, 2, 1)),
                (163, rrect(74, 3, 1.2, 2, -4, 3))]
    loft("Tissue sheet", sections, tissue_mat, tissue)


BUILDERS = {
    "vanity_tray": vanity_tray,
    "perfume_bottle/rect": rect_perfume,
    "perfume_bottle/round": round_perfume,
    "perfume_bottle/tall": tall_perfume,
    "lotion_bottle": lotion_bottle,
    "deodorant": deodorant,
    "serum_bottle": serum_bottle,
    "oil_bottle": oil_bottle,
    "reed_diffuser": reed_diffuser,
    "laptop": laptop,
    "flower_vase": flower_vase,
    "lipstick/gold": lambda w: lipstick(w, "gold"),
    "lipstick/black": lambda w: lipstick(w, "black"),
    "hairbrush": hairbrush,
    "compact_mirror": compact_mirror,
    "makeup_brush/powder": lambda w: makeup_brush(w, "powder"),
    "makeup_brush/eye": lambda w: makeup_brush(w, "eye"),
    "cosmetic_cup": cosmetic_cup,
    "nail_polish/red": lambda w: nail_polish(w, "red"),
    "nail_polish/nude": lambda w: nail_polish(w, "nude"),
    "nail_polish/plum": lambda w: nail_polish(w, "plum"),
    "cream_jar/white": lambda w: cream_jar(w, "white"),
    "cream_jar/pink": lambda w: cream_jar(w, "pink"),
    "lipstick/rose": lambda w: lipstick(w, "rose"),
    "perfume_bottle/faceted": faceted_perfume,
    "comb": comb,
    "trinket_dish": trinket_dish,
    "cotton_pad_jar": cotton_pad_jar,
    "tissue_box": tissue_box,
}
LYING = {"makeup_brush/powder", "makeup_brush/eye"}
UNCENTRED = {"reed_diffuser", "flower_vase"}


def build_vanity_asset(kind):
    category = kind.split("/")[0]
    whole = start(category)
    BUILDERS[kind](whole)
    if kind in LYING:
        lay_down(whole)
    finish(whole, center_xy=kind not in UNCENTRED)
