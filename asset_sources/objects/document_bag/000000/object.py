"""Blue continuous thin document bag, with explicit stage-one mouth geometry.

Reuses the approved A-direction model. Fixed-topology visual and convex meshes
share one surface; this is geometric deformation, not physical cloth support.
"""

from __future__ import annotations

import json
import math

import bpy
from mathutils import Vector

from table_1000.modeling.asset_builders import (
    asset, body, deformable, generate, box as build_box, joint, material, mesh, part,
)
THICKNESS = 0.0006
X_GRID = [-0.170, *[-0.162 + i * 0.324 / 16 for i in range(17)], 0.170]
Y_GRID = [-0.125, -0.110, -0.075, 0.0, 0.075, 0.110, 0.125]
PRISM_FACES = [(2, 1, 0), (3, 4, 5), (0, 1, 4, 3),
               (1, 2, 5, 4), (2, 0, 3, 5)]


def box(name, *args, **kwargs):
    obj = build_box(name, *args, **kwargs)
    proxy = bpy.data.objects.get("Collision_" + name)
    if proxy:
        proxy.name = name.replace(".visual", ".collision")
    return obj


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3 - 2 * value)


def aperture(x, q):
    t = x + 0.162
    if q <= 0 or t <= 0 or t >= q:
        return 0.0
    return 0.040 * min(1.0, q / 0.120) * math.sin(math.pi * t / q) ** 2


def pose_x_grid(q):
    """Place mesh knots at the mouth peak and zipper closure boundary.

    Analytic zero values at grid vertices alone do not seal a triangle edge
    beyond an unsampled slider position. Keep fixed topology and pin two
    interior columns instead; these are geometric samples, not material points.
    """
    xs = list(X_GRID)
    if q <= 0:
        return xs
    spacing = .324 / 16
    boundary = len(xs)-2 if q >= .324 else max(3, min(len(xs)-3, round(q/spacing)+1))
    peak = max(2, min(boundary-1, round(q/(2*spacing))+1))
    xs[boundary] = -.162 + q
    xs[peak] = -.162 + q/2
    assert all(b > a for a,b in zip(xs,xs[1:])), "Collapsed film sampling cell"
    return xs


def clearance(x, y, q, bulge):
    lateral = 1 - smoothstep((abs(x) - 0.130) / 0.040)
    depth = smoothstep((y + 0.125) / 0.050)
    back = 1 - smoothstep((y - 0.075) / 0.050)
    belly = (0.0018 + 0.039 * bulge) * lateral
    return ((1 - depth) * aperture(x, q) + depth * belly) * back


def grid_material():
    surface = material("Blue translucent mesh film", (0.10, 0.42, 0.72),
                       roughness=0.38, transmission=0.60)
    nodes = surface.node_tree.nodes
    links = surface.node_tree.links
    coordinate = nodes.new("ShaderNodeTexCoord")
    separate = nodes.new("ShaderNodeSeparateXYZ")
    links.new(coordinate.outputs["Object"], separate.inputs["Vector"])
    lines = []
    for axis in ("X", "Y"):
        frequency = nodes.new("ShaderNodeMath")
        frequency.operation = "MULTIPLY"
        frequency.inputs[1].default_value = 400
        links.new(separate.outputs[axis], frequency.inputs[0])
        fraction = nodes.new("ShaderNodeMath")
        fraction.operation = "FRACT"
        links.new(frequency.outputs[0], fraction.inputs[0])
        threshold = nodes.new("ShaderNodeMath")
        threshold.operation = "LESS_THAN"
        threshold.inputs[1].default_value = 0.11
        links.new(fraction.outputs[0], threshold.inputs[0])
        lines.append(threshold)
    combine = nodes.new("ShaderNodeMath")
    combine.operation = "MAXIMUM"
    for i, line in enumerate(lines):
        links.new(line.outputs[0], combine.inputs[i])
    color = nodes.new("ShaderNodeMixRGB")
    color.inputs[1].default_value = (0.10, 0.42, 0.72, 1)
    color.inputs[2].default_value = (0.72, 0.83, 0.87, 1)
    links.new(combine.outputs[0], color.inputs[0])
    links.new(color.outputs[0], nodes.get("Principled BSDF").inputs["Base Color"])
    return surface


class BagGeometry:
    def __init__(self):
        scene = bpy.context.scene
        scene.unit_settings.system = "METRIC"
        scene.unit_settings.scale_length = 1
        self.root = asset("document_bag", "flat bag bottom center", "+X zipper opening; +Y cavity; +Z up", "mouth at -Y")
        self.film_owner = deformable("document_bag.film", self.root)
        self.top_part = part("document_bag.film.upper_film", self.film_owner)
        bottom = part("document_bag.film.lower_film", self.film_owner)
        seam = part("document_bag.film.seam_binding", self.film_owner)
        top_zip = part("document_bag.film.upper_zipper_tape", self.film_owner)
        bottom_zip = part("document_bag.film.lower_zipper_tape", self.film_owner)
        blue = material("Blue zipper tape", (0.025, 0.10, 0.36), 0.55)
        silver = material("Zipper metal", (0.38, 0.42, 0.45), 0.28, metallic=0.7)
        film = grid_material()
        self.dynamic = []
        self.top = self.panel(self.top_part, X_GRID, Y_GRID, film, True)
        box(bottom.name + ".visual", (0.340, 0.250, THICKNESS),
            (0, 0, THICKNESS / 2), film, bottom)
        self.panel(top_zip, X_GRID, [-0.125, -0.122], blue, False)
        box(bottom_zip.name + ".visual", (0.324, 0.003, 0.00035),
            (0, -0.1235, 0.000425), blue, bottom_zip)
        for i, dimensions, position in [
            (1, (0.003, 0.250, 0.0012), (-0.1685, 0, 0.0006)),
            (2, (0.003, 0.250, 0.0012), (0.1685, 0, 0.0006)),
            (3, (0.334, 0.003, 0.0012), (0, 0.1235, 0.0006)),
        ]:
            box(seam.name + f".visual{i}", dimensions, position, blue, seam)
        self.slider = body("document_bag.slider", self.root)
        slider_part = part("document_bag.slider.housing", self.slider)
        self.slider_origin = Vector((-0.162, -0.132, 0.003))
        for i, dimensions, position in [
            (1, (0.010, 0.006, 0.001), (0, 0, 0.001)),
            (2, (0.010, 0.001, 0.004), (0, -0.0025, -0.001)),
            (3, (0.010, 0.001, 0.004), (0, 0.0025, -0.001)),
            (4, (0.001, 0.002, 0.0055), (-0.0035, -0.001, 0.00375)),
            (5, (0.001, 0.002, 0.0055), (0.0035, -0.001, 0.00375)),
        ]:
            box(slider_part.name + f".visual{i}", dimensions, position, silver, slider_part)
        self.puller = body("document_bag.puller", self.root)
        ring_part = part("document_bag.puller.loop", self.puller)
        self.puller_origin = Vector((-0.162, -0.134, 0.0085))
        for i, dimensions, position in [
            (1, (0.003, 0.012, 0.0015), (-0.0075, -0.008, 0)),
            (2, (0.003, 0.012, 0.0015), (0.0075, -0.008, 0)),
            (3, (0.012, 0.003, 0.0015), (0, -0.0035, 0)),
            (4, (0.012, 0.003, 0.0015), (0, -0.0125, 0)),
            (5, (0.004, 0.003, 0.0015), (0, -0.001, 0)),
        ]:
            box(ring_part.name + f".visual{i}", dimensions, position, blue, ring_part)
        self.slider.location = self.slider_origin
        self.puller.location = self.puller_origin
        # Ring rotation is a native hinge; unzip is a geometry control.
        self.hinge = joint("document_bag.puller_swing", "HINGE", self.slider, self.puller,
              tuple(self.puller_origin), (-math.pi / 3, math.pi / 3),
              rotation=(0, math.pi / 2, 0))
        self.hinge.parent = self.root
        scene.rigidbody_world.enabled = False
        for obj, points, offsets in self.dynamic:
            obj["geometry_samples"] = json.dumps(points)
            obj["geometry_offsets"] = json.dumps(offsets)
        apply_geometry_pose({"document_bag.unzip": 0, "document_bag.bulge": 0})

    def panel(self, parent, xs, ys, surface, collisions):
        samples = [(x, y) for y in ys for x in xs]
        n = len(samples)
        triangles = []
        faces = []
        for j in range(len(ys) - 1):
            for i in range(len(xs) - 1):
                a = j * len(xs) + i
                for ids in [(a, a + 1, a + len(xs) + 1),
                            (a, a + len(xs) + 1, a + len(xs))]:
                    triangles.append(ids)
                    faces.extend([tuple(reversed(ids)), tuple(k + n for k in ids)])
        perimeter = (list(range(len(xs)))
                     + [j * len(xs) + len(xs) - 1 for j in range(1, len(ys))]
                     + list(range(n - 2, n - len(xs) - 1, -1))
                     + [j * len(xs) for j in range(len(ys) - 2, 0, -1)])
        faces.extend((a, b, b + n, a + n)
                     for a, b in zip(perimeter, perimeter[1:] + perimeter[:1]))
        vertices = [(x, y, THICKNESS + extra)
                    for extra in (0, THICKNESS) for x, y in samples]
        obj = mesh(parent.name + ".visual", vertices, faces, parent, surface)
        for polygon in obj.data.polygons:
            polygon.use_smooth = True
        obj.data.set_sharp_from_angle(angle=.8)
        self.dynamic.append((obj, samples * 2, [0] * n + [THICKNESS] * n))
        if collisions:
            for i, ids in enumerate(triangles, 1):
                points = [samples[k] for k in ids] * 2
                proxy = mesh(parent.name + f".collision{i}",
                             [(x, y, THICKNESS + extra) for extra in (0, THICKNESS)
                              for x, y in [samples[k] for k in ids]],
                             PRISM_FACES, parent, shape="CONVEX_HULL")
                self.dynamic.append((proxy, points, [0] * 3 + [THICKNESS] * 3))
        return obj

def build():
    bag = BagGeometry()
    bpy.context.scene["geometry_controls"] = json.dumps({
        "document_bag.unzip": {"owner": "document_bag.film", "limits": [0, 0.324], "unit": "m"},
        "document_bag.bulge": {"owner": "document_bag.film", "limits": [0, 1], "unit": "1"},
    })
    # Protect ordinary object editing while controls and native joints continue
    # to drive their owner roots. Move the complete bag through its common root.
    for obj in bag.root.children_recursive:
        if obj.get("semantic_part") or obj.get("geometry_role"):
            obj.lock_location = obj.lock_rotation = obj.lock_scale = (True, True, True)
        if obj.get("geometry_role") == "collision":
            obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    bag.root.select_set(True)
    bpy.context.view_layer.objects.active = bag.root


def apply_geometry_pose(controls):
    """Set explicit visual/collision vertices and hardware from absolute controls.

    Sampling coordinates live in the blend; reloading does not need the build
    instance. The public preview applies native hinge rotation afterwards.
    """
    q, bulge = controls["document_bag.unzip"], controls["document_bag.bulge"]
    xs = dict(zip(X_GRID, pose_x_grid(q)))
    for obj in bpy.context.scene.objects:
        if "geometry_samples" not in obj:
            continue
        points = json.loads(obj["geometry_samples"])
        offsets = json.loads(obj["geometry_offsets"])
        for vertex, (x, y), extra in zip(obj.data.vertices, points, offsets):
            x = xs[x]
            vertex.co = (x, y, THICKNESS + clearance(x, y, q, bulge) + extra)
        obj.data.update()
    bpy.data.objects["document_bag.slider"].location = (-0.162 + q, -0.132, 0.003)
    bpy.data.objects["document_bag.puller"].location = (-0.162 + q, -0.134, 0.0085)
    bpy.data.objects["document_bag.puller_swing"].location = (-0.162 + q, -0.134, 0.0085)
    bpy.context.view_layer.update()


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
