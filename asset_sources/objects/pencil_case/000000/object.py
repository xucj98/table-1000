"""Black single-compartment case: approved stage-one rigid-lip approximation.

Lower shell is rigid. Retract the slider before opening both complete lips;
preview keyframes coordinate these independent joints, without an interlock.
No cloth deformation, zipper force, tooth engagement or dynamics are modeled.
"""

import math

import bpy

from table_1000.modeling.asset_builders import (
    asset, body, box, collider, generate, joint, material, mesh, part, tube,
)

PROFILE = {
    'dimensions_m': (0.240, 0.070, 0.075),
    'slider_travel_m': 0.216,
    'lip_angle_rad': math.pi / 4,
    'ring_outer_radius_m': 0.011,
    'ring_inner_radius_m': 0.009,
    'ring_segments': 16,
}


def visual_box(component, index, dimensions, position, surface, collision=True, bevel=0):
    visual = box(f'{component.name}.visual{index}', dimensions, position, surface,
                 component, collision=False, bevel=bevel)
    if collision:
        proxy = collider(visual, 'BOX')
        proxy.name = f'{component.name}.collision{index}'
    return visual


def prism(component, index, xmin, xmax, section, surface, collision=True):
    vertices = [(x, y, z) for x in (xmin, xmax) for y, z in section]
    n = len(section)
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)]
    visual = mesh(f'{component.name}.visual{index}', vertices, faces, component, surface)
    if collision:
        proxy = collider(visual)
        proxy.name = f'{component.name}.collision{index}'
    return visual


def ribbon(component, index, corners, surface):
    return mesh(f'{component.name}.visual{index}', corners, [(0, 1, 2, 3)], component, surface)


def frame_origin(owner, location):
    """Give a body its functional pivot, preserving all authored world geometry."""
    bpy.context.view_layer.update()
    saved = [(child, child.matrix_world.copy()) for child in owner.children]
    owner.location = location
    bpy.context.view_layer.update()
    for child, world in saved:
        child.matrix_world = world
    bpy.context.view_layer.update()


def build():
    cloth = material('pencil_case.cloth', (0.013, 0.018, 0.024), roughness=0.83)
    nodes = cloth.node_tree.nodes
    noise = nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 420
    noise.inputs['Detail'].default_value = 1
    bump = nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.22
    bump.inputs['Distance'].default_value = 0.00008
    cloth.node_tree.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    cloth.node_tree.links.new(bump.outputs['Normal'], nodes.get('Principled BSDF').inputs['Normal'])
    lining = material('pencil_case.lining', (0.033, 0.038, 0.044), roughness=0.92)
    binding = material('pencil_case.binding', (0.007, 0.010, 0.014), roughness=0.75)
    chain = material('pencil_case.zipper_tape', (0.040, 0.046, 0.052), roughness=0.69)
    steel = material('pencil_case.metal', (0.22, 0.24, 0.27), roughness=0.3, metallic=0.85)
    cord = material('pencil_case.white_cord', (0.72, 0.73, 0.68), roughness=0.9)
    patch = material('pencil_case.label', (0.085, 0.09, 0.10), roughness=0.85)

    root = asset('pencil_case', 'bottom-center of the lower shell',
                 '+X length and opening slider travel; +Y width; +Z up', 'front cloth at -Y')
    housing = body('pencil_case.housing', root)
    outer = part('pencil_case.housing.outer_cloth', housing)
    # BOX proxies preserve the cavity: bottom, two side walls and two end walls.
    for index, dimensions, center in [
        (1, (0.240, 0.070, 0.003), (0, 0, 0.0015)),
        (2, (0.230, 0.003, 0.051), (0, -0.0335, 0.0285)),
        (3, (0.230, 0.003, 0.051), (0, 0.0335, 0.0285)),
        (4, (0.005, 0.070, 0.051), (-0.1175, 0, 0.0285)),
        (5, (0.005, 0.070, 0.051), (0.1175, 0, 0.0285)),
    ]:
        visual_box(outer, index, dimensions, center, cloth, bevel=0.0008)
    # Short fixed end roofs keep slider hardware outside the active lips at +/-108 mm.
    roof = [(-0.035, 0.054), (0.035, 0.054), (0, 0.075)]
    prism(outer, 6, -0.120, -0.105, roof, cloth)
    prism(outer, 7, 0.105, 0.120, roof, cloth)
    liner = part('pencil_case.housing.lining', housing)
    for index, corners in enumerate([
        [(-0.115, -0.032, 0.003), (0.115, -0.032, 0.003), (0.115, 0.032, 0.003), (-0.115, 0.032, 0.003)],
        [(-0.115, -0.032, 0.003), (-0.115, -0.032, 0.0538), (0.115, -0.032, 0.0538), (0.115, -0.032, 0.003)],
        [(-0.115, 0.032, 0.003), (0.115, 0.032, 0.003), (0.115, 0.032, 0.0538), (-0.115, 0.032, 0.0538)],
        [(-0.115, -0.032, 0.003), (-0.115, 0.032, 0.003), (-0.115, 0.032, 0.0538), (-0.115, -0.032, 0.0538)],
        [(0.115, -0.032, 0.003), (0.115, -0.032, 0.0538), (0.115, 0.032, 0.0538), (0.115, 0.032, 0.003)],
    ], 1):
        ribbon(liner, index, corners, lining)
    edging = part('pencil_case.housing.binding', housing)
    for index, y in enumerate((-0.0348, 0.0348), 1):
        visual_box(edging, index, (0.232, 0.00035, 0.0015), (0, y, 0.006), binding, collision=False)
    label = part('pencil_case.housing.label', housing)
    visual_box(label, 1, (0.028, 0.00015, 0.009), (0.060, -0.035, 0.036), patch, collision=False)

    for side, sign in [('front', -1), ('back', 1)]:
        lip = body(f'pencil_case.{side}_lip', root)
        fabric = part(lip.name + '.cloth', lip)
        # 2.5-mm lip at the seam; clearance remains >44 mm at the open endpoint.
        section = [(sign * 0.035, 0.055), (sign * 0.0001, 0.075),
                   (sign * 0.0001, 0.0725), (sign * 0.032, 0.055)]
        prism(fabric, 1, -0.105, 0.105, section, cloth)
        underside = part(lip.name + '.lining', lip)
        ribbon(underside, 1, [(-0.1048, sign * 0.0319, 0.055),
                             (0.1048, sign * 0.0319, 0.055),
                             (0.1048, sign * 0.0002, 0.0724),
                             (-0.1048, sign * 0.0002, 0.0724)], lining)
        tape = part(lip.name + '.zipper_tape', lip)
        ribbon(tape, 1, [(-0.105, sign * 0.0038, 0.07298),
                        (0.105, sign * 0.0038, 0.07298),
                        (0.105, sign * 0.0001, 0.07504),
                        (-0.105, sign * 0.0001, 0.07504)], chain)
        rim = part(lip.name + '.rim', lip)
        visual_box(rim, 1, (0.210, 0.0012, 0.001), (0, sign * 0.0334, 0.0555), binding, collision=False)
        frame_origin(lip, (0, sign * 0.0335, 0.055))
        limits = (0, PROFILE['lip_angle_rad']) if sign == -1 else (-PROFILE['lip_angle_rad'], 0)
        joint('pencil_case.' + side + '_lip_open', 'HINGE', housing, lip,
              (0, sign * 0.0335, 0.055), limits, rotation=(0, math.pi / 2, 0))

    slider = body('pencil_case.slider', root)
    carriage = part('pencil_case.slider.carriage', slider)
    visual_box(carriage, 1, (0.005, 0.006, 0.0032), (-0.108, 0, 0.0773), steel, bevel=0.0003)
    lug = part('pencil_case.slider.attachment', slider)
    # Convex bridge above the closed roof; the parked bridge lies beyond the moving lips.
    section = [(0, 0.0785), (-0.036, 0.064), (-0.036, 0.0628), (0, 0.0773)]
    prism(lug, 1, -0.1095, -0.1065, section, steel)
    visual_box(lug, 2, (0.002, 0.004, 0.0012), (-0.108, -0.036, 0.0627), steel, collision=False)
    string = part('pencil_case.slider.white_cord', slider)
    data = bpy.data.curves.new(string.name + '.curve', 'CURVE')
    data.dimensions = '3D'
    data.bevel_depth = 0.00055
    data.bevel_resolution = 1
    spline = data.splines.new('POLY')
    points = [(-0.108, -0.001, 0.079), (-0.108, -0.001, 0.083),
              (-0.106, -0.004, 0.081), (-0.108, -0.008, 0.079)]
    spline.points.add(len(points) - 1)
    for point, co in zip(spline.points, points):
        point.co = (*co, 1)
    obj = bpy.data.objects.new(string.name + '.visual1', data)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = string
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.convert(target='MESH')
    obj.select_set(False)
    obj['geometry_role'] = 'visual'
    obj.data.materials.append(cord)
    frame_origin(slider, (-0.108, 0, 0.0773))
    joint('pencil_case.unzip', 'SLIDER', housing, slider, (-0.108, 0, 0.0773),
          (0, PROFILE['slider_travel_m']))

    ring = body('pencil_case.pull_ring', root)
    loop = part('pencil_case.pull_ring.ring', ring)
    tube(loop.name + '.visual1', PROFILE['ring_outer_radius_m'],
                  PROFILE['ring_inner_radius_m'], 0.002, (-0.108, -0.038, 0.0511),
                  steel, loop, axis='Y', segments=PROFILE['ring_segments'])
    proxies = sorted((o for o in loop.children if o.get('geometry_role') == 'collision'), key=lambda o: o.name)
    for index, proxy in enumerate(proxies, 1):
        proxy.name = f'{loop.name}.collision{index}'
    frame_origin(ring, (-0.108, -0.038, 0.0621))
    joint('pencil_case.ring_swing', 'HINGE', slider, ring, (-0.108, -0.038, 0.0621),
          (-math.pi / 2, math.pi / 2), rotation=(-math.pi / 2, 0, 0))
    # Move the common asset root to place the complete mechanism and its anchors.
    bpy.context.view_layer.update()
    for constraint in [obj for obj in bpy.context.scene.objects if obj.get('preview_joint')]:
        world = constraint.matrix_world.copy()
        constraint.parent = root
        constraint.matrix_world = world
    bpy.ops.object.select_all(action='DESELECT')
    root.select_set(True)
    bpy.context.view_layer.objects.active = root


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == '__main__':
    main()
