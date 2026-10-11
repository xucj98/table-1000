"""Build the wooden desktop rack and its two independently sliding drawers."""
import math

import bpy

from table_1000.modeling.asset_builders import asset, body, box, collider, cylinder, generate, joint, material, mesh, group_part


def wood_material():
    surface = material("Warm oak", (0.32, 0.21, 0.10), 0.52)
    nodes, links = surface.node_tree.nodes, surface.node_tree.links
    coordinates = nodes.new("ShaderNodeTexCoord")
    scale = nodes.new("ShaderNodeVectorMath")
    scale.operation = "MULTIPLY"
    scale.inputs[1].default_value = (4, 95, 75)
    noise = nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 3
    noise.inputs["Detail"].default_value = 1
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.24, 0.15, 0.065, 1)
    ramp.color_ramp.elements[1].color = (0.39, 0.28, 0.15, 1)
    links.new(coordinates.outputs["Object"], scale.inputs[0])
    links.new(scale.outputs["Vector"], noise.inputs["Vector"])
    links.new(noise.outputs["Fac"], ramp.inputs[0])
    links.new(ramp.outputs["Color"], nodes.get("Principled BSDF").inputs["Base Color"])
    return surface


def side_panel(name, x, surface, parent):
    contour = [(-.170, 0), (.180, 0), (.180, .435), (-.160, .435)]
    for degrees in (120, 150, 180):
        angle = math.radians(degrees)
        contour.append((-.160 + .010 * math.cos(angle), .425 + .010 * math.sin(angle)))
    n = len(contour)
    vertices = [(xx, y, z) for xx in (x - .007, x + .007) for y, z in contour]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)]
    collider(mesh(name, vertices, faces, parent, surface))


def build():
    wood = wood_material()
    inner = material("Drawer oak interior", (.37, .27, .14), .58)
    white = material("White drawer face", (.83, .84, .81), .42)
    root = asset("cabinet/000001", "bottom center", "+X width; +Y back; +Z up", "drawer fronts at -Y")
    housing = body("wooden_rack", root)
    for x, name in [(-.430, "Left cheek"), (.137, "Central divider"), (.430, "Right cheek")]:
        side_panel(name, x, wood, housing)
    for name, dimensions, position in [
        ("Left raised shelf", (.560, .350, .015), (-.147, .005, .149)),
        ("Right open cubby floor", (.280, .350, .0168), (.2835, .005, .2671)),
        ("Right cabinet bottom", (.280, .350, .013), (.2835, .005, .008)),
        ("Drawer divider shelf", (.280, .337, .012), (.2835, .009, .1351)),
        ("Left upper rear rail", (.560, .013, .1606666667), (-.147, .177, .3481666667)),
        ("Left lower rear board", (.560, .013, .1415), (-.147, .177, .07075)),
        ("Right upper rear rail", (.280, .013, .141), (.2835, .177, .358)),
        ("Drawer cabinet back", (.280, .008, .252), (.2835, .177, .1339)),
    ]:
        box(name, dimensions, position, wood, housing)
    # Scale the complete drawer bay vertically by 1.2 about the bottom's top (14.5 mm).
    for z in (.0667, .1951):
        for x in (.151, .416):
            box("Fixed side guide", (.009, .283, .0096), (x, .005, z), inner, housing)
    for i, z in [(1, .1969), (2, .0685)]:
        drawer = body(f"drawer{i}", root)
        drawer.location = (.2835, -.163, z)
        for name, dimensions, position, surface in [
            ("White drawer front", (.271, .012, .108), (0, 0, 0), white),
            ("Drawer frame behind face", (.267, .010, .1056), (0, .011, 0), wood),
            ("Open drawer bottom", (.252, .285, .0084), (0, .154, -.0492), inner),
            ("Drawer left wall", (.008, .279, .0996), (-.122, .154, -.0024), inner),
            ("Drawer right wall", (.008, .279, .0996), (.122, .154, -.0024), inner),
            ("Drawer rear wall", (.252, .008, .0996), (0, .298, -.0024), inner),
        ]:
            box(name, dimensions, position, surface, drawer)
        cylinder("Oak knob neck", .010, .014, (0, -.013, 0), wood, drawer, axis="Y")
        cylinder("Oak knob", .015, .015, (0, -.025, 0), wood, drawer,
                 axis="Y", radius_end=.013)
        joint(f"drawer{i}_open", "SLIDER", housing, drawer, (.2835, -.163, z),
              (0, .208), rotation=(0, 0, -math.pi / 2))

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    root = bpy.data.objects['cabinet/000001']
    root.name = 'cabinet'
    root["asset_id"] = 'cabinet/000001'
    owner = bpy.data.objects['wooden_rack']
    owner.name = 'cabinet.housing'
    for component, names in [
        ('side1', ['Left cheek']),
        ('divider', ['Central divider']),
        ('side2', ['Right cheek']),
        ('raised_shelf', ['Left raised shelf']),
        ('cubby_floor', ['Right open cubby floor']),
        ('bottom', ['Right cabinet bottom']),
        ('drawer_shelf', ['Drawer divider shelf']),
        ('left_upper_rail', ['Left upper rear rail']),
        ('left_lower_back', ['Left lower rear board']),
        ('right_upper_rail', ['Right upper rear rail']),
        ('drawer_back', ['Drawer cabinet back']),
        ('guide1', ['Fixed side guide']),
        ('guide2', ['Fixed side guide.001']),
        ('guide3', ['Fixed side guide.002']),
        ('guide4', ['Fixed side guide.003']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)
    owner = bpy.data.objects['drawer1']
    owner.name = 'cabinet.drawer1'
    for component, names in [
        ('front', ['White drawer front']),
        ('frame', ['Drawer frame behind face']),
        ('bottom', ['Open drawer bottom']),
        ('side1', ['Drawer left wall']),
        ('side2', ['Drawer right wall']),
        ('rear', ['Drawer rear wall']),
        ('handle', ['Oak knob neck', 'Oak knob']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)
    owner = bpy.data.objects['drawer2']
    owner.name = 'cabinet.drawer2'
    for component, names in [
        ('front', ['White drawer front.001']),
        ('frame', ['Drawer frame behind face.001']),
        ('bottom', ['Open drawer bottom.001']),
        ('side1', ['Drawer left wall.001']),
        ('side2', ['Drawer right wall.001']),
        ('rear', ['Drawer rear wall.001']),
        ('handle', ['Oak knob neck.001', 'Oak knob.001']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
