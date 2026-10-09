"""Build the wooden desktop rack and its two independently sliding drawers."""
from pathlib import Path
import math
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, box, collider, cylinder, generate, joint, material, mesh


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
    contour = [(-.170, 0), (.180, 0), (.180, .435), (-.058, .435)]
    for i in range(1, 5):
        t = i / 4
        contour.append((-.170 + .112 * (1 - t), .435 - .132 * t + .018 * math.sin(math.pi * t)))
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
        ("Right open cubby floor", (.280, .350, .014), (.2835, .005, .225)),
        ("Right cabinet bottom", (.280, .350, .013), (.2835, .005, .008)),
        ("Drawer divider shelf", (.280, .337, .010), (.2835, .009, .115)),
        ("Left upper rear rail", (.560, .013, .105), (-.147, .177, .376)),
        ("Right upper rear rail", (.280, .013, .105), (.2835, .177, .376)),
        ("Right cubby back", (.280, .009, .119), (.2835, .178, .279)),
        ("Drawer cabinet back", (.280, .008, .210), (.2835, .177, .114)),
    ]:
        box(name, dimensions, position, wood, housing)
    for z in (.058, .165):
        for x in (.151, .416):
            box("Fixed side guide", (.009, .283, .008), (x, .005, z), inner, housing)
    for i, z in [(1, .1665), (2, .0595)]:
        drawer = body(f"drawer{i}", root)
        drawer.location = (.2835, -.163, z)
        for name, dimensions, position, surface in [
            ("White drawer front", (.271, .012, .090), (0, 0, 0), white),
            ("Drawer frame behind face", (.267, .010, .088), (0, .011, 0), wood),
            ("Open drawer bottom", (.252, .285, .007), (0, .154, -.041), inner),
            ("Drawer left wall", (.008, .279, .083), (-.122, .154, -.002), inner),
            ("Drawer right wall", (.008, .279, .083), (.122, .154, -.002), inner),
            ("Drawer rear wall", (.252, .008, .083), (0, .298, -.002), inner),
        ]:
            box(name, dimensions, position, surface, drawer)
        cylinder("Oak knob neck", .010, .014, (0, -.013, 0), wood, drawer, axis="Y")
        cylinder("Oak knob", .015, .015, (0, -.025, 0), wood, drawer,
                 axis="Y", radius_end=.013)
        joint(f"drawer{i}_open", "SLIDER", housing, drawer, (.2835, -.163, z),
              (0, .208), rotation=(0, 0, -math.pi / 2))


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
