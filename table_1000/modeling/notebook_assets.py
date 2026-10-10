"""Closed pocket notebooks: one compound body, with fixed covers and strap.

Photo-guided dimensions are estimates. Covers, page-block footprint, spine,
strap position and colors reuse the existing scene-two parametric geometry.
The closed stack removes the old cloth snapshot gaps; the strap is a complete
fixed loop. This chapter-one model has no opening or page-turning mechanism.
"""

from pathlib import Path
import shutil

import bpy

from asset_builders import asset, body, box, generate, material


PROFILES = {
    "black": ((0.009, 0.009, 0.010), (0.028, 0.030, 0.035)),
    "navy": ((0.007, 0.012, 0.045), (0.019, 0.028, 0.058)),
}


def build_notebook(color):
    cover_color, strap_color = PROFILES[color]
    cover = material("Cover", cover_color, 0.73)
    paper = material("Paper", (0.89, 0.87, 0.78), 0.84)
    strap = material("Woven strap", strap_color, 0.85)
    root = asset(f"Closed notebook {color}", "bottom center of the closed strap loop",
                 "+X toward fore-edge; +Y along spine; +Z toward top cover",
                 "short page edge faces -Y; spine is -X")
    root.empty_display_type = "CUBE"
    root.empty_display_size = 0.075
    notebook = body("notebook", root)
    notebook["model_scope"] = "closed storage model; no page turning or elastic simulation"
    # 0.7 mm strap + 2.2 mm cover + 15.6 mm pages + cover + strap = 21.4 mm.
    # The page block includes the old fixed top leaf, with no cloth or air gap.
    box("Bottom cover", (0.103, 0.143, 0.0022), (0, 0, 0.0018),
        cover, notebook, bevel=0.0006)
    box("Top cover", (0.103, 0.143, 0.0022), (0, 0, 0.0196),
        cover, notebook, bevel=0.0006)
    box("Bound pages", (0.097, 0.136, 0.0156), (0.001, 0, 0.0107),
        paper, notebook, bevel=0.0003)
    box("Bound spine", (0.004, 0.140, 0.018), (-0.049, 0, 0.0107),
        cover, notebook, bevel=0.0005)
    for name, z in (("Bottom strap", 0.00035), ("Top strap", 0.02105)):
        box(name, (0.005, 0.143, 0.0007), (0.035, 0, z),
            strap, notebook, bevel=0.0001)
    for name, y in (("Front strap", -0.07115), ("Back strap", 0.07115)):
        box(name, (0.005, 0.0007, 0.020), (0.035, y, 0.0107),
            strap, notebook, bevel=0.0001)
    # Sub-millimetre page marks are visual only; the block provides contact.
    edge = material("Page edge lines", (0.65, 0.63, 0.56), 0.9)
    for i in range(15):
        box(f"Page edge {i:02d}", (0.00008, 0.134, 0.00008),
            (0.0495, 0, 0.0035 + i * 0.001), edge, notebook, collision=False)
    # Selecting the asset frame or compound root moves all contact geometry.
    # Decorative pieces cannot be picked and dragged away independently.
    for obj in notebook.children:
        obj.hide_select = True
    bpy.context.view_layer.objects.active = root
    root.select_set(True)


def generate_notebook(color, source_file, argv=None):
    generate(lambda: build_notebook(color), source_file, argv)
    # generate() copies the generic helper; also retain this shared generator.
    output = Path(bpy.data.filepath).parent
    original = Path(__file__).resolve()
    target = output / original.name
    if original != target:
        shutil.copyfile(original, target)
