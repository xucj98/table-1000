"""Export the actual Object.parent forest, optionally compacting numbered siblings.

Legacy assets without semantic parts show their organization and rigid roots.
Children display names relative to their actual parent; dots never add hierarchy.
Compact output retains an expanded file with full names for exact references.
Blender is only required by the command entry point.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import re
import sys


def asset_forest(objects):
    """Keep marked geometry, rigid roots and their actual ancestors."""
    objects = {obj.name: obj for obj in objects}
    retained = set()
    for obj in objects.values():
        if obj.get("geometry_role") not in {"visual", "collision"} and not obj.get("rigid_body_root") and not obj.get("semantic_part"):
            continue
        while obj is not None:
            if obj.type not in {"CAMERA", "LIGHT"} and not obj.rigid_body_constraint:
                retained.add(obj.name)
            obj = obj.parent
    children = defaultdict(list)
    for name in sorted(retained):
        obj = objects[name]
        parent = obj.parent.name if obj.parent and obj.parent.name in retained else None
        children[parent].append(obj)

    def branch(obj):
        return obj, [branch(child) for child in children[obj.name]]

    return [branch(obj) for obj in children[None]]


def _relative_name(name, parent):
    prefix = parent + "."
    return name[len(prefix):] if name.startswith(prefix) else name


def _signature(node):
    obj, children = node
    return (obj.type, bool(obj.get("rigid_body_root")), bool(obj.get("semantic_part")), obj.get("geometry_role"),
            obj.rigid_body.collision_shape if obj.rigid_body else None,
            tuple(slot.material.name if slot.material else None for slot in obj.material_slots),
            tuple(sorted((child[0].name.startswith(obj.name + "."),
                          _relative_name(child[0].name, obj.name), _signature(child)) for child in children)))


def _numbered(name):
    # Only a terminal integer attached to a name is explicit here.
    # A dot-number suffix (including Blender's .001) is never semantic.
    match = re.fullmatch(r"(.+?\D)([0-9]+)", name)
    if match and not match[1].endswith("."):
        return match[1], match[2]
    return None


def _siblings(nodes, compact):
    if not compact:
        return [(node, node[0].name) for node in nodes]
    groups = defaultdict(list)
    displayed = []
    for node in nodes:
        numbered = _numbered(node[0].name)
        if numbered is None:
            displayed.append((node, node[0].name))
        else:
            prefix, digits = numbered
            groups[(prefix, _signature(node))].append((int(digits), digits, node))
    for (prefix, _), members in groups.items():
        members.sort(key=lambda item: item[0])
        width = max((len(digits) for _, digits, _ in members if len(digits) > 1 and digits[0] == "0"), default=0)
        if any(digits != str(number).zfill(width) for number, digits, _ in members):
            displayed.extend((node, node[0].name) for _, _, node in members)
            continue
        runs = []
        for number, _, node in members:
            if not runs or number != runs[-1][-1][0] + 1:
                runs.append([])
            runs[-1].append((number, node))
        for run in runs:
            first, node = run[0]
            label = f"{prefix}{str(first).zfill(width)}..{str(run[-1][0]).zfill(width)}" if len(run) > 1 else node[0].name
            displayed.append((node, label))
    return sorted(displayed, key=lambda item: item[0][0].name)


def format_tree(forest, compact=False, geometry=False, full_names=False):
    """Render actual hierarchy and [rigid] labels without modifying objects."""
    lines = []

    def visible(node):
        obj, children = node
        return geometry or obj.get("rigid_body_root") or obj.get("semantic_part") or any(visible(child) for child in children)

    def visit(nodes, indent="", parent_name=None, displayed_parent=None):
        siblings = _siblings([node for node in nodes if visible(node)], compact)
        for node, name in siblings:
            obj, children = node
            if parent_name and name.startswith(parent_name + "."):
                name = (displayed_parent + name[len(parent_name):] if full_names
                        else name[len(parent_name) + 1:])
            marker = " [rigid]" if obj.get("rigid_body_root") else ""
            lines.append(indent + name + marker)
            visit(children, indent + "  ", obj.name, name)

    visit(forest)
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("blend", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--geometry", action="store_true",
                        help="Include visual/collision meshes; default shows organization, rigid roots and semantic parts")
    parser.add_argument("--compact", action="store_true",
                        help="Compact matching consecutive siblings; also write OUTPUT.stem.expanded.txt")
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)
    import bpy

    bpy.ops.wm.open_mainfile(filepath=str(args.blend.resolve()), load_ui=False)
    forest = asset_forest(bpy.context.scene.objects)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.compact:
        full_path = args.output.with_name(args.output.stem + ".expanded" + args.output.suffix)
        full_path.write_text(format_tree(forest, geometry=args.geometry, full_names=True), encoding="utf-8")
        text = ("# Compact display: only consecutive explicit integer siblings with matching\n"
                "# roles, collision types, materials and normalized child trees are merged.\n"
                "# Dot-number suffixes are preserved; dots never create parent nodes.\n"
                f"# View: {'geometry' if args.geometry else 'parts'}; expanded full names: {full_path.name}\n\n"
                + format_tree(forest, True, args.geometry))
    else:
        text = format_tree(forest, geometry=args.geometry)
    args.output.write_text(text, encoding="utf-8")
    print("ASSET_TREE_EXPORTED", args.output.resolve(), flush=True)
