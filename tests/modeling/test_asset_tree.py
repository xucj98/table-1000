"""Small host-Python checks for actual parenting and conservative display grouping."""

from types import SimpleNamespace
import unittest

from table_1000.modeling.asset_tree import asset_forest, format_tree


class Object(dict):
    def __init__(self, name, parent=None, role=None, rigid=False, material="oak", shape=None, kind="MESH"):
        super().__init__(geometry_role=role, rigid_body_root=rigid)
        self.name, self.parent, self.type = name, parent, kind
        self.rigid_body_constraint = None
        self.rigid_body = SimpleNamespace(collision_shape=shape) if shape else None
        self.material_slots = [SimpleNamespace(material=SimpleNamespace(name=material))] if material else []


class AssetTreeTests(unittest.TestCase):
    def test_actual_parent_forest_and_auxiliary_filter(self):
        body = Object("body", rigid=True)
        cap = Object("cap", rigid=True)
        tip = Object("invented.path.tip", body, "visual")
        camera = Object("Camera", kind="CAMERA")
        light = Object("Light", kind="LIGHT")
        joint = Object("slider", kind="EMPTY")
        joint.rigid_body_constraint = object()
        forest = asset_forest([tip, cap, body, camera, light, joint])
        self.assertEqual(format_tree(forest), "body [rigid]\ncap [rigid]\n")
        self.assertEqual(format_tree(forest, geometry=True), "body [rigid]\n  invented.path.tip\ncap [rigid]\n")

    def test_contiguous_numbered_subtrees(self):
        cabinet = Object("cabinet", kind="EMPTY")
        objects = [cabinet]
        for i in (1, 2, 3):
            drawer = Object(f"cabinet.drawer{i}", cabinet, rigid=True)
            handle = Object(drawer.name + ".handle", drawer, shape="COMPOUND")
            handle['semantic_part'] = True
            objects += [drawer, handle, Object(handle.name + ".visual", handle, "visual")]
        forest = asset_forest(objects)
        self.assertEqual(format_tree(forest, True),
                         "cabinet\n  drawer1..3 [rigid]\n    handle\n")
        self.assertIn("cabinet.drawer2.handle", format_tree(forest, full_names=True))
        self.assertEqual([obj.name for obj in objects[1:4]],
                         ["cabinet.drawer1", "cabinet.drawer1.handle", "cabinet.drawer1.handle.visual"])
        sectors = [Object(f"sector_{i:02}", cabinet, "collision", shape="CONVEX_HULL") for i in range(16)]
        self.assertIn("sector_00..15", format_tree(asset_forest([cabinet] + sectors), True, geometry=True))

    def test_different_identity_and_missing_numbers_stay_separate(self):
        root = Object("root", kind="EMPTY")
        objects = [root]
        variants = [
            Object("part1", root, "visual"), Object("part3", root, "visual"),
            Object("part.001", root, "visual"), Object("part.002", root, "visual"),
            Object("paint1", root, "visual"), Object("paint2", root, "visual", material="white"),
            Object("Collision_piece1", root, "collision", shape="BOX"),
            Object("Collision_piece2", root, "collision", shape="CONVEX_HULL"),
            Object("piece1", root, "visual"), Object("piece2", root, "collision"),
            Object("branch1", root, "visual"), Object("branch2", root, "visual"),
        ]
        objects += variants + [Object("branch2.extra", variants[-1], "visual")]
        self.assertNotIn("..", format_tree(asset_forest(objects), True, geometry=True))
        for i, color in [(1, "oak"), (2, "white")]:
            part = Object(f"root.component{i}", root, material=None, shape="COMPOUND")
            part['semantic_part'] = True
            objects += [part, Object(part.name + ".visual", part, "visual", material=color)]
        self.assertNotIn("..", format_tree(asset_forest(objects), True))


if __name__ == "__main__":
    unittest.main()
