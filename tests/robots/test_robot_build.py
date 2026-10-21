"""USD build regression tests; run inside Isaac Python after AppLauncher."""

import pytest

from table_1000.robots.build import author_effort_limits


def test_urdf_efforts_override_importer_limits(tmp_path):
    Usd = pytest.importorskip('pxr.Usd')
    from pxr import UsdPhysics

    imported = tmp_path / 'import.usda'
    stage = Usd.Stage.CreateNew(str(imported))
    for name, joint_type, axis in (
            ('finger', UsdPhysics.RevoluteJoint, 'angular'),
            ('base_x', UsdPhysics.PrismaticJoint, 'linear')):
        joint = joint_type.Define(stage, '/' + name)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis).CreateMaxForceAttr(1e6)
    stage.GetRootLayer().Save()
    output = tmp_path / 'robot.usda'
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = ['import.usda']
    stage.GetRootLayer().Save()
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('''<robot name="test">
      <joint name="finger" type="revolute"><limit effort="20"/></joint>
      <joint name="base_x" type="prismatic"><limit effort="116"/></joint>
      <joint name="fixed" type="fixed"/>
    </robot>''')

    author_effort_limits(urdf, output)

    for name, axis, effort in (('finger', 'angular', 20.), ('base_x', 'linear', 116.)):
        assert UsdPhysics.DriveAPI(stage.GetPrimAtPath('/' + name), axis).GetMaxForceAttr().Get() == effort
        # The override belongs to the final layer, not the imported asset.
        attr = stage.GetRootLayer().GetAttributeAtPath(f'/{name}.drive:{axis}:physics:maxForce')
        assert attr.default == effort
