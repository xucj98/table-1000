"""Choices for turning the vendor URDF package into this robot source.

Read by ``scripts/robots/prepare_robot.py x1pro_rm001``. The raw package is the
one of x1pro (``ex001_render``, see ../x1pro/README.md); this robot takes its
rm001_g1_dvt1 variant and otherwise makes the same choices. Meshes that x1pro
uses too are referenced from ../x1pro/meshes/; only the gripper's are written
here.
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'prepare')

BASE = 'x1pro'
# dvt1 chassis, hy_1.0 arms and the rm001_g1_dvt1 gripper with pivoting fingers.
RAW = _X1PRO.RAW
URDF = 'urdf/ex001_robot_rm001_g1_dvt1.urdf'
PIECES = _X1PRO.PIECES
# Each finger's inner face is a concave arc along its length (x): one hull or CoACD's pieces bridge
# it, up to 5 mm off the face, and a held object rests on the bridge; 8 slabs follow it to 0.4 mm.
SLABS = {f'{side}_arm_gripper_{finger}_link': (0, 8) for side in ('left', 'right') for finger in ('left', 'right')}
NO_COLLISION = _X1PRO.NO_COLLISION
edit = _X1PRO.edit
