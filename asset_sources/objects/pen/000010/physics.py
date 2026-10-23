"""Native cap detent masses, contact materials and spring drives.

Engineering estimates; effective follower mass is an inertia proxy.
COM and inertia are estimated by PhysX from collision geometry.
"""
from table_1000.physics.pen_assets import author as author_pen

MASSES = {'body': 0.007, 'cap': 0.002}
IS_CLICK = False
METAL_PARTS = ('body.steel_point',)
RUBBER_PARTS = ()


def author(geometry_path, output_path):
    from math import atan2, cos, sin
    from pathlib import Path
    from pxr import Gf, Usd, UsdPhysics

    layer = author_pen(geometry_path, output_path, masses=MASSES,
                       metal_parts=METAL_PARTS, rubber_parts=RUBBER_PARTS,
                       articulation=IS_CLICK, detent=DETENT)
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(Path(geometry_path).resolve())]
    stage.SetEditTarget(layer)
    # The free radial coordinate is unchanged. Keep the pen's longitudinal X
    # direction exactly on a locked joint axis instead of mixing it into the
    # shortest-arc frame. This reduces GPU PGS drift at the preloaded stop.
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.PrismaticJoint):
            continue
        joint = UsdPhysics.PrismaticJoint(prim)
        anchor = joint.GetLocalPos0Attr().Get()
        angle = atan2(anchor[2], anchor[1])
        frame = Gf.Quatf(cos(angle / 2), Gf.Vec3f(sin(angle / 2), 0, 0))
        joint.CreateAxisAttr('Y')
        joint.GetLocalRot0Attr().Set(frame)
        joint.GetLocalRot1Attr().Set(frame)
    layer.Save()
    return layer

DETENT = {'seat': -0.0275, 'neck': 0.0038, 'segments': 12, 'balls': 3, 'radius': 0.00035, 'height': 0.000196875, 'travel': 0.00035, 'mass': 8.3740234375e-05, 'stiffness': 765.625, 'damping': 0.13125, 'preload': 0.00013125, 'flat': False, 'dt': 0.001}
