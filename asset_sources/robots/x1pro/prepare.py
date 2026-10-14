"""Choices for turning the vendor URDF package into this robot source.

Read by ``scripts/assets/prepare_robot.py x1pro``. The raw package
(``ex001_render``, see README) is pulled into ``assets/`` with
``scripts/assets/remote.py pull robots/x1pro/raw/ex001_render``.
"""

# Package directory under assets/, and the URDF variant inside it: dvt1 chassis,
# hy_1.0 arms and the fx001_h_evt1 parallel gripper.
RAW = 'robots/x1pro/raw/ex001_render'
URDF = 'urdf/ex001_robot_fx001_h_evt1.urdf'

# Largest distance (m, 99th percentile) the collision surface may stand off the
# visual; pieces are added until it is met. Hands touch objects on purpose and
# get the tightest fit.
EXCESS = {'gripper': .003, '_arm_link': .010, '_arm_base_link': .010}
DEFAULT_EXCESS = .015


def excess(link):
    matches = [value for key, value in EXCESS.items() if key in link]
    return min(matches) if matches else DEFAULT_EXCESS


# Planes cutting parts of a visual mesh out of its collision, as (normal,
# offset) in the mesh frame, keeping normal . x >= offset: the bracket on the
# back of the torso runs inside the lift column and never meets anything else.
COLLISION_CLIP = {'lift_link': [((1., 0., 0.), .075)]}

# Links inside the gripper base; they get no collision.
NO_COLLISION = ('arm_link6', 'motor_link')


def edit(root):
    """Joint changes over the vendor URDF (listed in README)."""
    for joint in root.findall('joint'):
        # Fingers are driven directly in metres; the virtual motor joint becomes fixed.
        if joint.get('name').endswith('_arm_gripper'):
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit'):
                joint.remove(joint.find(tag))
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)
        if joint.get('name') == 'lift_joint':
            joint.find('limit').attrib.update(effort='2000', velocity='0.2')
