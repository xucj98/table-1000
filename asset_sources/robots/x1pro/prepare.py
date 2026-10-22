"""Choices for turning the vendor URDF package into this robot source.

Read by ``scripts/robots/prepare_robot.py x1pro``. The raw package
(``ex001_render``, see README) is pulled into ``assets/`` with
``scripts/assets/remote.py pull robots/x1pro/raw/ex001_render``.
"""

# Package directory under assets/, and the URDF variant inside it: dvt1 chassis,
# hy_1.0 arms and the fx001_h_evt1 parallel gripper.
RAW = 'robots/x1pro/raw/ex001_render'
URDF = 'urdf/ex001_robot_fx001_h_evt1.urdf'

# Convex pieces per link (CoACD); links not listed get a single hull.
PIECES = {name: 4 for name in ['chassis_base_link', 'lift_link', 'head_yaw_link', 'head_pitch_link'] + [
    f'{side}_{name}' for side in ('left', 'right') for name in (
        'arm_base_link', 'arm_link1', 'arm_link2', 'arm_link3', 'arm_link4', 'arm_link5',
        'arm_gripper_base_link', 'arm_gripper_left_link', 'arm_gripper_right_link')]}
# Links inside the gripper base; they get no collision.
NO_COLLISION = ('arm_link6', 'motor_link')
# Flat pad on each fingertip's inner face, a box in the finger link's frame: (centre of the left
# finger's, size); the right finger's is mirrored in z. The convex pieces of a finger meet a held
# object at one point each (their inner faces are not parallel) and the object turns about the line
# between the fingers; the pad, 0.5 mm proud of the pieces, covers the tip's flat 25 x 18 mm.
PAD = ((.069426, 0., .018), (.025, .003, .018))


def edit(root):
    """Joint changes over the vendor URDF and finger pads (listed in README)."""
    edit_joints(root)
    add_pads(root)


def edit_joints(root):
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


def add_pads(root):
    import xml.etree.ElementTree as ET
    (x, y, z), size = PAD
    for link in root.findall('link'):
        for finger, sign in (('left', 1.), ('right', -1.)):
            if link.get('name').endswith(f'_arm_gripper_{finger}_link'):
                collision = ET.SubElement(link, 'collision', name=f'{link.get("name")}_pad')
                ET.SubElement(collision, 'origin', xyz=f'{x:g} {y:g} {sign * z:g}', rpy='0 0 0')
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(f'{v:g}' for v in size))
