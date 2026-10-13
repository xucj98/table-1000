"""Choices for turning the vendor URDF package into this robot source.

Read by ``scripts/assets/prepare_robot.py x1pro``. The raw package
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
