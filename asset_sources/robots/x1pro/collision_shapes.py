"""X1Pro collision as boxes, cylinders and spheres (see table_1000/robots/primitives.py).

Regions are ranges in each visual mesh's own frame (m). Parts not listed get
one minimum-volume box; the fingers keep their convex pieces (wedges, the
grasp contact).
"""

KEEP = ('gripper_left_link', 'gripper_right_link')

# The bracket on the back of the torso runs inside the lift column (x < .075).
TORSO = (.075, None)

SHAPES = {
    # Base, the slope from base to column in four steps, the column.
    'chassis_base_link': [
        {'type': 'box', 'region': {'z': (None, .246)}},
        {'type': 'box', 'region': {'z': (.246, .308)}},
        {'type': 'box', 'region': {'z': (.308, .369)}},
        {'type': 'box', 'region': {'z': (.369, .43)}},
        {'type': 'box', 'region': {'z': (.43, .491)}},
        {'type': 'box', 'region': {'z': (.491, None)}},
    ],
    'left_wheel_link': [{'type': 'cylinder', 'axis': 'y'}],
    'right_wheel_link': [{'type': 'cylinder', 'axis': 'y'}],
    # Torso: trunk in two steps, the lower centre, the two shoulder bridges, the
    # two shoulder rings carrying the arms.
    'lift_link': [
        {'type': 'box', 'region': {'x': TORSO, 'z': (.324, None)}},
        {'type': 'box', 'region': {'x': TORSO, 'z': (.059, .324)}},
        {'type': 'box', 'region': {'x': TORSO, 'z': (None, .059), 'y': (-.11, .11)}},
        {'type': 'box', 'fit': 'obb', 'region': {'x': TORSO, 'z': (None, .059), 'y': (.11, .2)}},
        {'type': 'box', 'fit': 'obb', 'region': {'x': TORSO, 'z': (None, .059), 'y': (-.2, -.11)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'x': TORSO, 'y': (.2, None)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'x': TORSO, 'y': (None, -.2)}},
    ],
    'arm_base_link': [{'type': 'cylinder', 'axis': 'z'}],
    'head_yaw_link': [{'type': 'box'}],
    'head_pitch_link': [
        {'type': 'box', 'region': {'z': (None, .014)}},
        {'type': 'box', 'fit': 'obb', 'region': {'z': (.014, None)}},
    ],
    # Arm links: a few boxes along each link; joint housings as cylinders.
    'arm_link1': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, .032)}},
        {'type': 'box', 'region': {'z': (.032, None)}},
    ],
    'arm_link2': [
        {'type': 'box', 'region': {'x': (None, -.214)}},
        {'type': 'box', 'region': {'x': (-.214, -.046)}},
        {'type': 'box', 'region': {'x': (-.046, None)}},
    ],
    'arm_link3': [
        {'type': 'box', 'region': {'x': (None, .014)}},
        {'type': 'box', 'region': {'x': (.014, .063)}},
        {'type': 'box', 'region': {'x': (.063, None)}},
    ],
    'arm_link4': [
        {'type': 'box', 'region': {'x': (None, .033)}},
        {'type': 'box', 'region': {'x': (.033, .045)}},
        {'type': 'box', 'region': {'x': (.045, None)}},
    ],
    'arm_link5': [
        {'type': 'box', 'region': {'z': (None, -.041)}},
        {'type': 'box', 'region': {'z': (-.041, None)}},
    ],
    'arm_link6': [{'type': 'cylinder', 'axis': 'x'}],
    # Gripper: the round body, the finger rail, the camera arm.
    'gripper_base_link': [
        {'type': 'cylinder', 'axis': 'x', 'region': {'x': (None, .06), 'z': (None, .031)}},
        {'type': 'box', 'region': {'x': (.06, None), 'z': (None, .031)}},
        {'type': 'box', 'fit': 'obb', 'region': {'z': (.031, None)}},
    ],
}
