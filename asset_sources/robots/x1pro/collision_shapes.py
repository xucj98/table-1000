"""X1Pro collision as boxes and cylinders (see table_1000/robots/primitives.py).

Regions are ranges in each visual mesh's own frame (m). Arm links are modelled
by hand: every joint housing is a cylinder along its joint axis (axes and
positions from robot.urdf), the segments between them boxes. Body parts use
slabs chosen by primitives.design (least primitive volume for the count).
Parts not listed get one minimum-volume box; the fingers keep their convex
pieces (wedges, the grasp contact). The torso's bracket inside the lift column
(x < .075) gets no collision.
"""

KEEP = ('gripper_left_link', 'gripper_right_link')

SHAPES = {
    # Joint 1 housing (axis z at the origin), joint 2 housing (axis y at x .025, z .054).
    'arm_link1': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, .032)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'z': (.032, None)}},
    ],
    # Joint 2 housing (axis y at the origin), the beam, joint 3 housing (axis y at x -.26).
    'arm_link2': [
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (-.035, None)}},
        {'type': 'box', 'region': {'x': (-.225, -.035)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (None, -.225)}},
    ],
    # Joint 3 housing (axis y at the origin), the ramp up, the beam, joint 4 housing (axis y at x .232, z .075).
    'arm_link3': [
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (None, .04), 'z': (None, .045)}},
        {'type': 'box', 'fit': 'obb', 'region': {'x': (-.011, .088), 'z': (.03, None)}},
        {'type': 'box', 'region': {'x': (.063, .195)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (.195, None)}},
    ],
    # Joint 4 housing (axis y at the origin), the riser, joint 5 housing (axis z at x .082).
    'arm_link4': [
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (None, .021)}},
        {'type': 'box', 'region': {'x': (.021, .045)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'x': (.045, None)}},
    ],
    # Joint 5 housing (axis z at the origin), joint 6 housing (axis x at z -.094).
    'arm_link5': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (-.045, None)}},
        {'type': 'cylinder', 'axis': 'x', 'region': {'z': (None, -.045)}},
    ],
    'arm_link6': [{'type': 'cylinder', 'axis': 'x'}],
    # Gripper: the round body, the finger rail, the camera arm.
    'gripper_base_link': [
        {'type': 'cylinder', 'axis': 'x', 'region': {'x': (None, .06), 'z': (None, .031)}},
        {'type': 'box', 'region': {'x': (.06, None), 'z': (None, .031)}},
        {'type': 'box', 'fit': 'obb', 'region': {'z': (.031, None)}},
    ],
    # Body (primitives.design).
    'chassis_base_link': [
        {'type': 'box', 'region': {'z': (None, 0.0383)}},
        {'type': 'box', 'region': {'z': (0.0383, 0.222)}},
        {'type': 'box', 'region': {'z': (0.222, 0.2587)}},
        {'type': 'box', 'region': {'z': (0.2587, 0.2955)}},
        {'type': 'box', 'region': {'z': (0.2955, 0.3689)}},
        {'type': 'box', 'region': {'z': (0.3689, 0.5159)}},
        {'type': 'box', 'region': {'z': (0.5159, None)}},
    ],
    'lift_link': [
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (None, -0.1809)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1809, -0.1151)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1151, -0.0822)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.0822, 0.0822)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.0822, 0.1151)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.1151, 0.1645)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.1645, None)}},
    ],
    'head_pitch_link': [
        {'type': 'box', 'region': {'x': (None, -0.0406)}},
        {'type': 'cylinder', 'axis': 'x', 'region': {'x': (-0.0406, -0.0095)}},
        {'type': 'cylinder', 'axis': 'x', 'region': {'x': (-0.0095, 0.0371)}},
        {'type': 'box', 'region': {'x': (0.0371, None)}},
    ],
    'head_yaw_link': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0073)}},
        {'type': 'box', 'region': {'z': (0.0073, 0.0318)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'z': (0.0318, None)}},
    ],
    'arm_base_link': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0054)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (0.0054, None)}},
    ],
    'left_wheel_link': [
        {'type': 'cylinder', 'axis': 'y', 'region': {'y': (None, -0.0664)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'y': (-0.0664, None)}},
    ],
    'gripper_camera_base_link': [
        {'type': 'box', 'region': {'x': (None, -0.0014)}},
        {'type': 'box', 'region': {'x': (-0.0014, 0.0006)}},
        {'type': 'box', 'region': {'x': (0.0006, None)}},
    ],
    'right_wheel_link': [
        {'type': 'cylinder', 'axis': 'y', 'region': {'y': (None, 0.0664)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'y': (0.0664, None)}},
    ],
}
