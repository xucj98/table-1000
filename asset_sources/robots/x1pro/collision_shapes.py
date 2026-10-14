"""X1Pro collision as boxes and cylinders (see table_1000/robots/primitives.py).

Regions are ranges in each visual mesh's own frame (m). The arm links, torso
and chassis start from a hand model (joint housings as cylinders along their
joint axes, boxes between) refined by primitives.refine: the worst-fitting
primitive is split, across the axis and cut (median or widest empty band, e.g.
between the plates of a clevis) that leaves the least volume, until each is
within 6 mm (arm) or 10 mm (torso, chassis) at the 99th percentile or a part
has 10 (arm) or 14 primitives. The other body parts use slabs chosen by
primitives.design.
Parts not listed get one minimum-volume box. The hand (fingers, gripper base,
wrist camera) keeps its convex pieces: the grasps of the drawer-and-pen task
fail on millimetre changes of it. The torso's bracket inside the lift column
(x < .075) gets no collision.
"""

KEEP = ('gripper_left_link', 'gripper_right_link', 'gripper_base_link', 'gripper_camera_base_link')

SHAPES = {
    'arm_link1': [
        {'type': 'box', 'region': {'z': (None, 0.032), 'y': (None, -0.0293)}},
        {'type': 'box', 'region': {'z': (None, 0.032), 'y': (-0.0293, 0.0084)}},
        {'type': 'box', 'region': {'z': (None, 0.032), 'y': (0.0084, 0.029)}},
        {'type': 'box', 'region': {'z': (None, 0.032), 'y': (0.029, None)}},
        {'type': 'box', 'region': {'z': (0.032, 0.0535), 'y': (None, 0.0048)}},
        {'type': 'box', 'region': {'z': (0.0535, 0.0651), 'y': (None, 0.0048)}},
        {'type': 'box', 'region': {'z': (0.0651, None), 'y': (None, 0.0048)}},
        {'type': 'box', 'region': {'z': (0.032, 0.0535), 'y': (0.0048, None)}},
        {'type': 'box', 'region': {'z': (0.0535, 0.0665), 'y': (0.0048, None)}},
        {'type': 'box', 'region': {'z': (0.0665, None), 'y': (0.0048, None)}},
    ],
    # Joint 2 and joint 3 housings as cylinders on their axes, the beam between refined.
    'arm_link2': [
        {'type': 'cylinder', 'axis': 'y', 'center': (0., 0., 0.), 'region': {'x': (0., None)}},
        {'type': 'box', 'region': {'x': (-0.26, -0.2347)}},
        {'type': 'box', 'region': {'x': (-0.2347, -0.1945)}},
        {'type': 'box', 'region': {'x': (-0.1945, -0.0547)}},
        {'type': 'box', 'region': {'x': (-0.0547, 0.)}},
        {'type': 'cylinder', 'axis': 'y', 'center': (-0.26, 0., 0.), 'region': {'x': (None, -0.26)}},
    ],
    'arm_link3': [
        {'type': 'box', 'region': {'x': (None, 0.04), 'z': (None, 0.0099), 'y': (None, 0.0045)}},
        {'type': 'box', 'region': {'x': (None, 0.04), 'z': (None, 0.0099), 'y': (0.0045, None)}},
        {'type': 'box', 'region': {'x': (None, 0.04), 'z': (0.0099, 0.0299), 'y': (None, 0.0021)}},
        {'type': 'box', 'region': {'x': (None, 0.04), 'z': (0.0099, 0.0299), 'y': (0.0021, None)}},
        {'type': 'box', 'region': {'x': (None, 0.04), 'z': (0.0299, 0.045)}},
        {'type': 'box', 'region': {'x': (-0.011, 0.088), 'z': (0.03, 0.0589)}},
        {'type': 'box', 'region': {'x': (-0.011, 0.088), 'z': (0.0589, 0.0781)}},
        {'type': 'box', 'region': {'x': (-0.011, 0.088), 'z': (0.0781, None)}},
        {'type': 'box', 'region': {'x': (0.063, 0.195)}},
        {'type': 'cylinder', 'axis': 'y', 'region': {'x': (0.195, None)}},
    ],
    'arm_link4': [
        {'type': 'box', 'region': {'x': (None, 0.021), 'y': (None, 0.0059)}},
        {'type': 'box', 'region': {'x': (None, 0.021), 'y': (0.0059, None)}},
        {'type': 'box', 'region': {'x': (0.021, 0.045), 'z': (None, 0.0327)}},
        {'type': 'box', 'region': {'x': (0.021, 0.045), 'z': (0.0327, 0.0521)}},
        {'type': 'box', 'region': {'x': (0.021, 0.045), 'z': (0.0521, 0.0563)}},
        {'type': 'box', 'region': {'x': (0.021, 0.045), 'z': (0.0563, 0.0612)}},
        {'type': 'box', 'region': {'x': (0.021, 0.045), 'z': (0.0612, None)}},
        {'type': 'box', 'region': {'x': (0.045, 0.0519)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'x': (0.0519, None), 'z': (None, 0.0912)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'x': (0.0519, None), 'z': (0.0912, None)}},
    ],
    'arm_link5': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (-0.045, None)}},
        {'type': 'box', 'region': {'z': (None, -0.1153)}},
        {'type': 'box', 'region': {'z': (-0.1153, -0.1053)}},
        {'type': 'box', 'region': {'z': (-0.1053, -0.0835)}},
        {'type': 'box', 'region': {'z': (-0.0835, -0.0762)}},
        {'type': 'box', 'region': {'z': (-0.0762, -0.0728)}},
        {'type': 'box', 'region': {'z': (-0.0728, -0.0696), 'y': (None, 0.0287)}},
        {'type': 'box', 'region': {'z': (-0.0728, -0.0696), 'y': (0.0287, None)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (-0.0696, -0.0551)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (-0.0551, -0.045)}},
    ],
    'arm_link6': [{'type': 'cylinder', 'axis': 'x'}],
    # Gripper: the round body, the finger rail, the camera arm.
    'gripper_base_link': [
        {'type': 'cylinder', 'axis': 'x', 'region': {'x': (None, .06), 'z': (None, .031)}},
        {'type': 'box', 'region': {'x': (.06, None), 'z': (None, .031)}},
        {'type': 'box', 'fit': 'obb', 'region': {'z': (.031, None)}},
    ],
    'chassis_base_link': [
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0383), 'x': (None, 0.0188), 'y': (None, 0.)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0383), 'x': (None, 0.0188), 'y': (0., None)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0383), 'x': (0.0188, None), 'y': (None, 0.)}},
        {'type': 'cylinder', 'axis': 'z', 'region': {'z': (None, 0.0383), 'x': (0.0188, None), 'y': (0., None)}},
        {'type': 'box', 'region': {'z': (0.0383, 0.0639)}},
        {'type': 'box', 'region': {'z': (0.0639, 0.222), 'y': (None, 0.1855)}},
        {'type': 'box', 'region': {'z': (0.0639, 0.222), 'y': (0.1855, None)}},
        {'type': 'box', 'region': {'z': (0.222, 0.2384)}},
        {'type': 'box', 'region': {'z': (0.2384, 0.2587)}},
        {'type': 'box', 'region': {'z': (0.2587, 0.2955)}},
        {'type': 'box', 'region': {'z': (0.2955, 0.3689)}},
        {'type': 'box', 'region': {'z': (0.3689, 0.4294)}},
        {'type': 'box', 'region': {'z': (0.4294, 0.5159)}},
        {'type': 'box', 'region': {'z': (0.5159, None)}},
    ],
    'lift_link': [
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (None, -0.2425)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.2425, -0.1809)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1809, -0.1437)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1437, -0.1151)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1151, -0.0822), 'z': (None, 0.1058)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.1151, -0.0822), 'z': (0.1058, None)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.0822, 0.0822), 'z': (None, 0.2313)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (-0.0822, 0.0822), 'z': (0.2313, None)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.0822, 0.1151), 'z': (None, 0.1068)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.0822, 0.1151), 'z': (0.1068, None)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.1151, 0.1371)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.1371, 0.1645)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.1645, 0.2381)}},
        {'type': 'box', 'region': {'x': (0.075, None), 'y': (0.2381, None)}},
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
