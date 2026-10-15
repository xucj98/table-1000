"""X1Pro acceptance settings for ``scripts/robots/check_robot.py`` (cases in table_1000/robots/checks.py).

Check scene: the robot's root link (base_link) at the world origin facing +X,
a table whose near edge is 0.29 m in front of it. Case targets are placed
relative to ``WORK``, a point on the table top in front of the primary (right)
arm, and to the table height.
"""

TABLE = {'size': (.8, 1.2), 'height': .755, 'top_thickness': .03, 'leg_size': .05, 'center': (.69, 0.),
         'friction': (.5, .4)}
ROBOT = {'position': (0., 0., 0.), 'yaw': 0., 'keyframe': 'ready'}
WORK = (.45, -.15)
# Bodies that objects are dropped onto: the base, the torso, an arm and the head.
DROP_LINKS = {'base': 'base_link', 'torso': 'lift_link', 'arm': 'right_arm_link3', 'head': 'head_pitch_link'}
# Every case of the library applies to this robot.
CASES = ('keyframes', 'sensors', 'arm_hits_block', 'arm_pushes_objects', 'drop_on_robot', 'lift_into_shelf',
         'self_collision', 'grasp_and_shake', 'base_into_wall', 'drive_holding', 'joint_limits')
