"""A synthetic fixed single-arm robot (a Franka-like arm on a stand) for the generic robot code.

Six revolute joints (base yaw, shoulder, elbow, a z-y-z wrist) and a two-finger
gripper, collision from boxes and cylinders, no mobile base, lift or head.
"""

import pytest

from table_1000.robots.agent import ArmSpec, RobotSpec

# (joint, axis, origin of the child link in the parent, collision box of the child (size, centre))
CHAIN = [
    ('joint1', '0 0 1', (0., 0., .10), ((.10, .10, .10), (0., 0., .05))),
    ('joint2', '0 1 0', (0., 0., .10), ((.06, .06, .26), (0., 0., .15))),
    ('joint3', '0 1 0', (0., 0., .30), ((.05, .05, .21), (0., 0., .125))),
    ('joint4', '0 0 1', (0., 0., .25), ((.04, .04, .04), (0., 0., .03))),
    ('joint5', '0 1 0', (0., 0., .06), ((.04, .04, .04), (0., 0., .03))),
    ('joint6', '0 0 1', (0., 0., .06), ((.08, .03, .03), (0., 0., .02))),
]
LIMITS = {'joint2': (-1.8, 1.8), 'joint3': (-2.4, 2.4), 'joint5': (-2., 2.)}
TCP = (0., 0., .11)


def _inertial(mass=.5):
    return (f'<inertial><mass value="{mass}"/>'
            '<inertia ixx="1e-3" iyy="1e-3" izz="1e-3" ixy="0" ixz="0" iyz="0"/></inertial>')


def _link(name, size, centre):
    xyz = ' '.join(map(str, centre))
    box = ' '.join(map(str, size))
    return (f'<link name="{name}">{_inertial()}'
            f'<visual><origin xyz="{xyz}"/><geometry><box size="{box}"/></geometry></visual>'
            f'<collision><origin xyz="{xyz}"/><geometry><box size="{box}"/></geometry></collision></link>')


def arm_urdf():
    parts = ['<robot name="arm">', _link('base_link', (.16, .16, .10), (0., 0., .05))]
    parent = 'base_link'
    for index, (joint, axis, origin, (size, centre)) in enumerate(CHAIN, start=1):
        child = f'link{index}'
        low, high = LIMITS.get(joint, (-2.8, 2.8))
        parts += [_link(child, size, centre),
                  f'<joint name="{joint}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
                  f'<origin xyz="{" ".join(map(str, origin))}"/><axis xyz="{axis}"/>'
                  f'<limit lower="{low}" upper="{high}" effort="50" velocity="2"/></joint>']
        parent = child
    for side, sign in (('left', 1), ('right', -1)):
        lower, upper = (0., .04) if sign > 0 else (-.04, 0.)
        parts += [_link(f'finger_{side}', (.01, .01, .05), (0., sign * .005, .025)),
                  f'<joint name="finger_{side}_joint" type="prismatic"><parent link="link6"/>'
                  f'<child link="finger_{side}"/><origin xyz="0 0 .05"/><axis xyz="0 1 0"/>'
                  f'<limit lower="{lower}" upper="{upper}" effort="20" velocity=".2"/></joint>']
    return '\n'.join(parts + ['</robot>'])


def arm_spec(urdf='robot.urdf'):
    return RobotSpec(
        name='arm', urdf=urdf,
        arms={'arm': ArmSpec(name='arm', joints=tuple(joint for joint, *_ in CHAIN), tcp_link='arm_tcp',
                             tcp=('link6', TCP), fingers=('finger_left_joint', 'finger_right_joint'),
                             finger_links=('finger_left', 'finger_right'), max_gap=.08, mount='base_link')},
        keyframes={'ready': {'joint2': .5, 'joint3': 1., 'joint5': .8}},
        seated=(('finger_left', 'finger_right'),), unpadded=('finger_left', 'finger_right'))


@pytest.fixture(scope='module')
def arm_root(tmp_path_factory):
    root = tmp_path_factory.mktemp('arm_robot')
    (root / 'robot.urdf').write_text(arm_urdf())
    return root


@pytest.fixture(scope='module')
def arm_model(arm_root, tmp_path_factory):
    pytest.importorskip('mplib')
    from table_1000.robots.kinematics import RobotModel
    spec = arm_spec()
    model = RobotModel(spec, tmp_path_factory.mktemp('planning'), root=arm_root)
    model.set_state(spec.keyframe('ready'))
    return model
