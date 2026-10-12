"""scene-000000: X1Pro at a table with a three-drawer cabinet and a pen.

World frame: origin at the floor below the table centre, +Z up. The robot
stands in front of the table's -Y edge facing +Y. The cabinet's drawers pull
toward the robot (cabinet -Y = world -Y); the pen lies with its axis along
world X. Asset paths are relative to the ``assets/`` root.

``placement(seed)`` draws the cabinet and pen XY offsets independently and
uniformly within +/-HALF_RANGE of their region centres; orientation is fixed.
``check(placement)`` lists violated layout constraints (empty when valid).
"""

import math

import numpy as np

TABLE = {'size': (1.36, 1.02), 'height': .755, 'top_thickness': .03, 'leg_size': .05,
         'center': (0., 0.), 'friction': (.5, .4)}

ROBOT = {
    'asset': 'robots/x1pro',
    'position': (-.03, -.80, 0.),
    'yaw': math.pi / 2,
    'joints': {
        'lift_joint': .40,
        'right_arm_joint1': 0., 'right_arm_joint2': .4, 'right_arm_joint3': -1., 'right_arm_joint4': 0.,
        'right_arm_joint5': 0., 'right_arm_joint6': 0.,
        'left_arm_joint1': 0., 'left_arm_joint2': .4, 'left_arm_joint3': -1., 'left_arm_joint4': 0.,
        'left_arm_joint5': 0., 'left_arm_joint6': 0.,
    },
}

# Region centres (world XY) and fixed orientations. ``z`` is the asset origin
# height: the cabinet origin is the bottom of its feet, the pen origin is on
# its axis and the cap (radius 0.010 m) is its widest part.
CABINET = {'asset': 'objects/cabinet/000000', 'center': (.30, 0.), 'z': TABLE['height'], 'yaw': 0.}
PEN = {'asset': 'objects/pen/000002', 'center': (-.04, -.36), 'z': TABLE['height'] + .010, 'yaw': 0.}
HALF_RANGE = .05

# Footprints in each asset's own XY frame, for the layout checks.
CABINET_FOOTPRINT = ((-.143, -.147), (.143, .138))   # housing and closed handles
DRAWER_TRAVEL = .173                                  # full stroke along cabinet -Y
PEN_FOOTPRINT = ((-.093, -.010), (.086, .010))
ROBOT_REACH_CLEARANCE = .05                           # free band along the table's robot edge


def pose(center, z, yaw, offset=(0., 0.)):
    """4x4 world transform of an asset origin."""
    transform = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    transform[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    transform[:3, 3] = [center[0] + offset[0], center[1] + offset[1], z]
    return transform


def placement(seed=None, offsets=None):
    """Initial poses for one run.

    ``seed`` draws offsets with ``numpy.random.default_rng(seed)``: first the
    cabinet's (x, y), then the pen's. ``offsets`` gives them explicitly as
    ``{'cabinet': (dx, dy), 'pen': (dx, dy)}``; with neither, both are zero.
    """
    if seed is not None and offsets is not None:
        raise ValueError('give a seed or explicit offsets, not both')
    if seed is not None:
        rng = np.random.default_rng(seed)
        offsets = {name: tuple(rng.uniform(-HALF_RANGE, HALF_RANGE, 2)) for name in ('cabinet', 'pen')}
    offsets = {name: tuple(map(float, (offsets or {}).get(name, (0., 0.)))) for name in ('cabinet', 'pen')}
    for name, value in offsets.items():
        if max(abs(v) for v in value) > HALF_RANGE + 1e-12:
            raise ValueError(f'{name} offset {value} is outside +/-{HALF_RANGE} m')
    return {'seed': seed, 'offsets': offsets,
            'cabinet': pose(CABINET['center'], CABINET['z'], CABINET['yaw'], offsets['cabinet']),
            'pen': pose(PEN['center'], PEN['z'], PEN['yaw'], offsets['pen'])}


def _world_box(transform, box):
    (x0, y0), (x1, y1) = box
    corners = np.array([[x, y, 0., 1.] for x in (x0, x1) for y in (y0, y1)]) @ transform.T
    return corners[:, :2].min(0), corners[:, :2].max(0)


def _overlap(a, b):
    return bool(np.all(a[0] < b[1]) and np.all(b[0] < a[1]))


def check(layout):
    """Violated constraints of a placement: support, overlap and free space."""
    problems = []
    half = np.array(TABLE['size']) / 2
    table = (np.array(TABLE['center']) - half, np.array(TABLE['center']) + half)
    cabinet = _world_box(layout['cabinet'], CABINET_FOOTPRINT)
    (x0, y0), (x1, y1) = CABINET_FOOTPRINT
    swept = _world_box(layout['cabinet'], ((x0, y0 - DRAWER_TRAVEL), (x1, y1)))
    pen = _world_box(layout['pen'], PEN_FOOTPRINT)
    for name, box in (('cabinet', cabinet), ('pen', pen)):
        if np.any(box[0] < table[0]) or np.any(box[1] > table[1]):
            problems.append(f'{name} not fully supported by the table')
    if _overlap(swept, pen):
        problems.append('pen inside the cabinet or its fully opened drawer space')
    robot_edge = table[0][1] + ROBOT_REACH_CLEARANCE
    if min(cabinet[0][1], pen[0][1]) < robot_edge:
        problems.append('object within the free band along the robot edge')
    return problems
