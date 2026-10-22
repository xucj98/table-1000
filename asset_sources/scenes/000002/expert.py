"""Expert of scene 000002: move the pen from the cabinet's top drawer to its bottom drawer, by way of the table.

Open the top drawer, take the pen out, put it on the table, close the top
drawer; open the bottom drawer, pick the pen up from the table, put it in, close
it. Written with the atomic skills of :mod:`table_1000.experts.skills` alone:
each skill chooses the arm (the nearer that reaches), the grasp and its motion
from the live poses and the assets' geometry, so this file only says what to do
and where.
"""

import numpy as np

from table_1000.experts import skills

TOP, BOTTOM = 'drawer1', 'drawer3'
OPENING = .16           # m a drawer is pulled out (stroke 0.173)
OPEN_TOLERANCE = .02    # m short of OPENING still counts as open
CLOSED = .005           # m
PUSH_PAST = .003        # m pushed beyond the closed stop, so the drawer surely reaches it
# Spots for the pen on the table: a grid in front of the robot (SPOT_AHEAD of its centre, within
# the reach of a grasp from above), clear of the cabinet and of the drawers' pull-out space by
# SPOT_MARGIN, nearest the pen first.
SPOT_AHEAD, SPOT_SIDE, SPOT_STEP = (.38, .62), .45, .04
SPOT_MARGIN = .10
# The pen goes into a drawer along its width, either way round, or turned a little; on the table it
# may lie any way.
DRAWER_TURNS = np.radians([0., 180., 20., -20., 160., 200.])
TABLE_TURNS = np.radians([0., 180., 90., -90.])
DOWN = (0., 0., -1.)    # the pen is picked up from the table from above, to go down into the drawer


def slide(drawer):
    return f'cabinet_{drawer}_slide'


def body(drawer):
    return f'cabinet.{drawer}'


def pull(ctx, drawer, distance):
    """World displacement moving ``drawer``'s handle out by ``distance`` (in, when negative)."""
    return -ctx.pose('cabinet', body(drawer))[:3, 1] * distance


def move_drawer(ctx, drawer, opening):
    """Grasp the handle, slide the drawer to ``opening``, let go."""
    move = pull(ctx, drawer, opening - ctx.joint('cabinet', slide(drawer)))
    # Straight at the drawer face: pivoting fingers pinching the handle's ends from above slip off
    # in the pull.
    inward = ctx.pose('cabinet', body(drawer))[:3, 1]
    held = skills.grasp(ctx, 'cabinet', f'{body(drawer)}.handle', then=move, approach=inward)
    housing = ctx.pose('cabinet')[:3, 3]
    skills.move_by_displacement(ctx, held['arm'], *move, drag=('cabinet', body(drawer)))
    ctx.note('slid', drawer=drawer, opening=ctx.joint('cabinet', slide(drawer)),
             cabinet_shift=float(np.linalg.norm(ctx.pose('cabinet')[:3, 3] - housing)),
             finger_gap=ctx.solver(held['arm']).finger_gap())
    skills.release(ctx, held['arm'])


def open_drawer(ctx, drawer):
    move_drawer(ctx, drawer, OPENING)
    opening = ctx.joint('cabinet', slide(drawer))
    if opening < OPENING - OPEN_TOLERANCE:
        ctx.fail('drawer not opened', opening=opening)


def close_drawer(ctx, drawer):
    """Push the drawer shut by its handle with the closed fingers: no grasp needed, and the handle
    of an open drawer is too close to the body to grasp straight on."""
    inward = ctx.pose('cabinet', body(drawer))[:3, 1]
    housing = ctx.pose('cabinet')[:3, 3]
    pushed = skills.push(ctx, 'cabinet', f'{body(drawer)}.handle', direction=inward,
                         distance=ctx.joint('cabinet', slide(drawer)) + PUSH_PAST)
    opening = ctx.joint('cabinet', slide(drawer))
    ctx.note('pushed', drawer=drawer, opening=opening, arm=pushed['arm'], tilt=pushed['tilt'],
             cabinet_shift=float(np.linalg.norm(ctx.pose('cabinet')[:3, 3] - housing)))
    if opening > CLOSED:
        ctx.fail('drawer not closed', opening=opening)


def drawer_box(ctx, drawer):
    """``(T_world_drawer, (x0, y0), (x1, y1), floor, rear)``: the drawer's inner floor out of the
    cabinet (between the front wall and the housing face) and its rear wall, in the drawer frame."""
    geometry = ctx.geometry['cabinet']
    frame = ctx.pose('cabinet', body(drawer))
    front = geometry.bounds(f'{body(drawer)}.front', body(drawer))
    sides = sorted((geometry.bounds(f'{body(drawer)}.side{k}', body(drawer)) for k in (1, 2)), key=lambda b: b[0][0])
    floor = float(geometry.bounds(f'{body(drawer)}.bottom', body(drawer))[1][2])
    face = geometry.bounds('cabinet.housing.shell', 'cabinet.housing')[0][1]
    housing = ctx.pose('cabinet', 'cabinet.housing')
    face_world = housing[:3, :3] @ [0., face, 0.] + housing[:3, 3]
    face_y = float((np.linalg.inv(frame) @ np.r_[face_world, 1.])[1])
    rear = float(geometry.bounds(f'{body(drawer)}.rear', body(drawer))[0][1])
    return frame, (float(sides[0][1][0]), float(front[1][1])), (float(sides[1][0][0]), face_y), floor, rear


def table_spots(ctx):
    """Free spots on the table for the pen, nearest it first."""
    cabinet = ctx.pose('cabinet')
    low, high = ctx.geometry['cabinet'].bounds()
    low = low - [SPOT_MARGIN, OPENING + SPOT_MARGIN, 0.]
    high = high + SPOT_MARGIN
    base, height = ctx.base_transform(), ctx.scene.TABLE['height']
    pen = ctx.pose('pen', 'body')[:3, 3]
    spots = []
    for ahead in np.arange(SPOT_AHEAD[0], SPOT_AHEAD[1] + 1e-9, SPOT_STEP):
        for side in np.arange(-SPOT_SIDE, SPOT_SIDE + 1e-9, SPOT_STEP):
            spot = (base @ [ahead, side, 0., 1.])[:3]
            spot[2] = height
            local = (np.linalg.inv(cabinet) @ np.r_[spot, 1.])[:3]
            if not np.all((low[:2] <= local[:2]) & (local[:2] <= high[:2])):
                spots.append(spot)
    return sorted(spots, key=lambda spot: np.linalg.norm(spot[:2] - pen[:2]))


def solve(ctx):
    with ctx.subtask('open the top drawer of the cabinet'):
        open_drawer(ctx, TOP)
    with ctx.subtask('take the pen out of the top drawer'):
        skills.pick(ctx, 'pen')
    with ctx.subtask('put the pen on the table'):
        skills.place(ctx, 'pen', table_spots(ctx), turns=TABLE_TURNS)
    with ctx.subtask('close the top drawer'):
        close_drawer(ctx, TOP)
    with ctx.subtask('open the bottom drawer of the cabinet'):
        open_drawer(ctx, BOTTOM)
    with ctx.subtask('pick up the pen from the table'):
        skills.pick(ctx, 'pen', approach=DOWN)
    with ctx.subtask('put the pen into the bottom drawer'):
        frame, (x0, y0), (x1, y1), floor, rear = drawer_box(ctx, BOTTOM)
        spot = frame @ [(x0 + x1) / 2, (y0 + y1) / 2, floor, 1.]
        skills.place(ctx, 'pen', [spot[:3]], heading=frame[:3, 0], turns=DRAWER_TURNS)
        local = (np.linalg.inv(frame) @ ctx.pose('pen', 'body'))[:3, 3]
        if not (x0 <= local[0] <= x1 and y0 <= local[1] <= rear and local[2] < floor + .03):
            ctx.fail('pen not in the drawer', pen_in_drawer=local.tolist())
    with ctx.subtask('close the bottom drawer'):
        close_drawer(ctx, BOTTOM)
