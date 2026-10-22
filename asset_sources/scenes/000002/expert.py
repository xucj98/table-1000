"""Expert of scene 000002: move the pen from the cabinet's top drawer to its bottom drawer, by way of the table.

Open the top drawer, take the pen out, put it on the table, close the top
drawer; open the bottom drawer, pick the pen up from the table, put it in, close
it. Written with the atomic skills of :mod:`table_1000.experts.skills` alone:
each skill chooses the arm (the nearer that reaches), the grasp and its motion
from the live poses and the assets' geometry, so this file only says what to do
and where. Before reaching into the cabinet the base backs off or turns to where
the hand reaches with room (make_room); on the table it stands where it started.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts import skills
from table_1000.experts.context import ExpertFailed
from table_1000.experts.grasps import translated

TOP, BOTTOM = 'drawer1', 'drawer3'
OPENING = .16           # m a drawer is pulled out (stroke 0.173)
OPEN_TOLERANCE = .02    # m short of OPENING still counts as open
CLOSED = .005           # m
PUSH_PAST = .003        # m pushed beyond the closed stop, so the drawer surely reaches it
# The most a handle grasp may turn from straight at the drawer: tilted, pivoting fingers pinching the
# handle's ends may slip off; a low handle is reached tilted down.
HANDLE_ANGLES = np.radians([10., 30.])
PULLS, PULL_LEAST = 3, .25  # grasps a drawer is slid open with at most; the least part of the rest one slides
# Where the pen may go on the table: in front of the robot (SPOT_AHEAD of its centre, within the
# reach of a grasp from above), clear of the cabinet and of the drawers' pull-out space by
# SPOT_MARGIN; nearest ASIDE (ahead, to the side away from the cabinet), out of the arm's way.
SPOT_AHEAD, SPOT_SIDE = (.38, .62), .45
ASIDE = (.5, .15)
SPOT_MARGIN = .10
# The pen is picked up from the table from above, leaning PICK_TILT about its axis with the wrist
# toward the robot: put down along the drawer's width it then leans away from the drawers above.
PICK_TILT = np.radians(35.)
PICK_ANGLES = np.radians([15., 180.])  # how far the grasp may turn from that, the first that reaches
APPROACH_HEIGHT = .05   # m above a spot, the hand pose the base is placed to reach


def slide(drawer):
    return f'cabinet_{drawer}_slide'


def body(drawer):
    return f'cabinet.{drawer}'


def pull(ctx, drawer, distance):
    """World displacement moving ``drawer``'s handle out by ``distance`` (in, when negative)."""
    return -ctx.pose('cabinet', body(drawer))[:3, 1] * distance


def tcp_pose(position, approach, closing):
    """World TCP pose at ``position`` approaching along ``approach``, the fingers closing along ``closing``."""
    pose = np.eye(4)
    pose[:3, :3] = np.c_[approach, closing, np.cross(approach, closing)]
    pose[:3, 3] = position
    return pose


def handle_poses(ctx, drawer, opening):
    """Hand poses on ``drawer``'s handle straight at it, before, halfway and after sliding it to ``opening``."""
    frame = ctx.pose('cabinet', body(drawer))
    box, _ = ctx.part_box('cabinet', f'{body(drawer)}.handle', body(drawer))
    grasp = tcp_pose(box[:3, 3], frame[:3, 1], frame[:3, 0])
    move = pull(ctx, drawer, opening - ctx.joint('cabinet', slide(drawer)))
    return [grasp, translated(grasp, move / 2), translated(grasp, move)]


def above(position, across):
    """Hand pose straight down at ``position`` (lifted by APPROACH_HEIGHT), the fingers closing ``across``."""
    return tcp_pose(np.asarray(position, dtype=float) + [0., 0., APPROACH_HEIGHT], [0., 0., -1.], across)


def grasp_handle(ctx, drawer, move, inward):
    """Grasp ``drawer``'s handle within the first of HANDLE_ANGLES of straight at it that reaches."""
    for limit in HANDLE_ANGLES:
        try:
            return skills.grasp(ctx, 'cabinet', f'{body(drawer)}.handle', then=move, approach=inward,
                                max_angle=limit)
        except ExpertFailed as failure:
            if failure.reason != 'no reachable grasp' or limit == HANDLE_ANGLES[-1]:
                raise


def move_drawer(ctx, drawer, opening):
    """Grasp the handle, slide the drawer to ``opening``, let go; where the hand cannot follow the
    handle all the way (the wrist bends to its limit as the handle nears the body), it slides it
    part way, grasps again and goes on."""
    for _ in range(PULLS):
        skills.make_room(ctx, handle_poses(ctx, drawer, opening), path=True)
        move = pull(ctx, drawer, opening - ctx.joint('cabinet', slide(drawer)))
        # Straight at the drawer face: pivoting fingers pinching the handle's ends from above (even
        # 15 degrees) slip off.
        inward = ctx.pose('cabinet', body(drawer))[:3, 1]
        held = grasp_handle(ctx, drawer, move, inward)
        # Closing on the handle straight on may have pushed the drawer in a little.
        move = pull(ctx, drawer, opening - ctx.joint('cabinet', slide(drawer)))
        housing = ctx.pose('cabinet')[:3, 3]
        try:
            moved = skills.move_by_displacement(ctx, held['arm'], *move, drag=('cabinet', body(drawer)),
                                                least=PULL_LEAST)
        except ExpertFailed:
            skills.release(ctx, held['arm'])    # so that it can be tried again
            raise
        ctx.note('slid', drawer=drawer, opening=ctx.joint('cabinet', slide(drawer)), moved=moved,
                 cabinet_shift=float(np.linalg.norm(ctx.pose('cabinet')[:3, 3] - housing)),
                 finger_gap=ctx.solver(held['arm']).finger_gap())
        skills.release(ctx, held['arm'])
        if moved == 1.:
            return


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


def table_region(ctx):
    """``(region, keep_out)`` for the pen on the table: in front of the robot (SPOT_AHEAD of its
    centre, within reach of a grasp from above), clear of the cabinet and its drawers' pull-out
    space by SPOT_MARGIN."""
    frame = ctx.base_transform()
    frame[2, 3] = ctx.scene.TABLE['height']
    low, high = ctx.geometry['cabinet'].bounds()
    keep_out = (ctx.pose('cabinet'), low[:2] - [SPOT_MARGIN, OPENING + SPOT_MARGIN], high[:2] + SPOT_MARGIN)
    return (frame, (SPOT_AHEAD[0], -SPOT_SIDE), (SPOT_AHEAD[1], SPOT_SIDE)), [keep_out]


def pick_up(ctx):
    """Pick the pen up from the table leaning as :func:`pick_from_above`, within the first of PICK_ANGLES that reaches."""
    for limit in PICK_ANGLES:
        try:
            return skills.pick(ctx, 'pen', approach=pick_from_above(ctx), max_angle=limit)
        except ExpertFailed as failure:
            if failure.reason != 'no reachable grasp' or limit == PICK_ANGLES[-1]:
                raise


def pick_from_above(ctx):
    """Approach to the pen lying on the table: down, leaning about its axis toward the robot."""
    pen = ctx.pose('pen', 'body')
    across = np.cross([0., 0., 1.], pen[:3, 0])
    across /= np.linalg.norm(across)
    if across @ (pen[:3, 3] - ctx.base_transform()[:3, 3]) < 0.:
        across = -across
    return np.cos(PICK_TILT) * np.array([0., 0., -1.]) + np.sin(PICK_TILT) * across


def put_in_drawer(ctx, drawer):
    still_open(ctx, drawer)
    frame, (x0, y0), (x1, y1), floor, rear = drawer_box(ctx, drawer)
    surface = frame @ np.r_[np.c_[np.eye(3), [0., 0., floor]], [[0., 0., 0., 1.]]]
    near = (frame @ [(x0 + x1) / 2, (y0 + y1) / 2, floor, 1.])[:3]
    skills.retry(ctx, lambda: skills.place(ctx, 'pen', (surface, (x0, y0), (x1, y1)), near=near))
    local = (np.linalg.inv(frame) @ ctx.pose('pen', 'body'))[:3, 3]
    if not (x0 <= local[0] <= x1 and y0 <= local[1] <= rear and local[2] < floor + .03):
        ctx.fail('pen not in the drawer', pen_in_drawer=local.tolist())


def still_open(ctx, drawer):
    """Open ``drawer`` again if it was pushed (part way) shut since."""
    if ctx.joint('cabinet', slide(drawer)) < OPENING - OPEN_TOLERANCE:
        ctx.note('drawer pushed in', drawer=drawer, opening=ctx.joint('cabinet', slide(drawer)))
        open_drawer(ctx, drawer)


def take_out(ctx):
    still_open(ctx, TOP)
    pen = ctx.pose('pen', 'body')
    skills.make_room(ctx, [above(pen[:3, 3], np.cross([0., 0., 1.], pen[:3, 0]))])
    skills.pick(ctx, 'pen')


def aside(ctx):
    """World point on the table in front of the robot, on its side away from the cabinet: out of the
    way of the arm reaching into the cabinet."""
    base, cabinet = ctx.base_transform(), ctx.pose('cabinet')[:3, 3]
    side = -np.sign((np.linalg.inv(base) @ np.r_[cabinet, 1.])[1]) or 1.
    spot = base @ [ASIDE[0], ASIDE[1] * side, 0., 1.]
    return np.array([spot[0], spot[1], ctx.scene.TABLE['height']])


def put_on_table(ctx):
    skills.make_room(ctx, [])       # the spots are in front of the robot where it stood
    region, keep_out = table_region(ctx)
    skills.place(ctx, 'pen', region, keep_out=keep_out, near=aside(ctx))


def solve(ctx):
    # Each step starts with the free arms back at their start pose (out of the way of the other) and
    # is tried again when it fails (skills.retry), from where things are.
    def step(instruction, action):
        with ctx.subtask(instruction):
            skills.ready(ctx)
            action()

    step('open the top drawer of the cabinet', lambda: skills.retry(ctx, lambda: open_drawer(ctx, TOP)))
    step('take the pen out of the top drawer', lambda: skills.retry(ctx, lambda: take_out(ctx)))
    step('put the pen on the table', lambda: skills.retry(ctx, lambda: put_on_table(ctx)))
    step('close the top drawer', lambda: skills.retry(ctx, lambda: close_drawer(ctx, TOP)))
    step('open the bottom drawer of the cabinet', lambda: skills.retry(ctx, lambda: open_drawer(ctx, BOTTOM)))
    step('pick up the pen from the table', lambda: (skills.make_room(ctx, []), skills.retry(ctx, lambda: pick_up(ctx))))
    step('put the pen into the bottom drawer', lambda: put_in_drawer(ctx, BOTTOM))
    step('close the bottom drawer', lambda: skills.retry(ctx, lambda: close_drawer(ctx, BOTTOM)))


# -- each step alone (run_expert.py --stage <name>) -------------------------------------------------
def pen_on_table(ctx):
    """World pose of the pen lying on the table aside (see :func:`aside`), at a random heading."""
    spot = aside(ctx)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', ctx.rng.uniform(-np.pi, np.pi)).as_matrix()
    low, _ = ctx.geometry['pen'].bounds()
    pose[:3, 3] = [spot[0], spot[1], ctx.scene.TABLE['height'] - low[2] + .002]
    return pose


def opened(drawer):
    return {'cabinet': {slide(drawer): OPENING}}


def opened_with_pen(ctx, drawer):
    """``drawer`` open, the pen in it moved out with it."""
    ctx.arrange({'pen': translated(ctx.pose('pen'), pull(ctx, drawer, OPENING - ctx.joint('cabinet', slide(drawer))))},
                opened(drawer))


def staged(steps):
    """``{name: stage(ctx)}``, each running its step as one subtask."""
    def stage(name, step):
        def run(ctx):
            with ctx.subtask(name):
                step(ctx)
        return run
    return {name: stage(name, step) for name, step in steps.items()}


STAGES = staged({
    'open the top drawer': lambda ctx: skills.retry(ctx, lambda: open_drawer(ctx, TOP)),
    'take the pen out': lambda ctx: (opened_with_pen(ctx, TOP), skills.retry(ctx, lambda: take_out(ctx))),
    'take the pen out and put it on the table': lambda ctx: (
        opened_with_pen(ctx, TOP), skills.retry(ctx, lambda: take_out(ctx)),
        skills.retry(ctx, lambda: put_on_table(ctx))),
    'close the top drawer': lambda ctx: (
        ctx.arrange({'pen': pen_on_table(ctx)}, opened(TOP)), skills.retry(ctx, lambda: close_drawer(ctx, TOP))),
    'open the bottom drawer': lambda ctx: (
        ctx.arrange({'pen': pen_on_table(ctx)}), skills.retry(ctx, lambda: open_drawer(ctx, BOTTOM))),
    'pick the pen up from the table and put it into the bottom drawer': lambda ctx: (
        ctx.arrange({'pen': pen_on_table(ctx)}, opened(BOTTOM)),
        skills.make_room(ctx, []),
        skills.retry(ctx, lambda: pick_up(ctx)),
        put_in_drawer(ctx, BOTTOM)),
    'close the bottom drawer': lambda ctx: (
        ctx.arrange({'pen': pen_on_table(ctx)}, opened(BOTTOM)), skills.retry(ctx, lambda: close_drawer(ctx, BOTTOM))),
})
