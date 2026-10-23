"""Expert of scene 000002: move the pen from the cabinet's top drawer to its bottom drawer, by way of the table.

Open the top drawer, take the pen out, put it on the table, close the top
drawer; open the bottom drawer, pick the pen up from the table, put it in, close
it. Written with the semantic skills of :mod:`table_1000.experts.skills.semantic`
(handles, pull directions and places to put the pen come from the cabinet's
geometry and joints) and the atomic ones under them. Before reaching onto the
table the base goes back to where it started (make_room).
"""

import numpy as np

from table_1000.experts import skills
from table_1000.experts.context import Step, run_steps
from table_1000.experts.skills import semantic

TOP, BOTTOM = 'cabinet.drawer1.slide', 'cabinet.drawer3.slide'
OPENING = .16           # m a drawer is pulled out (stroke 0.173)
# On the table the pen keeps CLEAR of the cabinet and of its drawers' pull-out space, nearest ASIDE
# (ahead of the robot's centre, to the side away from the cabinet), out of the arm's way.
CLEAR, ASIDE = .10, (.5, .15)
# The pen is picked up from the table from above, leaning PICK_TILT about its axis with the wrist
# toward the robot: put down along the drawer's width it then leans away from the drawers above.
PICK_TILT = np.radians(35.)
PICK_ANGLES = np.radians([15., 180.])  # how far the grasp may turn from that, the first that reaches


def aside(ctx):
    base, cabinet = ctx.base_transform(), ctx.pose('cabinet')[:3, 3]
    side = -np.sign((np.linalg.inv(base) @ np.r_[cabinet, 1.])[1]) or 1.
    spot = base @ [ASIDE[0], ASIDE[1] * side, 0., 1.]
    return np.array([spot[0], spot[1], ctx.scene.TABLE['height']])


def clear(ctx):
    """Keep-out rectangles of the cabinet and its drawers' pull-out space, grown by CLEAR."""
    return [semantic.body_box(ctx, 'cabinet', 'cabinet.housing', CLEAR),
            *(semantic.travel(ctx, 'cabinet', drawer, CLEAR) for drawer in (TOP, BOTTOM))]


def pick_up(ctx):
    """Pick the pen up from the table from above, leaning about its axis toward the robot."""
    pen = ctx.pose('pen', 'body')
    across = np.cross([0., 0., 1.], pen[:3, 0])
    across /= np.linalg.norm(across)
    if across @ (pen[:3, 3] - ctx.base_transform()[:3, 3]) < 0.:
        across = -across
    approach = np.cos(PICK_TILT) * np.array([0., 0., -1.]) + np.sin(PICK_TILT) * across
    skills.make_room(ctx, [])
    skills.retry(ctx, lambda: skills.pick(ctx, 'pen', approach=approach, max_angle=PICK_ANGLES))


def subtask(instruction, *actions):
    """``actions(ctx)`` in turn as subtask ``instruction``, the free arms back at their start pose
    first (out of the way of the other)."""
    def run(ctx):
        with ctx.subtask(instruction):
            skills.ready(ctx)
            for action in actions:
                action(ctx)
    return run


def opened(drawer):
    return lambda ctx: semantic.open(ctx, 'cabinet', drawer, OPENING)


def closed(drawer):
    return lambda ctx: semantic.close(ctx, 'cabinet', drawer)


def is_open(drawer):
    return lambda ctx: semantic.require(ctx, semantic.is_open(ctx, 'cabinet', drawer, OPENING), 'drawer not opened')


def is_closed(drawer):
    return lambda ctx: semantic.require(ctx, semantic.is_closed(ctx, 'cabinet', drawer), 'drawer not closed')


def take_out(ctx):
    skills.retry(ctx, lambda: skills.pick(ctx, 'pen'))


def put_on_table(ctx):
    skills.make_room(ctx, [])       # the spots are in front of the robot where it stood
    semantic.place_on(ctx, 'pen', 'table', near=aside(ctx), keep_out=clear(ctx))


# The open skill does nothing to a drawer already open: before the pen is taken out of a drawer or
# put in, it opens it again only if it was pushed (part way) shut since.
STEPS = [
    Step('open the top drawer', subtask('open the top drawer of the cabinet', opened(TOP)), is_open(TOP)),
    Step('move the pen from the top drawer to the table', lambda ctx: (
         subtask('take the pen out of the top drawer', opened(TOP), take_out)(ctx),
         subtask('put the pen on the table', put_on_table)(ctx)),
         lambda ctx: semantic.require(ctx, semantic.on(ctx, 'pen', 'table', keep_out=clear(ctx)),
                                      'pen not on the table')),
    Step('close the top drawer', subtask('close the top drawer', closed(TOP)), is_closed(TOP)),
    Step('open the bottom drawer', subtask('open the bottom drawer of the cabinet', opened(BOTTOM)),
         is_open(BOTTOM)),
    Step('move the pen from the table to the bottom drawer', lambda ctx: (
         subtask('pick up the pen from the table', pick_up)(ctx),
         subtask('put the pen into the bottom drawer', opened(BOTTOM),
                 lambda ctx: semantic.place_inside(ctx, 'pen', 'cabinet', 'cabinet.drawer3'))(ctx)),
         lambda ctx: semantic.require(ctx, semantic.inside(ctx, 'pen', 'cabinet', 'cabinet.drawer3'),
                                      'pen not in the drawer')),
    Step('close the bottom drawer', subtask('close the bottom drawer', closed(BOTTOM)), is_closed(BOTTOM)),
]


def solve(ctx):
    run_steps(ctx, STEPS)

