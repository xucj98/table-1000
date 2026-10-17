"""Expert of scene-000000: open the cabinet's bottom drawer, put the pen in it, close it (right arm).

Every target is computed from the live poses at that moment and the assets'
collision geometry; nothing depends on the seed. Fingers may touch the drawer
handle or the pen only in the moves that need it. TCP: +X approach, +Y finger
opening (see asset_sources/robots/x1pro/robot.py); the fingertips are
TIP_BEYOND_TCP past the TCP.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.grasps import build_grasp_pose, standoff, translated
from table_1000.experts.solver import PlanningFailed

TIP_BEYOND_TCP = .0185
DRAWER, HANDLE = 'cabinet.drawer3', 'cabinet.drawer3.handle'
SLIDE = 'cabinet_drawer3_slide'
# Drawer travel for placing the pen (stroke 0.173 m): the exposed opening is
# about 0.11 m deep, so the fingers stay clear of the housing face.
OPENING = .12
CLOSED = .005
# Handle grasp: the fingers close on the handle's two ends; the approach points
# into the cabinet tilted down by HANDLE_PITCH and stops with the fingertips
# HANDLE_TIP_CLEARANCE in front of the drawer face.
HANDLE_PITCH = np.radians(30)
HANDLE_TIP_CLEARANCE = .006
HANDLE_HELD_GAP = (.035, .06)
# The drawer is pulled in segments; after each it must have followed the hand
# within SLIP_TOLERANCE, else the hand regrasps (at most MAX_REGRASPS times).
SEGMENTS, SLIP_TOLERANCE, MAX_REGRASPS = 3, .015, 2
# Pen grasp: top-down across the barrel at the centre of mass, kept within
# PEN_BARREL along its axis, fingertips PEN_TIP_CLEARANCE above the table.
PEN_BARREL = (-.024, .078)
PEN_TIP_CLEARANCE = .004
PEN_RADIUS = .009
PEN_APPROACH = .05
PEN_HELD_GAP = (.010, .03)
# Placement: the pen axis over the exposed part of the drawer, released PEN_DROP
# above the drawer walls with the hand leaning PLACE_TILT toward the cabinet.
PLACE_ABOVE, PEN_DROP, PLACE_TILT = .06, .012, np.radians(40)
# Options tried in order until one plans.
STANDOFFS = (.08, .05, .03)   # back-off from the handle or the released pen
RISES = (.10, .06, .03)       # rise after opening
PEN_LIFTS = (.10, .08, .06)


class Cabinet:
    """Handle and drawer frames of the cabinet from its collision geometry (drawer body frame)."""

    def __init__(self, geometry):
        low, high = geometry.bounds(HANDLE, DRAWER)
        self.handle_center = (low + high) / 2
        front_low, front_high = geometry.bounds(DRAWER + '.front', DRAWER)
        self.face_distance = float(front_low[1] - self.handle_center[1])
        left, right = sorted((geometry.bounds(DRAWER + '.side1', DRAWER), geometry.bounds(DRAWER + '.side2', DRAWER)),
                             key=lambda bounds: bounds[0][0])
        rear = geometry.bounds(DRAWER + '.rear', DRAWER)
        self.interior = ((float(left[1][0]), float(front_high[1])), (float(right[0][0]), float(rear[0][1])))
        self.floor = float(geometry.bounds(DRAWER + '.bottom', DRAWER)[1][2])
        self.wall_top = float(front_high[2])
        self.housing_face = float(geometry.bounds('cabinet.housing.shell', 'cabinet.housing')[0][1])

    def handle_grasp(self, drawer):
        """TCP pose closing on the handle's ends; the drawer's -Y is its pull direction."""
        c, s = np.cos(HANDLE_PITCH), np.sin(HANDLE_PITCH)
        approach = np.array([0., c, -s])
        tip_depth = self.face_distance - HANDLE_TIP_CLEARANCE
        tip = self.handle_center + [0., tip_depth, -s * tip_depth / c]
        local = build_grasp_pose(approach, [-1., 0., 0.], tip - approach * TIP_BEYOND_TCP)
        return drawer @ local

    def pen_inside(self, drawer, pen, margin=0.):
        local = (np.linalg.inv(drawer) @ pen)[:3, 3]
        (x0, y0), (x1, y1) = self.interior
        inside = (x0 + margin <= local[0] <= x1 - margin and y0 + margin <= local[1] <= y1 - margin
                  and self.floor - .005 <= local[2] <= self.wall_top)
        return bool(inside), local

    def release_point(self, drawer, housing):
        """World point above the exposed interior, between the inner front wall and the housing face."""
        face = housing[:3, :3] @ [0., self.housing_face, 0.] + housing[:3, 3]
        face_y = float((np.linalg.inv(drawer) @ np.r_[face, 1.])[1])
        (x0, y0), (x1, _) = self.interior
        local = np.array([(x0 + x1) / 2, (y0 + face_y) / 2, self.wall_top + PEN_DROP + PEN_RADIUS])
        return drawer[:3, :3] @ local + drawer[:3, 3]


def pen_grasp(pen, table_height, grasp_x):
    """Top-down grasp across the barrel at ``grasp_x`` (pen frame), wrist camera toward the cap end."""
    axis = pen[:3, 0].copy()
    axis[2] = 0.
    down = np.array([0., 0., -1.])
    centre = pen[:3, :3] @ [float(np.clip(grasp_x, *PEN_BARREL)), 0., 0.] + pen[:3, 3]
    centre[2] = max(centre[2], table_height + PEN_TIP_CLEARANCE + TIP_BEYOND_TCP)
    return build_grasp_pose(down, np.cross(down, axis / np.linalg.norm(axis)), centre)


def place_pose(release, tcp_pen, axis_target, toward):
    """TCP pose putting the held pen's origin at ``release`` with its axis along ``axis_target``,
    the hand pointing down and leaning PLACE_TILT toward ``toward``."""
    down = np.array([0., 0., -1.])
    axis_in_tcp = tcp_pen[:3, 0]
    candidates = [build_grasp_pose(down, np.cross(down, sign * axis_target), np.zeros(3))[:3, :3]
                  for sign in (1., -1.)]
    rotation = min(candidates, key=lambda r: np.linalg.norm(r @ axis_in_tcp - axis_target))
    axis = axis_target / np.linalg.norm(axis_target)
    lean = Rotation.from_rotvec(axis * PLACE_TILT)
    if np.dot(lean.apply(down), toward) < 0:
        lean = Rotation.from_rotvec(-axis * PLACE_TILT)
    rotation = lean.as_matrix() @ rotation
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(release) - rotation @ tcp_pen[:3, 3]
    return transform


class Expert:
    def __init__(self, ctx):
        self.ctx = ctx
        self.arm = arm = ctx.solver('right')
        self.cabinet = Cabinet(ctx.geometry['cabinet'])
        self.table_height = ctx.scene.TABLE['height']
        self.regrasps = 0
        arm.add_table()
        # Cabinet: one planning object per body, the handles apart (fingers may touch them).
        arm.add_object('cabinet', lambda body, part: (part if part.endswith('.handle') else body, body),
                       padded=lambda name: not name.endswith('.handle'))
        # Pen: its bodies stay assembled, one convex object in the pen body frame.
        arm.add_object('pen', lambda body, part: ('pen', 'body'), padded=lambda name: False, merged=lambda name: True)

    def drawer(self):
        return self.ctx.pose('cabinet', DRAWER)

    # -- drawer ------------------------------------------------------------------------
    def grasp_handle(self):
        arm = self.arm
        arm.open_gripper()
        grasp = self.cabinet.handle_grasp(self.drawer())
        arm.first_reachable([standoff(grasp, d) for d in STANDOFFS], arm.move_to_pose_with_RRTConnect)
        # Only the last, straight approach may touch the handle; a free-space path through it would
        # push the drawer.
        arm.allow(['cabinet:' + HANDLE])
        arm.move_to_pose_with_screw(self.cabinet.handle_grasp(self.drawer()))
        arm.close_gripper()
        gap = arm.finger_gap()
        if not HANDLE_HELD_GAP[0] <= gap <= HANDLE_HELD_GAP[1]:
            self.ctx.fail('handle not held', finger_gap=gap)

    def release_handle(self):
        arm = self.arm
        arm.open_gripper()
        held = arm.tcp()
        arm.first_reachable([standoff(held, d) for d in STANDOFFS])
        arm.allow(['cabinet:' + HANDLE], False)

    def slide(self, target):
        """Move the held handle along the drawer axis until the opening is ``target``; on slip, regrasp."""
        arm, segments = self.arm, SEGMENTS
        carried = ['cabinet:' + DRAWER, 'cabinet:' + HANDLE]
        contacts = ['cabinet:cabinet.housing', 'pen:pen']
        while True:
            start = self.ctx.joint('cabinet', SLIDE)
            if abs(target - start) <= .003:
                return
            step = (target - start) / max(1, segments)
            # The drawer is planned as carried by the hand; it slides in the housing and may push the pen in it.
            arm.carry(carried, contacts)
            try:
                arm.move_to_pose_with_screw(translated(arm.tcp(), -self.drawer()[:3, 1] * step))
            finally:
                arm.release(carried, contacts)
            moved = self.ctx.joint('cabinet', SLIDE) - start
            segments = max(1, segments - 1)
            if abs(moved - step) > SLIP_TOLERANCE:
                if self.regrasps >= MAX_REGRASPS:
                    self.ctx.fail('drawer did not follow the hand', moved=moved, commanded=step)
                self.regrasps += 1
                self.ctx.note('regrasp', moved=moved, commanded=step)
                self.release_handle()
                self.grasp_handle()

    # -- pen ---------------------------------------------------------------------------
    def pick_pen(self):
        ctx, arm = self.ctx, self.arm
        pen = ctx.pose('pen', 'body')
        grasp_x = float((np.linalg.inv(pen) @ np.r_[ctx.center_of_mass('pen', 'body'), 1.])[0])
        arm.allow(['pen:pen'])
        arm.move_to_pose_with_RRTConnect(standoff(pen_grasp(pen, self.table_height, grasp_x), PEN_APPROACH))
        arm.move_to_pose_with_screw(pen_grasp(ctx.pose('pen', 'body'), self.table_height, grasp_x))
        arm.close_gripper()
        gap = arm.finger_gap()
        if not PEN_HELD_GAP[0] <= gap <= PEN_HELD_GAP[1]:
            ctx.fail('pen not held', finger_gap=gap)
        arm.attach('pen:pen', ctx.pose('pen', 'body'))
        grasped = arm.tcp()
        arm.first_reachable([translated(grasped, [0., 0., h]) for h in PEN_LIFTS])
        ctx.hold(.2)
        pen = ctx.pose('pen', 'body')
        if pen[2, 3] < self.table_height + .03:
            ctx.fail('pen not lifted', pen_height=float(pen[2, 3]))
        # The carried geometry follows the pen as it actually sits in the hand.
        arm.detach('pen:pen')
        arm.world.set_pose('pen:pen', pen)
        return arm.attach('pen:pen', pen)

    def place_pen(self, tcp_pen):
        ctx, arm = self.ctx, self.arm

        def release_pose():
            drawer = self.drawer()
            release = self.cabinet.release_point(drawer, ctx.pose('cabinet', 'cabinet.housing'))
            return place_pose(release, tcp_pen, drawer[:3, 0], drawer[:3, 1])

        # A straight carry turns the hand only about the pen axis, so the pen stays
        # level above the drawer walls; RRTConnect is the fallback.
        above = translated(release_pose(), [0., 0., PLACE_ABOVE])
        try:
            arm.move_to_pose_with_screw(above)
        except PlanningFailed as error:
            ctx.note('carry with RRTConnect', reason=error.reason)
            arm.move_to_pose_with_RRTConnect(above)
        arm.move_to_pose_with_screw(release_pose())
        arm.open_gripper(.05)
        arm.detach('pen:pen')
        ctx.hold(.5)
        released = arm.tcp()
        # Back along the approach, or straight up when the wrist is at a limit.
        arm.first_reachable([standoff(released, d) for d in STANDOFFS]
                            + [translated(released, [0., 0., h]) for h in STANDOFFS])
        arm.allow(['pen:pen'], False)
        inside, local = self.cabinet.pen_inside(self.drawer(), ctx.pose('pen', 'body'))
        if not inside:
            ctx.fail('pen not in the drawer', pen_in_drawer=local.tolist())


def solve(ctx):
    expert = Expert(ctx)
    arm = expert.arm
    with ctx.subtask('open the bottom drawer of the cabinet'):
        expert.grasp_handle()
        expert.slide(OPENING)
        expert.release_handle()
        # Rise clear of the opened drawer's walls before the free-space move to the pen.
        released = arm.tcp()
        arm.first_reachable([translated(released, [0., 0., h]) for h in RISES])
    with ctx.subtask('pick up the pen'):
        tcp_pen = expert.pick_pen()
    with ctx.subtask('put the pen into the drawer'):
        expert.place_pen(tcp_pen)
    with ctx.subtask('close the drawer'):
        expert.grasp_handle()
        expert.slide(0.)
        expert.release_handle()
        if ctx.joint('cabinet', SLIDE) > CLOSED:
            ctx.fail('drawer not closed', opening=ctx.joint('cabinet', SLIDE))
