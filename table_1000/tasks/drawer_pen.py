"""Open the cabinet's bottom drawer, put the pen in it and close it.

``Solver.solve()`` is the fixed script, a generator of 25 Hz actions for one
environment (see ``table_1000.tasks.motion.run_expert``); every pose it targets
is computed from the live body poses at that moment and the assets' collision
geometry (see ``drawer_pen_geometry``). Contact permission in the planning
world is granted only between the fingers and the drawer handle or the pen, and
only for the phases that need it.
"""

import numpy as np

from table_1000.planning.world import model_shapes
from table_1000.tasks import drawer_pen_geometry as g
from table_1000.robots.x1pro import FINGER_LINKS
from table_1000.tasks.motion import MotionPlanner, PlanningFailed

DRAWER_JOINT = 'cabinet_drawer3_slide'
PULL_SEGMENTS = 3          # the opening is pulled in this many checked segments
SLIP_TOLERANCE = .015      # m the drawer may lag the hand in one segment
MAX_REGRASPS = 2
HANDLE_HELD_GAP = (.035, .06)   # finger gap when closed on the 48 mm handle
PEN_HELD_GAP = (.010, .03)      # finger gap when closed on the 18-20 mm pen
PLACE_TILT = np.radians(40)
PEN_LIFTS = (.10, .08, .06)   # highest straight lift that is reachable
STANDOFFS = (.08, .05, .03)   # longest reachable handle standoff
RISES = (.10, .06, .03)       # rise after opening, highest reachable
CLOSED_TOLERANCE = .005


class TaskFailed(RuntimeError):
    def __init__(self, stage, reason, **details):
        super().__init__(f'{stage}: {reason}')
        self.stage, self.reason, self.details = stage, reason, details


class CabinetGeometry:
    """Handle and drawer frames of the cabinet, from its collision geometry."""

    def __init__(self, model):
        shapes, rest = model_shapes(model)

        def bounds(part, frame):
            points = []
            for body, name, transform, kind, data in shapes:
                if name == part or name.startswith(part + '.'):
                    vertices = data if kind == 'convex' else np.zeros((1, 3))
                    local = vertices @ transform[:3, :3].T + transform[:3, 3]
                    local = (np.c_[local, np.ones(len(local))] @ (np.linalg.inv(rest[frame]) @ rest[body]).T)[:, :3]
                    points.append(local)
            points = np.vstack(points)
            return points.min(0), points.max(0)

        drawer = g.DRAWER
        low, high = bounds(g.HANDLE, drawer)
        self.handle_center = (low + high) / 2
        front_low, front_high = bounds(drawer + '.front', drawer)
        self.face_distance = float(front_low[1] - self.handle_center[1])
        side1, side2 = bounds(drawer + '.side1', drawer), bounds(drawer + '.side2', drawer)
        rear = bounds(drawer + '.rear', drawer)
        left, right = sorted((side1, side2), key=lambda b: b[0][0])
        self.interior = ((float(left[1][0]), float(front_high[1])), (float(right[0][0]), float(rear[0][1])))
        self.floor = float(bounds(drawer + '.bottom', drawer)[1][2])
        self.wall_top = float(front_high[2])
        # Housing front face (most negative Y of the shell) in the housing frame.
        self.housing_face = float(bounds('cabinet.housing.shell', 'cabinet.housing')[0][1])

    def housing_face_in_drawer(self, housing_world, drawer_world):
        point = housing_world[:3, :3] @ [0., self.housing_face, 0.] + housing_world[:3, 3]
        return float((np.linalg.inv(drawer_world) @ np.r_[point, 1.])[1])

    def pen_inside(self, drawer_world, pen_world, margin=.0):
        local = (np.linalg.inv(drawer_world) @ pen_world)[:3, 3]
        (x0, y0), (x1, y1) = self.interior
        return bool(x0 + margin <= local[0] <= x1 - margin and y0 + margin <= local[1] <= y1 - margin
                    and self.floor - .005 <= local[2] <= self.wall_top), local


class Solver:
    """Expert for one environment; ``view`` is its :class:`~table_1000.sim.env.EnvView`."""

    def __init__(self, view, assets):
        scene = view.scene
        self.env = view
        self.motion = motion = MotionPlanner(view, 'right')
        table = scene.TABLE
        top = np.eye(4)
        top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
        motion.add_box('table', (*table['size'], table['top_thickness']), top)
        # Cabinet: one object per body, handles split out for contact permission.
        # Pen: its bodies stay assembled, so one object in the pen body frame.
        motion.add_asset(f'{assets}/{scene.CABINET["asset"]}/model.json', 'cabinet:',
                         lambda body, part: (part if part.endswith('.handle') else body, body),
                         lambda name: not name.endswith('.handle'))
        motion.add_asset(f'{assets}/{scene.PEN["asset"]}/model.json', 'pen:',
                         lambda body, part: ('pen', 'body'), lambda name: False, lambda name: True)
        self.cabinet = CabinetGeometry(f'{assets}/{scene.CABINET["asset"]}/model.json')
        self.table_height = table['height']
        self.regrasps = 0
        self.stage = 'start'
        self.events = []

    # -- helpers ---------------------------------------------------------------
    def enter(self, stage):
        self.stage = stage
        self.env.contacts.phase = stage
        self.events.append({'stage': stage, 'time': self.env.time})

    def poses(self):
        poses = self.env.body_poses()
        return poses['cabinet:' + g.DRAWER], poses['cabinet:cabinet.housing'], poses['pen:body']

    def first_reachable(self, options, move, reasons):
        """Run ``move(option)`` for the first option that plans; a plan failing
        for one of ``reasons`` tries the next, anything else propagates."""
        for index, option in enumerate(options):
            try:
                return (yield from move(option))
            except PlanningFailed as error:
                if not error.reason.startswith(reasons) or index == len(options) - 1:
                    raise

    def opening(self):
        return self.env.object_joint('cabinet', DRAWER_JOINT)

    def handle_grasp(self):
        drawer, _, _ = self.poses()
        return g.handle_frame(drawer, self.cabinet.handle_center) @ g.handle_tcp(self.cabinet.face_distance)

    def grasp_handle(self, label):
        """Approach and close on the handle; returns the TCP grasp pose."""
        m = self.motion
        m.allow(['cabinet:' + g.HANDLE])
        yield from m.open_gripper()
        grasp = self.handle_grasp()
        self.enter(f'{label}: reach handle')
        yield from self.first_reachable(
            STANDOFFS, lambda d: m.move_to_pose_with_RRTConnect(self.stage, g.standoff(grasp, d)), ('IK Failed',))
        self.enter(f'{label}: approach handle')
        yield from m.move_to_pose_with_screw(self.stage, self.handle_grasp())
        self.enter(f'{label}: close on handle')
        yield from m.close_gripper()
        gap = m.finger_gap()
        if not HANDLE_HELD_GAP[0] <= gap <= HANDLE_HELD_GAP[1]:
            drawer, _, _ = self.poses()
            handle = drawer[:3, :3] @ self.cabinet.handle_center + drawer[:3, 3]
            raise TaskFailed(self.stage, 'handle not held', finger_gap=gap, target_tcp=grasp.tolist(),
                             tcp=m.tcp().tolist(), handle=handle.tolist(),
                             link6=self.env.link_pose('right_arm_link6').tolist(),
                             fingers=[self.env.link_pose(l)[:3, 3].tolist() for l in FINGER_LINKS])
        return grasp

    def release_handle(self, label):
        m = self.motion
        self.enter(f'{label}: release handle')
        yield from m.open_gripper()
        held = m.tcp()
        yield from self.first_reachable(
            STANDOFFS, lambda d: m.move_to_pose_with_screw(self.stage, g.standoff(held, d)), ('joint_limit', 'collision'))
        m.allow(['cabinet:' + g.HANDLE], False)

    def slide(self, label, target_opening):
        """Move the held handle along the drawer axis until ``target_opening``.

        The hand moves in segments; after each, the drawer must have followed
        within SLIP_TOLERANCE. On slip the hand lets go, re-reads the handle and
        grasps again, at most MAX_REGRASPS times in the whole task.
        """
        m = self.motion
        segments = PULL_SEGMENTS
        while True:
            start = self.opening()
            remaining = target_opening - start
            if abs(remaining) <= .003:
                return
            step = remaining / max(1, segments)
            drawer, _, _ = self.poses()
            self.enter(f'{label}: slide {start:.3f}->{start + step:.3f} m')
            tcp = m.tcp()
            # The drawer is planned as carried by the hand; it slides inside the
            # housing and may push the pen lying in it.
            carried = ['cabinet:' + g.DRAWER, 'cabinet:' + g.HANDLE]
            m.carry(carried, ['cabinet:cabinet.housing', 'pen:pen'])
            try:
                # Drawer +Y points into the cabinet; opening grows along -Y.
                yield from m.move_to_pose_with_screw(self.stage, g.translated(tcp, -drawer[:3, 1] * step))
            finally:
                m.release(carried, ['cabinet:cabinet.housing', 'pen:pen'])
            moved = self.opening() - start
            segments = max(1, segments - 1)
            if abs(moved - step) > SLIP_TOLERANCE:
                if self.regrasps >= MAX_REGRASPS:
                    raise TaskFailed(self.stage, 'drawer did not follow the hand', moved=moved, commanded=step)
                self.regrasps += 1
                yield from self.release_handle(f'{label} regrasp {self.regrasps}')
                yield from self.grasp_handle(f'{label} regrasp {self.regrasps}')

    # -- task --------------------------------------------------------------------
    def solve(self):
        m = self.motion
        self.enter('settle')
        yield from m.hold(.5)

        # 1. Open the drawer.
        yield from self.grasp_handle('open')
        yield from self.slide('open', g.OPENING)
        yield from self.release_handle('open')

        # Rise clear of the opened drawer's walls before the free-space move, so
        # the path to the pen does not skim them.
        self.enter('pick: rise')
        released = m.tcp()
        yield from self.first_reachable(
            RISES, lambda h: m.move_to_pose_with_screw(self.stage, g.translated(released, [0., 0., h])),
            ('joint_limit', 'collision'))

        # 2. Pick the pen at its centre of mass.
        _, _, pen = self.poses()
        com = np.linalg.inv(pen) @ np.r_[self.env.center_of_mass('pen:'), 1.]
        grasp_x = float(com[0])
        m.allow(['pen:pen'])
        self.enter('pick: reach pen')
        yield from m.move_to_pose_with_RRTConnect(
            self.stage, g.standoff(g.pen_tcp(pen, self.table_height, grasp_x), g.PEN_APPROACH))
        self.enter('pick: approach pen')
        _, _, pen = self.poses()
        yield from m.move_to_pose_with_screw(self.stage, g.pen_tcp(pen, self.table_height, grasp_x))
        self.enter('pick: close on pen')
        yield from m.close_gripper()
        gap = m.finger_gap()
        if not PEN_HELD_GAP[0] <= gap <= PEN_HELD_GAP[1]:
            raise TaskFailed(self.stage, 'pen not held', finger_gap=gap)
        _, _, pen = self.poses()
        m.attach('pen:pen', pen)
        self.enter('pick: lift pen')
        grasped = m.tcp()
        yield from self.first_reachable(
            PEN_LIFTS, lambda h: m.move_to_pose_with_screw(self.stage, g.translated(grasped, [0., 0., h])),
            ('joint_limit',))
        yield from m.hold(.2)
        _, _, pen = self.poses()
        if pen[2, 3] < self.table_height + .03:
            raise TaskFailed(self.stage, 'pen not lifted', pen_height=float(pen[2, 3]))
        # The carried geometry follows the pen as it actually sits in the hand.
        m.detach('pen:pen')
        m.world.set_pose('pen:pen', pen)
        tcp_pen = m.attach('pen:pen', pen)
        tilt = float(np.degrees(np.arcsin(min(1., abs(pen[2, 0])))))
        self.events[-1]['pen_tilt_deg'] = tilt

        # 3. Put it into the drawer: carry above the opening, then lower.
        def release_target():
            drawer, housing, _ = self.poses()
            face = self.cabinet.housing_face_in_drawer(housing, drawer)
            release = g.drawer_release_point(drawer, self.cabinet.interior, face, self.cabinet.wall_top)
            return g.place_tcp(release, tcp_pen, drawer[:3, 0], PLACE_TILT, drawer[:3, 1])

        # A straight carry turns the hand only about the pen axis, so the pen
        # stays level above the drawer walls; RRTConnect is the fallback.
        self.enter('place: carry pen')
        above = g.translated(release_target(), [0., 0., g.PLACE_ABOVE])
        try:
            yield from m.move_to_pose_with_screw(self.stage, above)
        except PlanningFailed:
            self.enter('place: carry pen (RRTConnect)')
            yield from m.move_to_pose_with_RRTConnect(self.stage, above)
        self.enter('place: lower pen')
        yield from m.move_to_pose_with_screw(self.stage, release_target())
        self.enter('place: release pen')
        yield from m.open_gripper(.05)
        m.detach('pen:pen')
        yield from m.hold(.5)
        self.enter('place: withdraw')
        released = m.tcp()
        # Back along the approach, or straight up when the wrist is at a limit.
        exits = [g.standoff(released, d) for d in STANDOFFS] + [g.translated(released, [0., 0., h]) for h in STANDOFFS]
        yield from self.first_reachable(exits, lambda target: m.move_to_pose_with_screw(self.stage, target),
                                        ('joint_limit', 'collision'))
        m.allow(['pen:pen'], False)
        drawer, _, pen = self.poses()
        inside, local = self.cabinet.pen_inside(drawer, pen)
        if not inside:
            raise TaskFailed(self.stage, 'pen not in the drawer', pen_in_drawer=local.tolist())

        # 4. Close the drawer.
        yield from self.grasp_handle('close')
        yield from self.slide('close', 0.)
        yield from self.release_handle('close')
        if self.opening() > CLOSED_TOLERANCE:
            raise TaskFailed('close', 'drawer not closed', opening=self.opening())


def evaluate(solver):
    """Final task state from the physics state alone."""
    drawer, _, pen = solver.poses()
    inside, local = solver.cabinet.pen_inside(drawer, pen)
    tcp = solver.motion.tcp()
    hand_clear = float(np.linalg.norm(tcp[:3, 3] - (drawer[:3, :3] @ solver.cabinet.handle_center + drawer[:3, 3])))
    return {'drawer_opening_m': solver.opening(), 'drawer_closed': solver.opening() <= CLOSED_TOLERANCE,
            'pen_in_drawer': inside, 'pen_in_drawer_frame_m': local.tolist(),
            'hand_to_handle_m': hand_clear, 'hand_withdrawn': hand_clear > .05}
