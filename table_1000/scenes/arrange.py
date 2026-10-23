"""Arrangement helpers shared by scene sources: standing things in holders, dropping things into
containers and cabinet drawers, strewing them over the desk. Each takes the scene's ``Layout`` and
draws randomness only from ``layout.rng``."""

import math

import numpy as np

from table_1000.scenes.geometry import simulator_name
from table_1000.scenes.layout import SamplingFailed, yaw_pose

ACROSS = math.pi / 2     # a row along X of things turned with their long axis along Y
UPRIGHT = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])     # asset X -> world Z


def tilted(rng, low, high):
    """Rotation of a long thing standing in a holder: its X axis (its length) up, leaning by an
    angle in [low, high] degrees toward a random side."""
    lean = math.radians(rng.uniform(low, high))
    side = rng.uniform(-math.pi, math.pi)
    c, s = math.cos(side), math.sin(side)
    turn = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    tip = np.array([[1., 0., 0.], [0., math.cos(lean), -math.sin(lean)], [0., math.sin(lean), math.cos(lean)]])
    return turn @ tip @ turn.T @ UPRIGHT


def stand_in(layout, names, holder, spots, lean=(4., 12.)):
    """Stand ``names`` upright in ``holder``, one at each of ``spots`` (x, y in the holder's frame),
    leaning by ``lean`` degrees."""
    if len(names) > len(spots):
        raise ValueError(f'{len(names)} things for {len(spots)} spots in {holder}')
    for name, spot in zip(names, spots):
        jitter = layout.rng.uniform(-.004, .004, 2)
        layout.drop(name, np.add(spot, jitter), rotation=tilted(layout.rng, *lean), frame=holder)


def into(layout, names, container, half):
    """Drop ``names`` at random in the square ``half`` (a value, or half extents x, y) around
    ``container``'s middle, onto its floor or onto each other."""
    hx, hy = np.broadcast_to(half, 2)
    for name in names:
        layout.scatter(name, ((-hx, -hy), (hx, hy)), yaw=(-math.pi, math.pi), frame=container,
                       onto=[container, *names])


def stand(layout, spots, yaw=0.):
    """Things at fixed spots ``{name: (x, y)}`` on the table, square to it (or turned by ``yaw``)."""
    for name, xy in spots.items():
        layout.place(name, (xy, xy), yaw=yaw)


def strew(layout, names, region, onto=('table',)):
    """Drop ``names`` in random order and yaw over ``region``, landing on any of ``onto``."""
    for name in layout.rng.permutation(list(names)):
        layout.scatter(name, region, yaw=(-math.pi, math.pi), onto=list(onto))


def footprint_area(layout, name):
    """Area of ``name``'s collision box from above, as modelled."""
    low, high = layout.geometry[name].bounds()
    return (high - low)[:2].prod()


def footprint(layout, name, yaw, rotation=None):
    """Extent ``(low, high)`` (x, y) about its origin of ``name`` resting at any yaw in ``yaw`` (a
    range), or turned by ``rotation`` (3x3) as given."""
    geometry = layout.geometry[name]
    if rotation is not None:
        points = geometry.points() @ np.asarray(rotation)[:2].T
        return points.min(0), points.max(0)
    points = geometry.points() @ geometry.stable_rotation(np.eye(3)).T
    if yaw[1] - yaw[0] > .5:
        reach = np.linalg.norm(points[:, :2], axis=1).max()
        return np.full(2, -reach), np.full(2, reach)
    turned = [points[:, :2] @ yaw_pose(0., 0., 0., angle)[:2, :2].T for angle in np.linspace(*yaw, 5)]
    return np.min([p.min(0) for p in turned], axis=0), np.max([p.max(0) for p in turned], axis=0)


def lying_rotation(layout, name):
    """The resting orientation of ``name`` on its side nearest its asset Z along X (a glue stick
    lying along a drawer's width)."""
    side = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])     # asset Z -> X
    return layout.geometry[name].stable_rotation(side)


class Drawer:
    """Drawer ``index`` (1 at the top) of drawer unit ``unit``, from its asset's geometry: the unit's
    rigid bodies are ``<kind>.housing`` and ``<kind>.drawer<i>``, the drawer's parts ``side1``,
    ``side2``, ``front`` and ``bottom``, its joint ``<kind>.drawer<i>.slide`` (open toward -Y)."""

    def __init__(self, layout, unit, index):
        geometry = layout.geometry[unit]
        joint = next(joint for joint in geometry.joints.values() if joint['body2'].endswith(f'.drawer{index}'))
        body = joint['body2']
        self.joint, self.travel = simulator_name(joint['name']), joint['limits'][1]
        left, right = sorted((geometry.bounds(f'{body}.side{i}') for i in (1, 2)), key=lambda box: box[0][0])
        self.x = (left[1][0], right[0][0])                                # between the drawer's sides
        self.front = geometry.bounds(f'{body}.front')[1][1]               # inner face of the front, closed
        self.mouth = geometry.bounds()[0][1]          # the front of the closed unit: the handles of the drawers above
        self.floor = geometry.bounds(f'{body}.bottom')[1][2]


def drawer_contents(layout, drawer, names, opening=0., yaw=(-math.pi, math.pi), lying=(), cabinet='cabinet'):
    """Drop ``names`` into drawer ``drawer`` (1 at the top) of the drawer unit ``cabinet`` pulled
    out fully, turned by ``yaw`` in the unit's frame, then slide it to ``opening`` (m) with them:
    things can only be put into an open drawer, onto the part of its floor in front of the handles
    of the drawers above.
    Those in ``lying`` (too tall to stand in a drawer) lie along the drawer's width. The largest go
    in first; nothing rests on anything else, so the drawer closes."""
    spec = Drawer(layout, cabinet, drawer)
    layout.set_joint(cabinet, spec.joint, spec.travel)
    front, back = spec.front - spec.travel, spec.mouth
    floor = layout.poses[cabinet][2, 3] + spec.floor

    def fill():
        for name in sorted(names, key=lambda name: footprint_area(layout, name), reverse=True):
            rotation = lying_rotation(layout, name) if name in lying else None
            for turn in (yaw, (-.15, .15)):     # too long to turn any way: along the drawer's width
                low, high = footprint(layout, name, turn, rotation)
                region = ((spec.x[0] + .003 - low[0], front + .003 - low[1]),
                          (spec.x[1] - .003 - high[0], back - .003 - high[1]))
                if region[0][0] <= region[1][0] and region[0][1] <= region[1][1]:
                    break
            else:
                raise ValueError(f'{name} does not fit in drawer {drawer} of {cabinet}')
            for _ in range(20):     # on the drawer's floor, not on a handle of the drawer above
                layout.scatter(name, region, yaw=turn, rotation=rotation, frame=cabinet, onto=cabinet)
                if abs(layout.world_points(name)[:, 2].min() - floor) < .001:
                    break
                layout.remove(name)
            else:
                raise SamplingFailed(f'{name} did not land in drawer {drawer}')

    for tries_left in reversed(range(10)):      # a crowded drawer: start it over when the last things find no room
        try:
            fill()
            break
        except SamplingFailed:
            for name in names:
                layout.remove(name)
            if not tries_left:
                raise
    frame = layout.frame(cabinet)
    shift = frame @ yaw_pose(0., spec.travel - opening, 0.) @ np.linalg.inv(frame)
    for name in names:
        layout.set_pose(name, shift @ layout.poses[name], layout.supports[name], loose=True)
    layout.set_joint(cabinet, spec.joint, opening)


def line_up_in(layout, names, container, start, direction=0., yaw=0., gap=.01):
    """Like ``Layout.row`` but inside ``container``: ``names`` dropped one after another from
    ``start`` (x, y in the container's frame) along ``direction``, each turned by ``yaw`` from
    ``direction``, ``gap`` apart between their collision outlines. Returns where the line ends."""
    along = np.array([math.cos(direction), math.sin(direction)])
    position = np.asarray(start, dtype=float)
    for name in names:
        rotation = layout.geometry[name].stable_rotation(yaw_pose(0., 0., 0., direction + yaw)[:3, :3])
        extent = layout.geometry[name].points() @ rotation.T
        low, high = (extent[:, :2] @ along).min(), (extent[:, :2] @ along).max()
        origin = position - low * along
        layout.drop(name, origin, rotation=rotation, frame=container)
        position = origin + (high + gap) * along
    return position
