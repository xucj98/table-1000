"""X1Pro facing a wooden organizer, a three-drawer cabinet and 11 stationery items.

The table and robot start match scene 000000. Storage stays at the rear;
stationery is sampled in the front workspace. All dimensions are metres.
The scene can be built before its separate expert program is implemented.
"""

import math

import numpy as np

from table_1000.scenes.layout import SamplingFailed, yaw_pose

# The cap detents in the selected pen assets require 1 ms integration.
PHYSICS_DT = .001

TABLE = {'size': (1.36, 1.02), 'height': .755, 'top_thickness': .03, 'leg_size': .05,
         'center': (0., 0.), 'friction': (.5, .4)}
ROBOT = {'asset': 'robots/x1pro', 'position': (-.03, -.80, 0.), 'yaw': math.pi / 2,
         'joints': {'lift_joint': .40, 'head_pitch_joint': .30}}

OBJECTS = {
    'organizer': {'asset': 'objects/cabinet/000001'},
    'cabinet': {'asset': 'objects/cabinet/000000'},
    'highlighter': {'asset': 'objects/pen/000007'},
    'red_marker': {'asset': 'objects/pen/000003'},
    'fine_pen': {'asset': 'objects/pen/000010'},
    'white_marker': {'asset': 'objects/pen/000011'},
    'ballpoint': {'asset': 'objects/pen/000001'},
    'tape': {'asset': 'objects/tape/000000'},
    'stapler': {'asset': 'objects/stapler/000000'},
    'glue_stick': {'asset': 'objects/glue/000001'},
    'glue_bottle': {'asset': 'objects/glue/000000'},
    'staples': {'asset': 'objects/staple/000000'},
    'knife': {'asset': 'objects/utility_knife/000000'},
}

VIEWS = {
    'wide': {'position': (1.55, -1.55, 1.8), 'target': (0., .0, .95),
             'focal_length_mm': 22, 'resolution': (1280, 720)},
    'overhead': {'position': (0., -.10, 2.55), 'target': (0., .0, .755),
                 'focal_length_mm': 18, 'resolution': (1280, 720)},
}

# The entire collision footprint, not just the origin, must be inside this area.
WORKSPACE = ((-.50, -.45), (.50, -.015))
ITEM_CLEARANCE = .012
ITEMS = tuple(name for name in OBJECTS if name not in ('organizer', 'cabinet'))


def footprint(layout, name, transform):
    points = layout.geometry[name].points() @ transform[:3, :3].T + transform[:3, 3]
    return points.min(axis=0), points.max(axis=0)


def initialize(layout):
    """Place storage first, then sample separated, reproducible stationery poses.

    The cabinet rests on the organizer's actual left shelf, not its overall top.
    Capped pens retain the assembled pose stored in the asset. Glue containers
    lie on their sides; tape and staple strip keep their normal resting faces.
    Drawer and tool joints retain their asset rest values.
    """
    organizer = layout.resting_pose('organizer', (0., .25))
    layout.set_pose('organizer', organizer)

    shelf = layout.geometry['organizer'].points('cabinet.housing.raised_shelf')
    shelf = shelf @ organizer[:3, :3].T + organizer[:3, 3]
    low, high = shelf.min(axis=0), shelf.max(axis=0)
    cabinet = yaw_pose(*((low[:2] + high[:2]) / 2), 0.)
    cabinet[2, 3] = high[2] - layout.geometry['cabinet'].lowest(cabinet[:3, :3])
    layout.set_pose('cabinet', cabinet, on='organizer')

    occupied = []
    region_low, region_high = np.asarray(WORKSPACE)
    # Larger objects first makes the remaining free area easier to fill.
    ordered = sorted(ITEMS, key=lambda name: np.ptp(layout.geometry[name].points(), axis=0).max(), reverse=True)
    for name in ordered:
        geometry = layout.geometry[name]
        for _ in range(200):
            xy = layout.rng.uniform(region_low, region_high)
            angle = layout.rng.uniform(-math.pi, math.pi)
            transform = yaw_pose(*xy, 0., angle)
            if name in ('glue_stick', 'glue_bottle'):
                # Turn the asset's upright +Z axis into the tabletop plane.
                transform[:3, :3] = transform[:3, :3] @ np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
            transform[:3, :3] = geometry.stable_rotation(transform[:3, :3])
            transform[2, 3] = TABLE['height'] - geometry.lowest(transform[:3, :3])
            low, high = footprint(layout, name, transform)
            if np.any(low[:2] < region_low) or np.any(high[:2] > region_high):
                continue
            if any(np.all(high[:2] + ITEM_CLEARANCE > other_low) and
                   np.all(other_high + ITEM_CLEARANCE > low[:2])
                   for other_low, other_high in occupied):
                continue
            if layout.penetration(name, transform) > .001:
                continue
            layout.set_pose(name, transform)
            occupied.append((low[:2], high[:2]))
            break
        else:
            raise SamplingFailed(f'no separated tabletop pose for {name}')
