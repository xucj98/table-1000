"""Scalar trace sources and one shared-time figure per physical test."""

import csv
import math
from pathlib import Path


def source_parts(source):
    category, rest = source.split('.', 1)
    if category in ('rigid_bodies', 'actions'):
        parts = rest.rsplit('.', 2)
        vectors = {'position': 'xyz', 'quaternion': 'wxyz', 'linear_velocity': 'xyz',
                   'angular_velocity': 'xyz'} if category == 'rigid_bodies' else {
                       'force': 'xyz', 'torque': 'xyz', 'point': 'xyz', 'position': 'xyz',
                       'rotation': 'wxyz', 'position_error': 'xyz', 'rotation_error': 'xyz'}
        if len(parts) == 3 and parts[1] in vectors and parts[2] in vectors[parts[1]]:
            return category, parts[0], parts[1], parts[2]
        if category == 'actions':
            name, field = rest.rsplit('.', 1)
            if field in ('active', 'target_position'):
                return category, name, field, ''
    elif category in ('joints', 'behaviors'):
        name, field = rest.rsplit('.', 1)
        if category == 'behaviors' or field in ('position', 'velocity'):
            return category, name, field, ''
    raise ValueError(f'Unknown scalar plot source: {source}')


def plot_unit(source, values, model, actions):
    category, name, field, component = source_parts(source)
    if category == 'joints' or category == 'actions' and field == 'target_position':
        joint_name = name if category == 'joints' else actions[name]['joint']
        joint = next(item for item in model['joints'] if item['name'] == joint_name)
        unit = ('m' if joint['type'] == 'SLIDER' else 'rad') + ('/s' if field == 'velocity' else '')
    else:
        unit = {'position': 'm', 'point': 'm', 'position_error': 'm',
                'linear_velocity': 'm/s', 'angular_velocity': 'rad/s',
                'force': 'N', 'torque': 'N·m', 'rotation_error': 'rad',
                'opening': 'm', 'radial_error': 'm', 'retention_force': 'N'}.get(field, '1')
    if unit == 'm' and max((abs(value) for value in values if math.isfinite(value)), default=0) < .1:
        return 'mm', 1000
    return unit, 1


def plots(rows, sources, output, model, actions):
    """CSV remains SI; only plotted lengths adapt to metres or millimetres."""
    destination = Path(output) / 'plots.jpg'
    destination.unlink(missing_ok=True)
    if not sources:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(len(sources), 1, sharex=True, squeeze=False,
                                figsize=(12, 2.5 * len(sources)))
    times = [float(row['time']) for row in rows]
    try:
        for axis, source in zip(axes[:, 0], sources):
            values = [float(row[source]) if row.get(source, '') != '' else math.nan for row in rows]
            if not any(source in row for row in rows):
                raise ValueError(f'Plot source missing from trace: {source}')
            unit, scale = plot_unit(source, values, model, actions)
            axis.plot(times, [value * scale for value in values], label=source)
            axis.set_ylabel(unit)
            axis.legend(loc='best', fontsize=9)
            axis.grid(alpha=.25)
        axes[-1, 0].set_xlabel('Simulation time (s)')
        figure.tight_layout()
        figure.savefig(destination, dpi=120, pil_kwargs={'quality': 95})
    finally:
        plt.close(figure)


def plot_trace(trace, sources, model, actions):
    with Path(trace).open() as stream:
        rows = list(csv.DictReader(stream))
    plots(rows, sources, Path(trace).parent, model, actions)
