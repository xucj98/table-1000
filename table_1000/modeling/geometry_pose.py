"""Explicit, fixed-topology stage-one deformation driven by the source snapshot.

Like building an asset, previewing a deformable asset executes its trusted
object.py. No handlers, drivers, scene-embedded Python or dynamics are used.
"""

import json
import math
import runpy

import bpy
import numpy as np


class GeometryPose:
    def __init__(self, blend, checks):
        self.definitions = {}
        self.meshes = []
        self.callback = None
        encoded = bpy.context.scene.get('geometry_controls')
        if not checks.deformables:
            if encoded is not None:
                raise ValueError('geometry_controls requires a deformable owner')
            return
        definitions = json.loads(encoded) if isinstance(encoded, str) else None
        if not isinstance(definitions, dict) or not definitions:
            raise ValueError('Deformable assets require scene geometry_controls JSON')
        owners = {o.name for o in checks.deformables}
        for name, definition in definitions.items():
            if not isinstance(name, str) or not name or not isinstance(definition, dict):
                raise ValueError('Invalid geometry control definition')
            if set(definition) != {'owner', 'limits', 'unit'} or definition['owner'] not in owners:
                raise ValueError(f'Invalid geometry control owner or fields: {name}')
            limits = definition['limits']
            if (not isinstance(limits, list) or len(limits) != 2 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in limits) or
                    not limits[0] <= 0 <= limits[1] or limits[0] == limits[1]):
                raise ValueError(f'Control limits must be finite, increasing and include zero: {name}')
            if definition['unit'] not in {'m', 'rad', '1'}:
                raise ValueError(f'Unsupported control unit: {name}')
        if {d['owner'] for d in definitions.values()} != owners:
            raise ValueError('Every deformable owner needs at least one control')
        self.definitions = definitions
        for obj in checks.visuals + checks.colliders:
            if checks.owner[obj.name] not in owners:
                continue
            coordinates = np.empty(len(obj.data.vertices) * 3, dtype=np.float32)
            obj.data.vertices.foreach_get('co', coordinates)
            topology = tuple(tuple(face.vertices) for face in obj.data.polygons)
            self.meshes.append((obj, coordinates, topology, obj.parent))
        source = runpy.run_path(str(blend.parent / 'object.py'))
        self.callback = source.get('apply_geometry_pose')
        if not callable(self.callback):
            raise ValueError('Deformable object.py must define apply_geometry_pose(controls)')

    def apply(self, values):
        unknown = values.keys() - self.definitions.keys()
        if unknown:
            raise ValueError(f'Unknown geometry controls: {sorted(unknown)}')
        complete = {name: values.get(name, 0.0) for name in self.definitions}
        for name, value in complete.items():
            lo, hi = self.definitions[name]['limits']
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or not lo <= value <= hi):
                raise ValueError(f'{name}={value} outside [{lo}, {hi}]')
        if self.callback is None:
            return complete
        # Always start from saved mesh coordinates, including after an open pose.
        for obj, coordinates, _, _ in self.meshes:
            obj.data.vertices.foreach_set('co', coordinates)
            obj.data.update()
        self.callback(complete)
        bpy.context.view_layer.update()
        for obj, coordinates, topology, parent in self.meshes:
            if (obj.parent != parent or len(obj.data.vertices) * 3 != len(coordinates) or
                    tuple(tuple(face.vertices) for face in obj.data.polygons) != topology):
                raise ValueError(f'Geometry control changed hierarchy or topology: {obj.name}')
            current = np.empty_like(coordinates)
            obj.data.vertices.foreach_get('co', current)
            if not np.isfinite(current).all():
                raise ValueError(f'Nonfinite deformed vertices: {obj.name}')
            if obj.modifiers or obj.data.shape_keys:
                raise ValueError(f'Geometry control requires explicit vertices: {obj.name}')
        return complete
