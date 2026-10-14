"""Simulator-independent scene helpers."""


def primitive_surface(spec, pose, count=400):
    """Points on the surface of a primitive (``{'shape', 'size'}``, as for :func:`add_primitive`) posed in the world."""
    import trimesh
    if spec['shape'] == 'sphere':
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=spec['size'])
    elif spec['shape'] == 'box':
        mesh = trimesh.creation.box(spec['size'])
    else:
        mesh = trimesh.creation.cylinder(radius=spec['size'][0], height=spec['size'][1])
    points, _ = trimesh.sample.sample_surface_even(mesh, count, seed=0)
    return points @ pose[:3, :3].T + pose[:3, 3]
