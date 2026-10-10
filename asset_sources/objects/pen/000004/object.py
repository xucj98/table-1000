"""Build this capped pen as two independent top-level rigid-body subtrees."""

import bpy

from table_1000.modeling.asset_builders import capped_pen, generate, group_part

PROFILE = {'geometry': {'segments': 16,
              'seat_x_m': -0.032,
              'barrel_end_x_m': 0.086,
              'barrel_radius_m': 0.009,
              'neck_length_m': 0.026,
              'neck_radius_m': 0.0078,
              'nib_length_m': 0.013,
              'nib_tip_radius_m': 0.0022,
              'nib_base_radius_m': 0.0042,
              'cap_radius_m': 0.01,
              'cap_length_m': 0.061,
              'end_thickness_m': 0.0015,
              'lip_length_m': 0.003,
              'shell_inner_radius_m': 0.0086,
              'rigid_lip_clearance_m': 0.0002,
              'label_length_m': 0.046},
 'barrel_color': [0.48, 0.012, 0.018],
 'cap_color': [0.67, 0.7, 0.69],
 'cap_transmission': 0.48,
 'body_mass_kg': 0.012,
 'cap_mass_kg': 0.003,
 'metal_point': False,
 'label_color': [0.82, 0.82, 0.73]}


def build():
    capped_pen("pen/000004", PROFILE)

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    owner = bpy.data.objects['body']
    owner.name = 'body'
    for component, names in [
        ('barrel', ['Barrel']),
        ('label', ['Pale paper label']),
        ('neck', ['Neck']),
        ('tip', ['Nib']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)
    owner = bpy.data.objects['cap']
    owner.name = 'cap'
    for component, names in [
        ('shell', ['Cap shell']),
        ('lip', ['Cap lip']),
        ('end', ['Cap end']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
