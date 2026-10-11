"""Build this capped pen as two independent top-level rigid-body subtrees."""

import bpy

from table_1000.modeling.asset_builders import capped_pen, generate, group_part

PROFILE = {'geometry': {'segments': 16,
              'seat_x_m': -0.0365,
              'barrel_end_x_m': 0.07,
              'barrel_radius_m': 0.0048,
              'neck_length_m': 0.014,
              'neck_radius_m': 0.0038,
              'nib_length_m': 0.008,
              'nib_tip_radius_m': 0.0007,
              'nib_base_radius_m': 0.0026,
              'rear_start_x_m': 0.069,
              'rear_end_x_m': 0.0735,
              'rear_radius_m': 0.0048,
              'cap_radius_m': 0.0055,
              'cap_length_m': 0.045,
              'end_thickness_m': 0.001,
              'lip_length_m': 0.002,
              'shell_inner_radius_m': 0.0047,
              'rigid_lip_clearance_m': 0.0002},
 'barrel_color': [0.24, 0.43, 0.6],
 'cap_color': [0.61, 0.75, 0.82],
 'clip_color': [0.24, 0.43, 0.6],
 'rear_color': [0.78, 0.8, 0.78],
 'cap_transmission': 0.48,
 'rear_transmission': 0,
 'clip_dimensions_m': [0.033, 0.0018, 0.0018],
 'clip_position_m': [-0.025, 0.0015, 0.0058],
 'body_mass_kg': 0.006,
 'cap_mass_kg': 0.0015,
 'metal_point': True}


def build():
    capped_pen("pen/000005", PROFILE)

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    owner = bpy.data.objects['body']
    owner.name = 'body'
    for component, names in [
        ('barrel', ['Barrel']),
        ('rear_plug', ['Rear plug']),
        ('neck', ['Neck']),
        ('tip', ['Nib']),
        ('point', ['Fine point']),
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
        ('clip', ['Cap clip']),
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
