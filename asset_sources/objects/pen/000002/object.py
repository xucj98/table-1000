"""Blue capped pen with three native radial detent balls.

Detent geometry follows the Claude native-contact contribution (c1d058f)
and its frozen research implementation (519d6df). External styling is retained.
"""
import math
import bpy
from mathutils import Quaternion
from table_1000.modeling.asset_builders import body, capped_pen, generate, group_part, joint, mesh, part, rigid_settings

PROFILE = {'geometry': {'segments': 24,
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
              'rigid_lip_clearance_m': 0.00065,
              'label_length_m': 0.046},
 'barrel_color': [0.008, 0.014, 0.09],
 'cap_color': [0.67, 0.7, 0.69],
 'cap_transmission': 0.48,
 'body_mass_kg': 0.012,
 'cap_mass_kg': 0.003,
 'metal_point': False,
 'label_color': [0.82, 0.82, 0.73]}


DETENT = {
    'balls': 3,                 # equally spaced around the axis
    'ball_r': 0.0008,
    'ball_mass': 0.001,
    'ball_x': -0.0015,          # ball centre in the cap frame (behind the mouth)
    'ball_clearance': 0.00005,  # ball surface to neck at rest
    'travel': 0.0008,           # outward joint range
    'ridge_height': 0.00045,    # above the neck radius
    'lead_angle_deg': 25.0,     # flank met while closing
    'retain_angle_deg': 45.0,   # flank met while pulling out
    'ridge_flat': 0.0003,
    'seated_play': 0.0001,      # axial play between ball and ridge when seated
    'lip_inner_r': 0.00845,     # clears the ridge
}

def ring(x, r, n, phase=0.0):
    return [(x, r * math.cos(phase + i * math.tau / n), r * math.sin(phase + i * math.tau / n)) for i in range(n)]


def revolve(profile, n, closed=True):
    """Closed mesh of revolution from (x, r) rings; convex when r(x) is concave."""
    vertices, faces = [], []
    for x, r in profile:
        vertices += ring(x, r, n)
    for k in range(len(profile) - 1):
        a, b = k * n, (k + 1) * n
        faces += [(a + i, a + (i + 1) % n, b + (i + 1) % n, b + i) for i in range(n)]
    if closed:
        faces.append(tuple(reversed(range(n))))
        last = (len(profile) - 1) * n
        faces.append(tuple(last + i for i in range(n)))
    return vertices, faces


def profile_sector(profile, inner, n, i):
    """Convex sector of a solid of revolution between `inner` and an (x, r) profile.

    Profile radii are exact on the sector's two bounding azimuths, where the
    detent balls are placed; cooking never has more than 2 * (len + 2) vertices.
    """
    angles = [i * math.tau / n, (i + 1) * math.tau / n]
    outline = [(profile[0][0], inner), *profile, (profile[-1][0], inner)]
    vertices = [(x, r * math.cos(a), r * math.sin(a)) for a in angles for x, r in outline]
    m = len(outline)
    faces = [tuple(range(m))[::-1], tuple(range(m, 2 * m))]
    faces += [(k, (k + 1) % m, m + (k + 1) % m, m + k) for k in range(m)]
    return vertices, faces


def ridge_profile(p, d):
    """(x, r) rings of the neck ridge, placed so the seated ball has the requested play."""
    r0, h = p['neck_r'], d['ridge_height']
    lead = h / math.tan(math.radians(d['lead_angle_deg']))
    retain = h / math.tan(math.radians(d['retain_angle_deg']))
    centre_x = p['seat_x'] + d['ball_x']
    centre_r = r0 + d['ball_clearance'] + d['ball_r']
    # The retention flank is the line through (end, r0) rising toward -X.
    # Place `end` so the ball centre is ball_r away after moving by seated_play.
    s = math.radians(d['retain_angle_deg'])
    # signed distance from centre (moved by -play) to the flank line
    # line normal pointing to +X/+r: (sin s, cos s); line passes (end, r0)
    cx = centre_x - d['seated_play']
    end = cx - (d['ball_r'] - (centre_r - r0) * math.cos(s)) / math.sin(s)
    return [(end - retain - d['ridge_flat'] - lead, r0), (end - retain - d['ridge_flat'], r0 + h),
            (end - retain, r0 + h), (end, r0)]



def build():
    capped_pen(PROFILE)
    for root_name, part_name, visual_name in [
        ("body", "barrel", "Barrel"), ("body", "label", "Pale paper label"),
        ("body", "neck", "Neck"), ("body", "tip", "Nib"),
        ("cap", "shell", "Cap shell"), ("cap", "lip", "Cap lip"), ("cap", "end", "Cap end"),
    ]:
        owner = bpy.data.objects[root_name]
        visual = bpy.data.objects[visual_name]
        collisions = sorted([obj for obj in owner.children if obj.name == "Collision_" + visual_name
                             or obj.name.startswith("Collision_" + visual_name + "_")], key=lambda obj: obj.name)
        group_part(root_name + "." + part_name, owner, [visual], collisions)

    pen, cap = bpy.data.objects['body'], bpy.data.objects['cap']
    g = PROFILE['geometry']
    p = {'neck_r': g['neck_radius_m'], 'seat_x': g['seat_x_m']}
    ridge = ridge_profile(p, DETENT)
    node = part('body.detent_ridge', pen)
    surface = bpy.data.materials['Barrel plastic']
    mesh('body.detent_ridge.visual', *revolve(ridge, g['segments']), node, surface)
    for i in range(g['segments']):
        mesh(f'body.detent_ridge.collision{i+1}',
             *profile_sector(ridge, p['neck_r'] - .001, g['segments'], i), node,
             shape='CONVEX_HULL')
    radius = p['neck_r'] + DETENT['ball_clearance'] + DETENT['ball_r']
    for i in range(1, 4):
        phi = (i-1) * math.tau / 3
        local = (DETENT['ball_x'], radius*math.cos(phi), radius*math.sin(phi))
        ball = body(f'cap_ball{i}', None)
        ball.location = (p['seat_x']+local[0], local[1], local[2])
        ball.rigid_body.mass = DETENT['ball_mass']
        node = part(f'cap_ball{i}.ball', ball)
        bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=DETENT['ball_r'])
        collider = bpy.context.object
        collider.name = f'cap_ball{i}.ball.collision'
        collider.parent = node
        collider.location = (0,0,0)
        collider['geometry_role'] = 'collision'
        collider.hide_render = True
        rigid_settings(collider, 'SPHERE')
        frame = Quaternion((1,0,0), phi) @ Quaternion((0,0,1), math.pi/2)
        slider = joint(f'cap.detent{i}', 'SLIDER', cap, ball,
                       ball.location, (0, DETENT['travel']))
        slider.rotation_mode = 'QUATERNION'
        slider.rotation_quaternion = frame
        slider.rigid_body_constraint.disable_collisions = True


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
