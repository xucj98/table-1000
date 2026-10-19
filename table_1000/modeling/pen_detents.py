"""Pen-only native follower/ridge geometry. Importing needs no Blender.

Consumer supplies dimensions and local part names; no catalog IDs or scene
paths. Equivalent internal mechanism, not measured plastic deformation.
"""


def add_detents(d):
    import math
    import bpy
    from mathutils import Vector
    from table_1000.modeling.asset_builders import body, part, mesh, joint, rigid_settings
    pen, cap = bpy.data.objects['body'], bpy.data.objects['cap']
    surface = bpy.data.materials['Barrel plastic']
    ball_x, clearance, play = -.001, .00005, .00015
    n = 24 if not d['flat'] else 4
    # Widen only the internal mouth enough to clear the new ridge. External
    # dimensions and the rest of the hollow cap stay unchanged.
    required = d['neck'] + d['height'] + .00015
    z_scale = 15/22 if d['flat'] else 1.
    lip = bpy.data.objects['cap.lip']
    for obj in lip.children:
        if obj.type != 'MESH':
            continue
        for v in obj.data.vertices:
            y, z = v.co.y, v.co.z/z_scale
            radius = math.hypot(y, z)
            angle = math.atan2(z, y)
            section = ((math.cos(angle)**4 + math.sin(angle)**4)**-.25) if d['flat'] else 1.
            if radius < (d['neck']+.0008)*section:
                minimum_vertex_radius = required*section/math.cos(math.pi/d['segments'])
                scale = max(1., minimum_vertex_radius/radius)
                v.co.y *= scale; v.co.z *= scale
        obj.data.update()
    for i in range(d['balls']):
        phi = i*math.tau/d['balls']
        radial = Vector((0, math.cos(phi), math.sin(phi)))
        tangent = Vector((0, -math.sin(phi), math.cos(phi)))
        neck = d['neck']*(z_scale if d['flat'] and i % 2 else 1.)
        center_r = neck+clearance+d['radius']
        h, r = d['height'], d['radius']
        end = d['seat']+ball_x-play-(r-(center_r-neck)*math.cos(math.pi/4))/math.sin(math.pi/4)
        profile = [(end-h-.0002-h/math.tan(math.radians(25)), neck),
                   (end-h-.0002, neck+h), (end-h, neck+h), (end, neck)]
        # A local convex ramp under each follower; no full circular ridge on
        # a flattened cap. Tangential width leaves the existing neck intact.
        width = .0015 if d['flat'] else .0012
        outline = [(profile[0][0], neck-.0003), *profile, (profile[-1][0], neck-.0003)]
        vertices = [tuple(Vector((x, 0, 0))+radial*rr+tangent*tt)
                    for tt in (-width/2, width/2) for x, rr in outline]
        count = len(outline)
        faces = [tuple(reversed(range(count))), tuple(range(count, 2*count))]
        faces += [(k, (k+1)%count, count+(k+1)%count, count+k) for k in range(count)]
        if d['flat']:
            node = part('body.detent_ramp'+str(i+1), pen)
            mesh(node.name+'.visual', vertices, faces, node, surface)
            mesh(node.name+'.collision', vertices, faces, node, shape='CONVEX_HULL')
        elif i == 0:
            node = part('body.detent_ridge', pen)
            for sector in range(n):
                angles = [sector*math.tau/n, (sector+1)*math.tau/n]
                vertices = [(x, rr*math.cos(angle), rr*math.sin(angle))
                            for angle in angles for x, rr in outline]
                mesh(node.name+'.visual'+str(sector+1), vertices, faces, node, surface)
                mesh(node.name+'.collision'+str(sector+1), vertices, faces, node, shape='CONVEX_HULL')
        ball = body('cap_ball'+str(i+1), None)
        ball.location = Vector((d['seat']+ball_x, 0, 0))+radial*center_r
        ball.rigid_body.mass = d['mass']
        node = part(ball.name+'.ball', ball)
        bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=8, radius=r)
        collider = bpy.context.object
        collider.name = node.name+'.collision'
        collider.parent = node
        collider.location = (0, 0, 0)
        collider['geometry_role'] = 'collision'
        collider.hide_render = True
        rigid_settings(collider, 'SPHERE')
        slider = joint('cap.detent'+str(i+1), 'SLIDER', cap, ball, ball.location, (0, d['travel']))
        slider.rotation_mode = 'QUATERNION'
        slider.rotation_quaternion = Vector((1, 0, 0)).rotation_difference(radial)
        slider.rigid_body_constraint.disable_collisions = True


def split_click(*, axis_height, anchor_x, shaft_rear_x):
    """Separate the existing button from the refill; trim only the hidden tail.

    Both sliders retain [-3, 0] mm travel. The unchanged rest geometry is the
    retracted-tip configuration. Button and refill move independently.
    """
    import bpy
    from table_1000.modeling.asset_builders import body, joint
    refill = bpy.data.objects['pen.refill']
    button = body('pen.button', refill.parent)
    component = bpy.data.objects['pen.refill.button']
    bpy.context.view_layer.update()
    world = component.matrix_world.copy()
    component.parent = button
    component.matrix_world = world
    component.name = 'pen.button.button'
    for obj in component.children:
        obj.name = obj.name.replace('pen.refill.button.', 'pen.button.button.')
    # At full button compression its front must clear the retracted shaft.
    # Map the shaft interval affinely, preserving the tip-side endpoint.
    for obj in bpy.data.objects['pen.refill.shaft'].children:
        if obj.type != 'MESH':
            continue
        local_to_world = obj.matrix_world
        inverse = local_to_world.inverted()
        low = min((local_to_world @ v.co).x for v in obj.data.vertices)
        high = max((local_to_world @ v.co).x for v in obj.data.vertices)
        for vertex in obj.data.vertices:
            point = local_to_world @ vertex.co
            point.x = low + (point.x-low)*(shaft_rear_x-low)/(high-low)
            vertex.co = inverse @ point
        obj.data.update()
    press = bpy.data.objects['button_press']
    press.rigid_body_constraint.object2 = button
    # Native articulation suppresses collisions between directly joined bodies.
    # Clearances are also checked geometrically at all independent endpoints.
    press.rigid_body_constraint.disable_collisions = True
    slide = joint('refill_slide', 'SLIDER', bpy.data.objects['pen.shell'], refill,
                  (anchor_x, 0, axis_height), (-.003, 0))
    slide.rigid_body_constraint.disable_collisions = True
