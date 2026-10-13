"""Five-component pegboard cabinet, in meters; no scene or texture dependencies."""
import math

import bpy
from mathutils import Vector

from table_1000.modeling.asset_builders import (
    BOX_FACES, asset, body, box, collider, cylinder, generate, group_part,
    joint, material, mesh,
)


def panel(name, dimensions, position, surface, owner, bevel=0):
    return box(name, dimensions, Vector(position) - owner.location, surface, owner, bevel=bevel)


def prism(name, contour, surface, owner):
    """Extrude a convex XZ contour through the 8 mm pegboard thickness."""
    n = len(contour)
    vertices = [(x, y, z) for y in (.097, .105) for x, z in contour]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2*n))]
    faces += [(i, (i+1) % n, (i+1) % n+n, i+n) for i in range(n)]
    visual = mesh(name, vertices, faces, owner, surface)
    collider(visual)
    return visual


def pegboard(surface, owner):
    # Preserve all twenty original octagonal bores exactly. Full-width strips
    # and three bands per hole row replace the individual cell collars.
    xs = [-.125, -.070, -.015, .040]
    zs = [.120, .175, .230, .285, .340]
    r = .005
    q = r * (math.sqrt(2) - 1)
    visuals = []
    strip_bottoms = [.070] + [z+r for z in zs]
    strip_tops = [z-r for z in zs] + [.380]
    for i, (lo, hi) in enumerate(zip(strip_bottoms, strip_tops), 1):
        visuals.append(panel(f"board_strip{i}", (.2225,.008,hi-lo),
                             (-.04875,.101,(lo+hi)/2), surface, owner))
    for row, z in enumerate(zs, 1):
        # At each band boundary, the five solid gaps share the exact aperture
        # vertices. Upper/lower gaps are convex trapezoids; middle gaps boxes.
        for band, (z0,z1,w0,w1) in enumerate([
            (z-r,z-q,q,r), (z-q,z+q,r,r), (z+q,z+r,r,q)
        ], 1):
            left0 = [-.160] + [x+w0 for x in xs]
            right0 = [x-w0 for x in xs] + [.0625]
            left1 = [-.160] + [x+w1 for x in xs]
            right1 = [x-w1 for x in xs] + [.0625]
            for gap, (a,b,c,d) in enumerate(zip(left0,right0,left1,right1), 1):
                name = f"board_row{row}_band{band}_gap{gap}"
                if band == 2:
                    visual = panel(name, (b-a,.008,z1-z0),
                                   ((a+b)/2,.101,(z0+z1)/2), surface, owner)
                else:
                    visual = prism(name, [(a,z0),(b,z0),(d,z1),(c,z1)], surface, owner)
                visuals.append(visual)
    visuals.append(panel("board_back",(.0975,.008,.310),(.11125,.101,.225),surface,owner))
    return visuals


def rod(name, a, b, radius, surface, owner):
    a,b=Vector(a),Vector(b)
    obj=cylinder(name,radius,(b-a).length,(a+b)/2-owner.location,surface,owner,axis="Z",segments=8)
    rotation=(b-a).to_track_quat('Z','Y').to_euler()
    obj.rotation_euler=rotation
    # cylinder() creates the matching collision object immediately.
    bpy.data.objects['Collision_'+obj.name].rotation_euler=rotation
    return obj


def mounting_pegs(owner, xs, hole_z, surface):
    # Inferred straight peg mounting: seated on the aperture bottom, removable
    # along -Y. Pads contact the board front; no permanent weld/constraint.
    visuals = []
    for i,x in enumerate(xs, 1):
        visuals.append(panel(f"{owner.name}_pad{i}",(.009,.0055,.010),(x,.09425,hole_z-.0035),surface,owner))
        visuals.append(panel(f"{owner.name}_peg{i}",(.003,.020,.003),(x,.101,hole_z-.0035),surface,owner))
    return visuals


def component(owner, name, objects):
    """One functional/material part, with a selectable visual and proxies."""
    proxies = [bpy.data.objects['Collision_'+o.name] for o in objects]
    node = group_part(owner.name+'.'+name, owner, objects, proxies)
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:
        bpy.context.view_layer.objects.active=o
        for modifier in list(o.modifiers):
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        o.select_set(True)
    bpy.context.view_layer.objects.active=objects[0]
    if len(objects) > 1:
        bpy.ops.object.join()
    objects[0].name=node.name+'.visual'
    for o in node.children:
        if o.get('geometry_role')=='collision':
            o.hide_select=True


def build():
    ivory=material('Ivory ABS',(.82,.83,.80),.35)
    smoke=material('Smoky clear PET',(.36,.40,.39),.20,transmission=.72)
    clear=material('Clear tray PET',(.64,.68,.67),.16,transmission=.85)
    root=asset('pegboard_cabinet','housing bottom center','+X width; +Y back; +Z up','-Y')
    housing=body(root.name+'.housing',root)
    for name,dim,pos in [
        ('bottom',(.320,.220,.006),(0,0,.003)),
        ('platform',(.320,.220,.010),(0,0,.068)),
        ('left_wall',(.006,.220,.064),(-.157,0,.035)),
        ('right_wall',(.006,.220,.064),(.157,0,.035)),
        ('back_wall',(.320,.006,.064),(0,.107,.035)),
        ('divider',(.006,.220,.064),(0,0,.035)),
    ]: component(housing,name,[panel(name,dim,pos,ivory,housing)])
    component(housing,'pegboard',pegboard(ivory,housing))
    # Reuse the current delivered three 25-degree cubbies, not the outdated
    # vertical pen partitions in the earlier scene Blender source.
    for name,dim,pos in [
        # The old holder_back was entirely inside board_back, same material.
        ('cubby_left_wall',(.003,.100,.262),(.0655,.051,.224)),
        ('cubby_right_wall',(.003,.100,.262),(.1525,.051,.224)),
        ('cubby_top',(.090,.100,.003),(.109,.051,.354)),
        ('cubby_base_trim',(.087,.003,.048),(.109,.002,.117)),
    ]: component(housing,name,[panel(name,dim,pos,ivory,housing)])
    for i in range(3):
        front=.141+i*.080
        vertices=[(x,y,front-(y-.002)*math.tan(math.radians(25))+dz)
                  for x in (.0655,.1525) for y in (.002,.097) for dz in (-.0015,.0015)]
        floor=mesh(f'holder_sloped_floor_{i}',vertices,BOX_FACES,housing,ivory)
        collider(floor)
        lip=panel(f'holder_retaining_lip_{i}',(.087,.003,.008),(.109,.002,front+.004),ivory,housing)
        component(housing,f'cubby_shelf{i+1}',[floor,lip])
    for side,x in [('left',-.079),('right',.079)]:
        drawer=body(root.name+'.drawer_'+side,root)
        drawer.location=(x,-.003,.033)
        panels=[]
        for name,dim,pos in [
            ('floor',(.145,.197,.003),(x,-.003,.012)),
            ('wall_left',(.003,.197,.045),(x-.071,-.003,.036)),
            ('wall_right',(.003,.197,.045),(x+.071,-.003,.036)),
            ('rear',(.144,.003,.045),(x,.094,.036)),
            ('front_upper',(.144,.004,.023),(x,-.104,.047)),
            ('front_left',(.044,.004,.023),(x-.050,-.104,.025)),
            ('front_right',(.044,.004,.023),(x+.050,-.104,.025)),
        ]: panels.append(panel(side+'_'+name,dim,pos,smoke,drawer))
        component(drawer,'bin',panels)
        points=[(x-.031,-.106,.018),(x-.031,-.116,.026),(x,-.120,.030),
                (x+.031,-.116,.026),(x+.031,-.106,.018)]
        handles=[rod(side+f'_handle_{i}',a,b,.0017,clear,drawer)
                 for i,(a,b) in enumerate(zip(points,points[1:]))]
        component(drawer,'handle',handles)
        joint(drawer.name+'.slide','SLIDER',housing,drawer,drawer.location,(0,.170),rotation=(0,0,-math.pi/2))
    tray=body(root.name+'.tray',root)
    tray.location=(-.060,.055,.144)
    panels=[]
    for name,dim,pos in [
        ('floor',(.196,.074,.003),(-.060,.055,.144)),
        ('front',(.196,.003,.045),(-.060,.020,.165)),
        ('back',(.196,.003,.045),(-.060,.090,.165)),
        ('left',(.003,.074,.045),(-.157,.055,.165)),
        ('right',(.003,.074,.045),(.037,.055,.165)),
    ]: panels.append(panel('tray_'+name,dim,pos,clear,tray))
    component(tray,'bin',panels)
    component(tray,'mounting',mounting_pegs(tray,[-.125,-.015],.175,clear))
    hook=body(root.name+'.hook',root)
    hook.location=(-.070,.094,.3365)
    component(hook,'mounting',mounting_pegs(hook,[-.070],.340,ivory))
    component(hook,'crook',[
        rod('hook_stem',(-.070,.094,.3365),(-.070,.060,.3365),.0034,ivory,hook),
        rod('hook_tip',(-.070,.060,.3365),(-.070,.060,.350),.0034,ivory,hook),
    ])


def main(argv=None):
    generate(build,__file__,argv)


if __name__=='__main__':
    main()
