"""Five-component pegboard cabinet, in meters; no scene or texture dependencies."""
from pathlib import Path
import math
import sys

import bpy
from mathutils import Vector

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, box, collider, cylinder, generate, joint, material, mesh, BOX_FACES


def panel(name, dimensions, position, surface, owner, bevel=0):
    return box(name, dimensions, Vector(position) - owner.location, surface, owner, bevel=bevel)


def prism(name, contour, surface, owner):
    """Extrude a convex XZ contour through the 8 mm pegboard thickness."""
    n = len(contour)
    vertices = [(x, y, z) for y in (.097, .105) for x, z in contour]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2*n))]
    faces += [(i, (i+1) % n, (i+1) % n+n, i+n) for i in range(n)]
    collider(mesh(name, vertices, faces, owner, surface))


def pegboard(surface, owner):
    # Octagonal 10 mm clear bores. Convex sectors around each cell leave the
    # holes open in both visual and collision geometry, without Boolean slivers.
    xs = [-.125, -.070, -.015, .040]
    zs = [.120, .175, .230, .285, .340]
    xb = [-.160] + [(a+b)/2 for a,b in zip(xs,xs[1:])] + [.0625]
    zb = [.070] + [(a+b)/2 for a,b in zip(zs,zs[1:])] + [.380]
    for i,x in enumerate(xs):
        for j,z in enumerate(zs):
            # Equal square collar simplifies convex partition and preserves
            # a flat bottom at z-5mm for the removable mounting pegs.
            h = .007
            angles = [math.pi/8 + k*math.pi/4 for k in range(8)]
            # Include cell corners so every sector remains convex.
            angles = sorted(angles + [math.pi/4+k*math.pi/2 for k in range(4)])
            def inner(a):
                # Ray intersection with the regular octagonal aperture.
                normals = [k*math.pi/4 for k in range(8)]
                radius = .005 / max(math.cos(a-n) for n in normals)
                return (x+radius*math.cos(a), z+radius*math.sin(a))
            def outer(a):
                radius=h/max(abs(math.cos(a)),abs(math.sin(a)))
                return (x+radius*math.cos(a),z+radius*math.sin(a))
            for k,a in enumerate(angles):
                b=angles[(k+1)%len(angles)]
                prism(f"Bore_{i}_{j}_{k}", [inner(a),outer(a),outer(b),inner(b)], surface, owner)
            for name,lo,hi in [('left',xb[i],x-h),('right',x+h,xb[i+1])]:
                panel(f"Board_{i}_{j}_{name}",(hi-lo,.008,zb[j+1]-zb[j]),((lo+hi)/2,.101,(zb[j]+zb[j+1])/2),surface,owner)
            for name,lo,hi in [('bottom',zb[j],z-h),('top',z+h,zb[j+1])]:
                panel(f"Board_{i}_{j}_{name}",(.014,.008,hi-lo),(x,.101,(lo+hi)/2),surface,owner)
    panel("Board behind sloped cubbies",(.0975,.008,.310),(.11125,.101,.225),surface,owner)


def rod(name, a, b, radius, surface, owner):
    a,b=Vector(a),Vector(b)
    obj=cylinder(name,radius,(b-a).length,(a+b)/2-owner.location,surface,owner,axis="Z",segments=8)
    rotation=(b-a).to_track_quat('Z','Y').to_euler()
    obj.rotation_euler=rotation
    # cylinder() creates the matching collision object immediately.
    bpy.data.objects['Collision_'+obj.name].rotation_euler=rotation


def mounting_pegs(owner, xs, hole_z, surface):
    # Inferred straight peg mounting: seated on the aperture bottom, removable
    # along -Y. Pads contact the board front; no permanent weld/constraint.
    for i,x in enumerate(xs):
        panel(f"{owner.name} mounting pad {i}",(.009,.0055,.010),(x,.09425,hole_z-.0035),surface,owner)
        panel(f"{owner.name} mounting peg {i}",(.003,.020,.003),(x,.101,hole_z-.0035),surface,owner)


def join_visual(owner):
    objects=[o for o in owner.children if o.get('geometry_role')=='visual']
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:
        bpy.context.view_layer.objects.active=o
        for modifier in list(o.modifiers):
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        o.select_set(True)
    bpy.context.view_layer.objects.active=objects[0]
    bpy.ops.object.join()
    objects[0].name=owner.name+'_visual'
    for o in owner.children:
        if o.get('geometry_role')=='collision':
            o.hide_select=True


def build():
    ivory=material('Ivory ABS',(.82,.83,.80),.35)
    smoke=material('Smoky clear PET',(.36,.40,.39),.20,transmission=.72)
    clear=material('Clear tray PET',(.64,.68,.67),.16,transmission=.85)
    root=asset('cabinet/000002','housing bottom center','+X width; +Y back; +Z up','-Y')
    housing=body('housing',root)
    for name,dim,pos in [
        ('base_bottom',(.320,.220,.006),(0,0,.003)),
        ('base_top',(.320,.220,.010),(0,0,.068)),
        ('base_left',(.006,.220,.064),(-.157,0,.035)),
        ('base_right',(.006,.220,.064),(.157,0,.035)),
        ('base_back',(.320,.006,.064),(0,.107,.035)),
        ('base_center',(.006,.220,.064),(0,0,.035)),
    ]: panel(name,dim,pos,ivory,housing)
    pegboard(ivory,housing)
    # Reuse the current delivered three 25-degree cubbies, not the outdated
    # vertical pen partitions in the earlier scene Blender source.
    for name,dim,pos in [
        ('holder_back',(.090,.004,.262),(.109,.099,.224)),
        ('holder_left',(.003,.100,.262),(.0655,.051,.224)),
        ('holder_right',(.003,.100,.262),(.1525,.051,.224)),
        ('holder_top',(.090,.100,.003),(.109,.051,.354)),
        ('holder_base_trim',(.087,.003,.048),(.109,.002,.117)),
    ]: panel(name,dim,pos,ivory,housing)
    for i in range(3):
        front=.141+i*.080
        vertices=[(x,y,front-(y-.002)*math.tan(math.radians(25))+dz)
                  for x in (.0655,.1525) for y in (.002,.097) for dz in (-.0015,.0015)]
        collider(mesh(f'holder_sloped_floor_{i}',vertices,BOX_FACES,housing,ivory))
        panel(f'holder_retaining_lip_{i}',(.087,.003,.008),(.109,.002,front+.004),ivory,housing)
    for side,x in [('left',-.079),('right',.079)]:
        drawer=body('drawer_'+side,root)
        drawer.location=(x,-.003,.033)
        for name,dim,pos in [
            ('floor',(.145,.197,.003),(x,-.003,.012)),
            ('wall_left',(.003,.197,.045),(x-.071,-.003,.036)),
            ('wall_right',(.003,.197,.045),(x+.071,-.003,.036)),
            ('rear',(.144,.003,.045),(x,.094,.036)),
            ('front_upper',(.144,.004,.023),(x,-.104,.047)),
            ('front_left',(.044,.004,.023),(x-.050,-.104,.025)),
            ('front_right',(.044,.004,.023),(x+.050,-.104,.025)),
        ]: panel(side+'_'+name,dim,pos,smoke,drawer)
        points=[(x-.031,-.106,.018),(x-.031,-.116,.026),(x,-.120,.030),
                (x+.031,-.116,.026),(x+.031,-.106,.018)]
        for i,(a,b) in enumerate(zip(points,points[1:])): rod(side+f'_handle_{i}',a,b,.0017,clear,drawer)
        joint('drawer_'+side+'_slide','SLIDER',housing,drawer,drawer.location,(0,.170),rotation=(0,0,-math.pi/2))
    tray=body('tray',root)
    tray.location=(-.060,.055,.144)
    for name,dim,pos in [
        ('floor',(.196,.074,.003),(-.060,.055,.144)),
        ('front',(.196,.003,.045),(-.060,.020,.165)),
        ('back',(.196,.003,.045),(-.060,.090,.165)),
        ('left',(.003,.074,.045),(-.157,.055,.165)),
        ('right',(.003,.074,.045),(.037,.055,.165)),
    ]: panel('tray_'+name,dim,pos,clear,tray)
    mounting_pegs(tray,[-.125,-.015],.175,clear)
    hook=body('hook',root)
    hook.location=(-.070,.094,.3365)
    mounting_pegs(hook,[-.070],.340,ivory)
    rod('Hook stem',(-.070,.094,.3365),(-.070,.060,.3365),.0034,ivory,hook)
    rod('Hook raised tip',(-.070,.060,.3365),(-.070,.060,.350),.0034,ivory,hook)
    for owner in (housing,tray,hook,bpy.data.objects['drawer_left'],bpy.data.objects['drawer_right']):
        join_visual(owner)


def main(argv=None):
    generate(build,__file__,argv)


if __name__=='__main__':
    main()
