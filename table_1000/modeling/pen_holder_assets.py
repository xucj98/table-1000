"""Container helpers; object sources supply all approved dimensions and layouts."""
import math
import bpy
from table_1000.modeling.asset_builders import (
    asset, body, box, collider, generate, group_part, material, mesh,
)


def semantic(root,name,previous):
    objects=[o for o in root.children if o not in previous]
    return group_part(root.name+'.'+name,root,
                      [o for o in objects if o.get('geometry_role')=='visual'],
                      [o for o in objects if o.get('geometry_role')=='collision'])


PROFILES = {
    "transparent": {"width": .120, "depth": .080, "height": .100,
                    "wall": .002, "floor": .003, "radius": .006, "columns": 3},
    "black": {"width": .100, "depth": .100, "height": .130,
              "wall": .0025, "floor": .003, "radius": .008, "columns": 1},
}


def outline(width, depth, radius, front_notch=False):
    """Rounded rectangular contour, counterclockwise; corner centres stay fixed."""
    points = []
    for cx, cy, start in ((width/2-radius, depth/2-radius, 0),
                           (-width/2+radius, depth/2-radius, 90),
                           (-width/2+radius, -depth/2+radius, 180),
                           (width/2-radius, -depth/2+radius, 270)):
        for step in range(4):
            angle = math.radians(start+step*30)
            points.append((cx+radius*math.cos(angle), cy+radius*math.sin(angle)))
        if front_notch and start == 180:
            points.extend((x, -depth/2) for x in (-.030, -.016, .016, .030))
    return points


def floor_mesh(contour, depth, surface, parent):
    n = len(contour)
    vertices = [(x,y,z) for z in (0,depth) for x,y in contour]
    faces = [tuple(range(n-1,-1,-1)), tuple(range(n,2*n))]
    faces.extend((i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n))
    obj = mesh("Supporting floor", vertices, faces, parent, surface)
    collider(obj)


def build_holder(style):
    p = PROFILES[style]
    w,d,h,t,f,r = (p[k] for k in ("width","depth","height","wall","floor","radius"))
    clear = style == "transparent"
    surface = material("Clear moulded plastic" if clear else "Matte black plastic",
                       (.90,.94,.94) if clear else (.012,.015,.017),
                       .12 if clear else .34, transmission=.96 if clear else 0)
    surface.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.49
    frame = asset("pen_holder", "bottom centre", "+X width; +Y back; +Z up", "front wall at -Y")
    holder = body("pen_holder.container", frame)
    holder["compartments"] = p["columns"]
    holder["nominal_dimensions_m"] = [w,d,h]
    outer = outline(w,d,r,clear)
    inner = outline(w-2*t,d-2*t,r-t,clear)
    previous=set(holder.children)
    floor_mesh(outer,f,surface,holder)
    semantic(holder,"floor",previous)
    previous=set(holder.children)
    n = len(outer)
    for i in range(n):
        j = (i+1)%n
        heights = []
        for k in (i,j):
            x,y = outer[k]
            if clear and y < -d/2+.00001:
                ratio = min(1,max(0,(abs(x)-.016)/.014))
                heights.append(.080+.020*ratio)
            else:
                heights.append(h)
        vertices = [(x,y,z) for ztop in (False,True)
                    for k,top in ((i,heights[0]),(j,heights[1]))
                    for x,y in (outer[k],inner[k])
                    for z in (top if ztop else f,)]
        # Four lower and corresponding four upper points, around the strip.
        faces = [(0,1,3,2),(4,6,7,5),(0,2,6,4),(1,5,7,3),(0,4,5,1),(2,3,7,6)]
        obj = mesh(f"Outer wall {i:02d}",vertices,faces,holder,surface)
        collider(obj)
    semantic(holder,"walls",previous)
    if clear:
        usable = w-2*t
        for index in (1,2):
            previous=set(holder.children)
            x = -usable/2 + usable*index/3
            box(f"Full-height divider {index}", (t,d-2*t,h-f), (x,0,(h+f)/2),
                surface,holder)
            semantic(holder,"divider"+str(index),previous)
    for obj in holder.children_recursive:obj.hide_select=True
    bpy.ops.object.select_all(action="DESELECT")
    holder.select_set(True)
    bpy.context.view_layer.objects.active=holder
