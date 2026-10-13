"""Warm-white basket with two end handles and a continuous rounded rim."""
import math
import bpy
import bmesh
from table_1000.modeling.asset_builders import (
    BOX_FACES, asset, body, collider, generate, group_part, material, mesh,
)

WALL_THICKNESS = .0022
WALL_SLOPE = .12
WALL_BOTTOM = .020
WALL_TOP = .093
ROWS = 7
HANDLE_BOTTOM = WALL_BOTTOM + 5 * (WALL_TOP - WALL_BOTTOM) / ROWS
CORNER_RADIUS = .012


def wall_point(axis, sign, base, u, z, offset):
    normal = sign * (base + WALL_SLOPE*z + offset)
    return (normal,u,z) if axis == 'X' else (u,normal,z)


def panel(name,axis,sign,base,u0,u1,z0,z1,parent,surface=None,shape=None):
    lo=(u0,u0)if isinstance(u0,(int,float))else u0
    hi=(u1,u1)if isinstance(u1,(int,float))else u1
    vertices=[wall_point(axis,sign,base,u[k],z,offset)
              for u in(lo,hi)for offset in(-WALL_THICKNESS/2,WALL_THICKNESS/2)
              for k,z in enumerate((z0,z1))]
    return mesh(name,vertices,BOX_FACES,parent,surface,shape)


def perforated_wall(axis,sign,base,half_span,columns,handle,surface,parent):
    """Eight-sided ellipse tunnels; neighbouring cells share surface boundaries."""
    vertices,faces=[],[]
    pitch=2*half_span/columns; height=(WALL_TOP-WALL_BOTTOM)/ROWS
    count=0
    for row in range(ROWS):
        zc=WALL_BOTTOM+(row+.5)*height
        for col in range(columns):
            uc=-half_span+(col+.5)*pitch
            if handle and row>=5 and abs(uc)<handle:continue
            count+=1;start=len(vertices)
            for offset in (-WALL_THICKNESS/2,WALL_THICKNESS/2):
                for inner in (False,True):
                    for i in range(8):
                        c,s=math.cos(math.tau*i/8),math.sin(math.tau*i/8)
                        if inner:du,dz=pitch*.29*c,height*.25*s
                        else:
                            factor=max(abs(c),abs(s));du,dz=pitch/2*c/factor,height/2*s/factor
                        vertices.append(wall_point(axis,sign,base,uc+du,zc+dz,offset))
            for i in range(8):
                j=(i+1)%8
                faces.extend(tuple(start+k for k in f)for f in (
                    (i,j,8+j,8+i),(16+j,16+i,24+i,24+j),(8+j,24+j,24+i,8+i)))
    visual=mesh('perforated_wall',vertices,faces,parent,surface)
    bm=bmesh.new();bm.from_mesh(visual.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-9)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(visual.data);bm.free()
    visual['elliptical_holes']=count
    # Continuous solid lower edge; no free-standing corner posts.
    panel('lower_edge',axis,sign,base,-half_span,half_span,0,WALL_BOTTOM,parent,surface)
    extent=lambda z:half_span+WALL_SLOPE*z
    if handle:
        panel('lower_proxy',axis,sign,base,(-extent(0),-extent(HANDLE_BOTTOM)),(extent(0),extent(HANDLE_BOTTOM)),0,HANDLE_BOTTOM,parent,shape='CONVEX_HULL')
        for side in(-1,1):
            a,b=((-extent(HANDLE_BOTTOM),-extent(WALL_TOP)),-handle)if side<0 else(handle,(extent(HANDLE_BOTTOM),extent(WALL_TOP)))
            panel('handle_flank_proxy',axis,sign,base,a,b,HANDLE_BOTTOM,WALL_TOP,parent,shape='CONVEX_HULL')
    else:
        panel('wall_proxy',axis,sign,base,(-extent(0),-extent(WALL_TOP)),(extent(0),extent(WALL_TOP)),0,WALL_TOP,parent,shape='CONVEX_HULL')
    # Flat perforated strips meet the sloping rounded corners through tapered edge strips.
    adjacent=.102 if axis=='X'else.147
    for side in(-1,1):
        verts=[]
        for z in(0,WALL_TOP):
            for u in(side*half_span,side*(adjacent+WALL_SLOPE*z-CORNER_RADIUS)):
                for off in(-WALL_THICKNESS/2,WALL_THICKNESS/2):
                    verts.append(wall_point(axis,sign,base,u,z,off))
        mesh('corner_transition',verts,[(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)],parent,surface)


def contour(hx,hy,r,segments):
    points=[]
    for sx,sy,start in((1,1,0),(-1,1,90),(-1,-1,180),(1,-1,270)):
        for i in range(segments+1):
            a=math.radians(start+90*i/segments)
            points.append((sx*(hx-r)+r*math.cos(a),sy*(hy-r)+r*math.sin(a)))
    return points


def ring(name,outer_lo,inner_lo,outer_hi,inner_hi,z0,z1,parent,surface,collision=False):
    n=len(outer_lo)
    vertices=[(x,y,z)for points,z in((outer_lo,z0),(inner_lo,z0),(outer_hi,z1),(inner_hi,z1))for x,y in points]
    faces=[]
    for i in range(n):
        j=(i+1)%n
        faces.extend(((i,j,2*n+j,2*n+i),(n+j,n+i,3*n+i,3*n+j),
                      (i,n+i,n+j,j),(2*n+i,2*n+j,3*n+j,3*n+i)))
    mesh(name,vertices,faces,parent,surface)
    if collision:
        for i in range(n):
            j=(i+1)%n
            vs=[vertices[k]for k in(i,j,n+j,n+i,2*n+i,2*n+j,3*n+j,3*n+i)]
            mesh('rim_proxy',vs,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],parent,shape='CONVEX_HULL')


def semantic(root,name,start):
    added=[o for o in root.children if o not in start]
    group_part(root.name+'.'+name,root,
               [o for o in added if o.get('geometry_role')=='visual'],
               [o for o in added if o.get('geometry_role')=='collision'])


def build():
    pp=material('Warm white polypropylene',(.83,.85,.82),.30)
    frame=asset('file_basket','centre of bottom supporting plane','+X long; +Y short; +Z up','long wall at -Y')
    root=body('file_basket.shell',frame)
    root['collision_approximation']='Small elliptical holes filled; inner cavity and only two end handles open'
    start=set(root.children)
    # Low-segment rounded floor, connected to the continuous side skirt.
    pts=contour(.1481,.1031,.012,4);n=len(pts)
    vs=[(x,y,z)for z in(0,.003)for x,y in pts]
    fs=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n)for i in range(n)]
    floor=mesh('floor',vs,fs,root,pp)
    collider(floor)
    semantic(root,'floor',start)
    for axis,base,span,cols,handle in(('X',.147,.090,12,.045),('Y',.102,.135,18,0)):
        for index,sign in enumerate((-1,1),1):
            start=set(root.children)
            perforated_wall(axis,sign,base,span,cols,handle,pp,root)
            semantic(root,('end_wall'if axis=='X'else'long_wall')+str(index),start)
    start=set(root.children)
    # Rounded sidewall corners, inset into the wall contour rather than posts.
    for sx,sy,start_angle in((1,1,0),(-1,1,90),(-1,-1,180),(1,-1,270)):
        vertices=[]
        for z in(0,WALL_TOP):
            for off in(-WALL_THICKNESS/2,WALL_THICKNESS/2):
                for i in range(5):
                    a=math.radians(start_angle+i*22.5)
                    vertices.append((sx*(.147+WALL_SLOPE*z-CORNER_RADIUS)+(CORNER_RADIUS+off)*math.cos(a),
                                     sy*(.102+WALL_SLOPE*z-CORNER_RADIUS)+(CORNER_RADIUS+off)*math.sin(a),z))
        faces=[]
        for i in range(4):
            j=i+1;faces.extend(((i,j,10+j,10+i),(5+j,5+i,15+i,15+j),(i,5+i,5+j,j),(10+i,10+j,15+j,15+i)))
        faces.extend(((0,10,15,5),(4,9,19,14)))
        mesh('rounded_corner',vertices,faces,root,pp)
        for i,j in((0,2),(2,4)):
            vs=[vertices[k]for k in(i,j,5+j,5+i,10+i,10+j,15+j,15+i)]
            mesh('corner_proxy',vs,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],root,shape='CONVEX_HULL')
    semantic(root,'rounded_corners',start)
    start=set(root.children)
    ring('continuous_rim',contour(.164,.121,.015,4),contour(.156,.113,.011,4),
         contour(.164,.121,.015,4),contour(.156,.113,.011,4),.093,.1005,root,pp)
    # Coarser closed convex rim segments preserve the opening and end grips.
    outer=contour(.164,.121,.015,2);inner=contour(.156,.113,.011,2)
    n=len(outer)
    for i in range(n):
        j=(i+1)%n
        vs=[(x,y,z)for z in(.093,.1005)for x,y in(outer[i],outer[j],inner[j],inner[i])]
        mesh('rim_proxy',vs,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],root,shape='CONVEX_HULL')
    semantic(root,'rim',start)
    for o in root.children_recursive:o.hide_select=True
    bpy.ops.object.select_all(action='DESELECT');root.select_set(True)
    bpy.context.view_layer.objects.active=root


def main(argv=None):
    generate(build,__file__,argv)


if __name__=='__main__':main()
