"""Three-body native press-unlock knife. Metres; engineering geometry."""
import math
import bpy
from table_1000.modeling.asset_builders import asset, body, part, box, generate
from table_1000.modeling.small_stationery_assets import rounded_prism, finish_body

Q_MAX = .020
P_MAX = .0015


def slider(name, parent_body, child, position, rotation, maximum):
    obj = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = position
    obj.rotation_euler = rotation
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.rigidbody.constraint_add()
    joint = obj.rigid_body_constraint
    joint.type = 'SLIDER'
    joint.object1, joint.object2 = parent_body, child
    joint.use_limit_lin_x = True
    joint.limit_lin_x_lower, joint.limit_lin_x_upper = 0, maximum
    joint.disable_collisions = False
    obj['preview_joint'] = True
    obj.select_set(False)
    return obj


def build():
    from table_1000.modeling.asset_builders import material
    root = asset('utility_knife', 'bottom footprint center',
                 '+X rear; -X blade extension; +Z up', 'blade exits at -X')
    housing = body('utility_knife.body', root)
    carriage = body('utility_knife.carriage', root)
    button = body('utility_knife.button', root)
    white = material('Knife white shell', (.82, .84, .83), .32)
    green = material('Knife pale green press slider', (.48, .59, .37), .44)
    dark = material('Knife recessed track', (.018, .022, .023), .52)
    steel = material('Knife blade steel', (.39, .42, .44), .32, metallic=.8)
    shell = part('utility_knife.body.housing', housing)
    rails = part('utility_knife.body.guide', housing)
    rack = part('utility_knife.body.rack', housing)
    # Hollow capsule: the front mid-height opening is real geometry.
    rounded_prism(shell.name+'.bottom', (.095, .020, .0014), (0,0,.0007), .010, white, shell, segments=4)
    for index,y in enumerate((-.00925,.00925),1):
        box(shell.name+f'.side{index}', (.075,.0015,.0066), (0,y,.0047), white,shell)
    # Curved end annuli, each segment is convex and leaves the exit clear.
    for end,cx in [('front',-.0375),('rear',.0375)]:
        angles = ([math.pi/2+i*math.pi/8 for i in range(9)] if end=='front'
                  else [-math.pi/2+i*math.pi/8 for i in range(9)])
        from table_1000.modeling.asset_builders import mesh,collider
        for i,(a,b) in enumerate(zip(angles,angles[1:]),1):
            # Omit the central front sectors so the 9 mm wide blade can exit.
            if end=='front' and i in (3,4,5,6):continue
            points=[(cx+r*math.cos(t),r*math.sin(t),z)
                    for z in (.0014,.008) for r,t in ((.0085,a),(.010,a),(.010,b),(.0085,b))]
            faces=[(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
            visual=mesh(shell.name+f'.{end}{i}',points,faces,shell,white);collider(visual)
    # Top face slot spans the complete knob travel; recessed into white roof.
    for index,y in enumerate((-.00685,.00685),1):
        box(shell.name+f'.roof{index}',(.075,.0063,.0013),(0,y,.00865),white,shell)
    for index,x in enumerate((-.029,.029),1):
        box(shell.name+f'.slot_end{index}',(.017,.0074,.0013),(x,0,.00865),white,shell)
    # Cosmetic rounded top ends sit outside the moving channel.
    for index,x in enumerate((-.0415,.0415),1):
        rounded_prism(shell.name+f'.roof_end{index}',(.012,.019,.0013),(x,0,.00865),.0059,white,shell,segments=3)
    # Low ledges support the blade but do not clamp its entire face.
    for index,y in enumerate((-.0065,.0065),1):
        box(rails.name+f'.ledge{index}',(.079,.001,.0016),(0,y,.0022),dark,rails)
    # Five slots at x=+10,+5,0,-5,-10 mm; 1.1 mm axial opening.
    centers=sorted(.010-i*.005 for i in range(5))
    cursor=-.015
    for i,x in enumerate(centers+[.01555],1):
        left=x-.00055
        if left>cursor:
            box(rack.name+f'.tooth{i}',(left-cursor,.0026,.0015),((left+cursor)/2,.0054,.00725),steel,rack)
        cursor=x+.00055
    blade = part('utility_knife.carriage.blade', carriage)
    support = part('utility_knife.carriage.support', carriage)
    # 55.1 x 9 x 0.3 mm steel strip, tip hidden 10 mm at q=0.
    box(blade.name+'.strip',(.0551,.009,.0003),(-.00995,0,.0032),steel,blade)
    box(support.name+'.frame',(.034,.006,.0006),(.003,0,.0039),dark,support)
    knob = part('utility_knife.button.knob',button)
    tongue = part('utility_knife.button.latch',button)
    rounded_prism(knob.name+'.pad',(.015,.007,.0007),(.010,0,.00965),.0027,green,knob,segments=2)
    box(knob.name+'.stem',(.0028,.0028,.0030),(.010,0,.0078),green,knob)
    box(tongue.name+'.arm',(.0007,.0074,.0004),(.010,.0023,.0062),steel,tongue)
    box(tongue.name+'.tip',(.0007,.0026,.0010),(.010,.0054,.0067),steel,tongue)
    slider('utility_knife.slide',housing,carriage,(0,0,0),(0,0,math.pi),Q_MAX)
    slider('utility_knife.press',carriage,button,(.010,0,.007),(0,math.pi/2,0),P_MAX)
    for rigid,mass in [(housing,.016),(carriage,.003),(button,.001)]:
        rigid.rigid_body.mass=mass
        finish_body(rigid)


def main(argv=None):
    generate(build,__file__,argv)


if __name__=='__main__':main()
