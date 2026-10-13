"""Source-snapshot entry for hard storage containers; dimensions supplied by assets."""
import bpy
from table_1000.modeling.asset_builders import (
    body, cylinder, generate, group_part, material, tube,
)


def semantic(root,name,previous):
    objects=[o for o in root.children if o not in previous]
    return group_part(root.name+'.'+name,root,
                      [o for o in objects if o.get('geometry_role')=='visual'],
                      [o for o in objects if o.get('geometry_role')=='collision'])


PROFILES = {
    "large": {"diameter": .150, "height": .090, "lid": False},
    "medium": {"diameter": .090, "height": .050, "lid": True},
    "mini": {"diameter": .055, "height": .035, "lid": False},
}
SEGMENTS = 24
FLOOR = .002
WALL = .0015
LID_RADIAL_CLEARANCE = .00045
LID_VERTICAL_CLEARANCE = .00025
LID_PLATE = .002
LID_OVERLAP = .005


def build_box(size):
    p = PROFILES[size]
    radius,h = p["diameter"]/2,p["height"]
    clear = material("Clear storage plastic",(1,1,1),.035,transmission=1)
    clear.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.49
    paper = material("Unbranded white paper band",(.88,.90,.89),.6)
    blue = material("Blue band trim",(.015,.060,.19),.5)
    container = body("storage_box.container",None)
    container["coordinate_frame_normalized"] = True
    container["origin_convention"] = "bottom centre of container; lid origin at underside of lid plate"
    container["axis_convention"] = "+Z up; X/Y radial"
    container["nominal_dimensions_m"] = [p["diameter"],p["diameter"],h]
    container["inner_diameter_m"] = 2*(radius-.0001-WALL)
    container["clear_height_m"] = h-FLOOR
    previous=set(container.children)
    cylinder("Closed bottom",radius-.0001,FLOOR,(0,0,FLOOR/2),clear,container,
             axis="Z",segments=SEGMENTS)
    semantic(container,"floor",previous)
    previous=set(container.children)
    tube("Hollow side wall",radius-.0001,radius-.0001-WALL,h-FLOOR,
         (0,0,(h+FLOOR)/2),clear,container,axis="Z",segments=SEGMENTS)
    semantic(container,"walls",previous)
    previous=set(container.children)
    tube("Moulded mouth rim",radius,radius-.0018,.002,
         (0,0,h-.001),clear,container,axis="Z",segments=SEGMENTS,collision=False)
    semantic(container,"mouth_rim",previous)
    previous=set(container.children)
    label_height = h*.38
    tube("Plain white wrap",radius-.00002,radius-.00008,label_height,
         (0,0,h*.28),paper,container,axis="Z",segments=SEGMENTS,collision=False)
    semantic(container,"paper_label",previous)
    previous=set(container.children)
    for z in (h*.28-label_height/2+.00125,h*.28+label_height/2-.00125):
        tube("Blue paper border",radius-.00001,radius-.00003,.0025,
             (0,0,z),blue,container,axis="Z",segments=SEGMENTS,collision=False)
    semantic(container,"label_border",previous)
    if p["lid"]:
        lid = body("storage_box.lid",None)
        lid.location.z = h+LID_VERTICAL_CLEARANCE
        lid["fit"] = "Unlatched placement sleeve, no snap fit or holding-force claim"
        lid["radial_clearance_m"] = LID_RADIAL_CLEARANCE
        lid["vertical_clearance_m"] = LID_VERTICAL_CLEARANCE
        inner = radius+LID_RADIAL_CLEARANCE
        outer = inner+.00175
        previous=set(lid.children)
        cylinder("Lid plate",outer,LID_PLATE,(0,0,LID_PLATE/2),clear,lid,
                 axis="Z",segments=SEGMENTS)
        semantic(lid,"plate",previous)
        previous=set(lid.children)
        depth = LID_OVERLAP+LID_VERTICAL_CLEARANCE
        tube("Open lid skirt",outer,inner,depth,(0,0,-depth/2),clear,lid,
             axis="Z",segments=SEGMENTS)
        semantic(lid,"skirt",previous)
        for obj in lid.children_recursive:obj.hide_select=True
    for obj in container.children_recursive:obj.hide_select=True
    bpy.ops.object.select_all(action="DESELECT")
    container.select_set(True)
    bpy.context.view_layer.objects.active = container
