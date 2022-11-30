# -------------------------------------------------------------------
# imports
# -------------------------------------------------------------------

import bpy

# -------------------------------------------------------------------
# functions
# -------------------------------------------------------------------

def recursive_find_layer_collection(layer_collection, collection_name):
    found = None
    if (layer_collection.name == collection_name):
        return layer_collection
    for layer in layer_collection.children:
        found = recursive_find_layer_collection(layer, collection_name)
        if found:
            return found
        
# -------------------------------------------------------------------
# parameters
# -------------------------------------------------------------------

path_height = 0.2
path_width = 1.8

section1_length = 80
section2_radius = 12
section3_length = 30
section4_radius = 30
section5_length = 165
section5_obstacle_height = 5
section5_obstacle_radius = 7
section6_length = 165
section6_obstacle_height = 5
section6_obstacle_radius = 7
end_of_path_length = 20

# -------------------------------------------------------------------
# path_collection
# -------------------------------------------------------------------

path_collection = bpy.data.collections.new('path_collection')
bpy.context.scene.collection.children.link(path_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'path_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

# -------------------------------------------------------------------
# path_collection/section1_collection
# -------------------------------------------------------------------

section1_collection = bpy.data.collections.new('section1_collection')
path_collection.children.link(section1_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section1_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, -section1_length/2, path_height/2),
    scale=(path_width, section1_length, path_height)
)
bpy.context.active_object.name = 'section1_line_800mm'

# -------------------------------------------------------------------
# path_collection/section2_collection
# -------------------------------------------------------------------

section2_collection = bpy.data.collections.new('section2_collection')
path_collection.children.link(section2_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section2_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cylinder_add(
    radius=section2_radius-path_width,
    depth=2,
    enter_editmode=False,
    align='WORLD',
    location=(section2_radius-path_width/2, -section1_length, path_height/2),
    scale=(1, 1, path_height)
)
bpy.context.active_object.name = 'section2_negative_90turn_120mm'

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(section2_radius+path_width/2, -section1_length+section2_radius/1.5, path_height/2),
    scale=(section2_radius*2, section2_radius, path_height*2)
)
bpy.context.active_object.name = 'section2_negative_cube1_90turn_120mm'

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(section2_radius+section2_radius/2, -section1_length+path_width/1.5, path_height/2),
    scale=(section2_radius, section2_radius*2, path_height*2)
)
bpy.context.active_object.name = 'section2_negative_cube2_90turn_120mm'

bpy.ops.mesh.primitive_cylinder_add(
    radius=section2_radius,
    depth=2,
    enter_editmode=False,
    align='WORLD',
    location=(section2_radius-path_width/2, -section1_length, path_height/2),
    scale=(1, 1, path_height)
)
bpy.context.active_object.name = 'section2_90turn_120mm'

# apply boolean modifier
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["section2_negative_90turn_120mm"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# apply boolean modifier
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["section2_negative_cube1_90turn_120mm"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# apply boolean modifier
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["section2_negative_cube2_90turn_120mm"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# delete negative turn
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['section2_negative_90turn_120mm'].select_set(True)
bpy.data.objects['section2_negative_cube1_90turn_120mm'].select_set(True)
bpy.data.objects['section2_negative_cube2_90turn_120mm'].select_set(True)
bpy.ops.object.delete()

# -------------------------------------------------------------------
# path_collection/section3_collection
# -------------------------------------------------------------------

section3_collection = bpy.data.collections.new('section3_collection')
path_collection.children.link(section3_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section3_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length/2-path_width/2,
        -section1_length-section2_radius+path_width/2,
        path_height/2
    ),
    scale=(section3_length, path_width, path_height)
)
bpy.context.active_object.name = 'section3_line_300mm'

# -------------------------------------------------------------------
# path_collection/section4_collection
# -------------------------------------------------------------------

section4_collection = bpy.data.collections.new('section4_collection')
path_collection.children.link(section4_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section4_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cylinder_add(
    radius=section4_radius-path_width,
    depth=2,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length-path_width/2,
        -section1_length-section2_radius+path_width-section4_radius,
        path_height/2
    ),
    scale=(1, 1, path_height)
)
bpy.context.active_object.name = 'section4_negative_180turn_300mm'

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length-path_width/2-section4_radius/2,
        -section1_length-section2_radius+path_width-section4_radius,
        path_height/2
    ),
    scale=(section4_radius, section4_radius*2-path_width, path_height*2)
)
bpy.context.active_object.name = 'section4_negative_cube1_180turn_300mm'

bpy.ops.mesh.primitive_cylinder_add(
    radius=section4_radius,
    depth=2,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length-path_width/2,
        -section1_length-section2_radius+path_width-section4_radius,
        path_height/2
    ),
    scale=(1, 1, path_height)
)
bpy.context.active_object.name = 'section4_180turn_300mm'

# apply boolean modifier
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["section4_negative_180turn_300mm"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# apply boolean modifier
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["section4_negative_cube1_180turn_300mm"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# delete negative turn
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['section4_negative_180turn_300mm'].select_set(True)
bpy.data.objects['section4_negative_cube1_180turn_300mm'].select_set(True)
bpy.ops.object.delete()

# -------------------------------------------------------------------
# path_collection/section5_collection
# -------------------------------------------------------------------

section5_collection = bpy.data.collections.new('section5_collection')
path_collection.children.link(section5_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section5_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length-path_width/2-section5_length/2,
        path_width*1.5-section1_length-section2_radius-2*section4_radius,
        path_height/2
    ),
    scale=(section5_length, path_width, path_height)
)
bpy.context.active_object.name = 'section5_line_165mm'

# -------------------------------------------------------------------
# path_collection/section6_collection
# -------------------------------------------------------------------

section6_collection = bpy.data.collections.new('section6_collection')
path_collection.children.link(section6_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'section6_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(
        section2_radius+section3_length-path_width/2-section5_length-section6_length/2,
        path_width*1.5-section1_length-section2_radius-2*section4_radius,
        path_height/2
    ),
    scale=(section5_length, path_width, path_height)
)
bpy.context.active_object.name = 'section6_line_165mm'

# -------------------------------------------------------------------
# path_collection/obstacles_collection
# -------------------------------------------------------------------
obstacles_collection = bpy.data.collections.new('obstacles_collection')
path_collection.children.link(obstacles_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'obstacles_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cylinder_add(
        radius=section5_obstacle_radius,
        enter_editmode=False,
        align='WORLD', 
        location=(
            section2_radius+section3_length-path_width/2-section5_length/2 - 50,
            path_width*1.5-section1_length-section2_radius-2*section4_radius,
            section5_obstacle_height
        ),
        scale=(1, 1, section5_obstacle_height),
        rotation=(0, 0, 0)
    )
bpy.context.active_object.name = 'section5_obstacle'

# -------------------------------------------------------------------
# path_collection/end_of_path_collection
# -------------------------------------------------------------------
end_of_path_collection = bpy.data.collections.new('end_of_path_collection')
path_collection.children.link(end_of_path_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'end_of_path_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
        size=1,
        enter_editmode=False,
        align='WORLD', 
        location=(
            -(section5_length/2)+section2_radius+section3_length-path_width/2-section5_length-section6_length/2,
        path_width*1.5-section1_length-section2_radius-2*section4_radius,
        path_height/2
        ),
        scale=(path_width, end_of_path_length, path_height),
        rotation=(0, 0, 0)
    )
bpy.context.active_object.name = 'end_of_path'

# -------------------------------------------------------------------
# end
# -------------------------------------------------------------------