# -----------------------------------------------------------------------------
# start
# -----------------------------------------------------------------------------

import bpy
import numpy as np

def recursive_find_layer_collection(layer_collection, collection_name):
    found = None
    if (layer_collection.name == collection_name):
        return layer_collection
    for layer in layer_collection.children:
        found = recursive_find_layer_collection(layer, collection_name)
        if found:
            return found

# -----------------------------------------------------------------------------
# dimensions
# -----------------------------------------------------------------------------

car_push_back = -5

wheel_radius = 4
wheel_width = 1.5
wheel_body_offset = 3.5

body_width = 10
body_length = 25
body_height = 3

axel_width = 8

pebble_cup_height = 5
pebble_cup_width = 5
pebble_cup_length = 5
pebble_radius = 1.5/2

sphere_extrusion_depth = 0.20
sphere_extrusion_radius = 14

ball_cup_height = 5
ball_cup_width = 5
ball_cup_length = 5

sonars_panel_width = 7
sonars_panel_height = 3
sonars_panel_length = 0.5

sonar_width = 0.1
sonar_radius = 0.8

follower_plate_connector_height = 0.5
follower_plate_connector_width = 2
follower_plate_connector_length = 2

follower_plate_height = 0.5
follower_plate_length = 3
follower_plate_width = 20
follower_captors_height = 0.2

invisible_cylinder_radius = 1.8
invisible_cylinder_height = 2

invisible_cone_radius = 29.4448637287/2
invisible_cone_length = 51

#------------------------
#Gravity changes
#------------------------
bpy.context.scene.gravity[2] = -98.1

# -----------------------------------------------------------------------------
# car_collection
# -----------------------------------------------------------------------------

car_collection = bpy.data.collections.new('car_collection')
bpy.context.scene.collection.children.link(car_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'car_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back+12.5, wheel_radius),
    scale=(body_width, body_length, body_height)
)
bpy.context.active_object.name = 'body'

bpy.ops.mesh.primitive_cylinder_add(
    radius=1,
    enter_editmode=False,
    align='WORLD', 
    location=(0, car_push_back+wheel_body_offset, wheel_radius),
    scale=(0.5, 0.5, axel_width),
    rotation=(0, 90*np.pi/180, 0)
)
bpy.context.active_object.name = 'front_axeel'

bpy.ops.mesh.primitive_cylinder_add(
    radius=1,
    enter_editmode=False,
    align='WORLD', 
    location=(0, car_push_back+body_length-wheel_body_offset, wheel_radius),
    scale=(0.5, 0.5, axel_width),
    rotation=(90*np.pi/180, 0, 90*np.pi/180)
)
bpy.context.active_object.name = 'back_axeel'

for i in range(0, 2):
    bpy.ops.mesh.primitive_cylinder_add(
        radius=wheel_radius,
        enter_editmode=False,
        align='WORLD', 
        location=((-1)**i*(body_width/2+wheel_width+2), car_push_back+body_length-wheel_body_offset, wheel_radius),
        scale=(1, 1, wheel_width),
        rotation=(0, 90*np.pi/180, 0)
    )
    bpy.context.active_object.name = 'back_wheel_'+str(i)

for i in range(0, 2):
    bpy.ops.mesh.primitive_cylinder_add(
        radius=wheel_radius,
        enter_editmode=False,
        align='WORLD', 
        location=((-1)**i*(body_width/2+wheel_width+2), car_push_back+wheel_body_offset, wheel_radius),
        scale=(1, 1, wheel_width),
        rotation=(0, 90*np.pi/180, 0)
    )
    bpy.context.active_object.name = 'front_wheel_'+str(i)

# -----------------------------------------------------------------------------
# car_collection/pebble_collection
# -----------------------------------------------------------------------------

pebble_collection = bpy.data.collections.new('pebble_collection')
car_collection.children.link(pebble_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'pebble_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_uv_sphere_add(
    radius=pebble_radius,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back+pebble_cup_length/2, wheel_radius+body_height/2+pebble_cup_height+pebble_radius+sphere_extrusion_depth/2),
    scale=(1, 1, 1)
)
bpy.context.active_object.name = 'pebble'

#Appliquer RigidBody
bpy.ops.rigidbody.object_add()
bpy.context.object.rigid_body.friction = 0.5
bpy.context.object.rigid_body.mesh_source = 'FINAL'

# -----------------------------------------------------------------------------
# car_collection/pebble_cup_collection
# -----------------------------------------------------------------------------

ball_cup_collection = bpy.data.collections.new('ball_cup_collection')
car_collection.children.link(ball_cup_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'ball_cup_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_uv_sphere_add(
    radius=sphere_extrusion_radius,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back+pebble_cup_length/2, wheel_radius+body_height/2+pebble_cup_height+sphere_extrusion_radius-sphere_extrusion_depth),
    scale=(1, 1, 1)
)
bpy.context.active_object.name = 'sphere_extrusion'

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back+pebble_cup_length/2, wheel_radius+body_height/2+pebble_cup_height/2),
    scale=(pebble_cup_width, pebble_cup_length, pebble_cup_height)
)
bpy.context.active_object.name = 'pebble_cup'

# Make sphere_extrusion rigid
bpy.ops.rigidbody.object_add()
bpy.context.object.rigid_body.type = 'PASSIVE'
bpy.context.object.rigid_body.kinematic = True
bpy.context.object.rigid_body.collision_shape = 'MESH'
bpy.context.object.rigid_body.mesh_source = 'FINAL'
bpy.context.object.rigid_body.friction = 0.5

# Extrude the pebble_cub
bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["sphere_extrusion"]
bpy.ops.object.modifier_apply(modifier="Boolean")

# Delete the sphere_extrusion
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects['sphere_extrusion'].select_set(True)
bpy.ops.object.delete()

# -----------------------------------------------------------------------------
# car_collection/sonars_collection
# -----------------------------------------------------------------------------

sonars_collection = bpy.data.collections.new('sonars_collection')
car_collection.children.link(sonars_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'sonars_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back+sonars_panel_length/2, wheel_radius+body_height/2+ball_cup_height+sonars_panel_height/2),
    scale=(sonars_panel_width, sonars_panel_length, sonars_panel_height)
)
bpy.context.active_object.name = 'sonars_panel'

bpy.ops.mesh.primitive_cylinder_add(
    radius=sonar_radius,
    enter_editmode=False,
    align='WORLD',
    location=(-sonars_panel_width/2+sonar_radius+1, car_push_back, wheel_radius+body_height/2+ball_cup_height+sonars_panel_height/2),
    scale=(1, 1, sonar_width),
    rotation=(90*np.pi/180, 180*np.pi/180, 180*np.pi/180)
)
bpy.context.active_object.name = 'left_sonar'

bpy.ops.mesh.primitive_cylinder_add(
    radius=sonar_radius,
    enter_editmode=False,
    align='WORLD',
    location=(sonars_panel_width/2-sonar_radius-1, car_push_back, wheel_radius+body_height/2+ball_cup_height+sonars_panel_height/2),
    scale=(1, 1, sonar_width),
    rotation=(90*np.pi/180, 180*np.pi/180, 180*np.pi/180)
)
bpy.context.active_object.name = 'right_sonar'

# -----------------------------------------------------------------------------
# car_collection/sonars_collection/cone_invisible
# -----------------------------------------------------------------------------
invisible_cone_collection = bpy.data.collections.new('invisible_cone_collection')
sonars_collection.children.link(invisible_cone_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'invisible_cone_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection
bpy.ops.mesh.primitive_cone_add(
radius1=invisible_cone_radius, 
radius2=0,
depth=invisible_cone_length, 
enter_editmode=False, 
align='WORLD', 
location=(sonars_panel_width/2-sonar_radius-1, car_push_back-invisible_cone_length/2, wheel_radius+body_height/2+ball_cup_height+sonars_panel_height/2),
rotation=(-np.pi/2, 0, 0), 
scale=(1, 1, 1))
bpy.context.active_object.name = 'invisible_cone'

bpy.ops.object.modifier_add(type='MASK')
# -----------------------------------------------------------------------------
# car_collection/follower_collection
# -----------------------------------------------------------------------------

follower_collection = bpy.data.collections.new('follower_collection')
car_collection.children.link(follower_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'follower_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back-follower_plate_connector_length/2, wheel_radius-body_height/2+follower_plate_connector_height/2),
    scale=(follower_plate_connector_width, follower_plate_connector_length, follower_plate_connector_height),
    rotation=(0, 0, 0)
)
bpy.context.active_object.name = 'follower_plate_connector'

bpy.ops.mesh.primitive_cube_add(
    size=1,
    enter_editmode=False,
    align='WORLD',
    location=(0, car_push_back-follower_plate_length/2-follower_plate_connector_length, wheel_radius-body_height/2+follower_plate_height/2),
    scale=(follower_plate_width, follower_plate_length, follower_plate_height),
    rotation=(0, 0, 0)
)
bpy.context.active_object.name = 'follower_plate'

# -----------------------------------------------------------------------------
# car_collection/follower_collection/invisible_cylinders
# -----------------------------------------------------------------------------

invisible_cylinders_collection = bpy.data.collections.new('invisible_cylinders_collection')
follower_collection.children.link(invisible_cylinders_collection)
layer_collection = bpy.context.view_layer.layer_collection
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'invisible_cylinders_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

for i in range(0, 5):
    bpy.ops.mesh.primitive_cylinder_add(
    radius=invisible_cylinder_radius,
    enter_editmode=False,
    align='WORLD', 
    location=(0+(i-2)*follower_plate_width/5, car_push_back-follower_plate_length/2-follower_plate_connector_length, wheel_radius-body_height/2-follower_captors_height/2/2-invisible_cylinder_height),
    scale=(1, 1, invisible_cylinder_height),
    rotation=(0, 0, 0)
    )
    bpy.context.active_object.name = 'invisble_cylinder_'+str(i)

    bpy.ops.object.modifier_add(type='MASK')

# -----------------------------------------------------------------------------
# move origin of car_collection objects
# -----------------------------------------------------------------------------

bpy.ops.object.select_all(action='DESELECT')
sub_layer_collection = recursive_find_layer_collection(layer_collection, 'car_collection')
bpy.context.view_layer.active_layer_collection = sub_layer_collection

for i in range(0, 5):
    bpy.data.objects['invisble_cylinder_'+str(i)].select_set(True)
bpy.data.objects['follower_plate'].select_set(True)
bpy.data.objects['follower_plate_connector'].select_set(True)
bpy.data.objects['right_sonar'].select_set(True)
bpy.data.objects['left_sonar'].select_set(True)
bpy.data.objects['sonars_panel'].select_set(True)
bpy.data.objects['invisible_cone'].select_set(True)
for i in range(0, 2):
    bpy.data.objects['front_wheel_'+str(i)].select_set(True)
for i in range(0, 2):
    bpy.data.objects['back_wheel_'+str(i)].select_set(True)
bpy.data.objects['body'].select_set(True)
bpy.data.objects['back_axeel'].select_set(True)
bpy.data.objects['front_axeel'].select_set(True)

bpy.ops.object.mode_set(mode = 'OBJECT')
bpy.context.scene.cursor.location = [0, car_push_back+wheel_body_offset, wheel_radius]
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')  

# -----------------------------------------------------------------------------
# end
# -----------------------------------------------------------------------------

