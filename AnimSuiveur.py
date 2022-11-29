# -----------------------------------------------------------------------------
# imports
# -----------------------------------------------------------------------------

import bpy
import math
from math import tan
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# -----------------------------------------------------------------------------
# global variables
# -----------------------------------------------------------------------------

FPS = 24
FORWARD_ACCELERATION = 0.3/FPS
BACKWARD_ACCELERATION = -0.3/FPS
FORWARD_TURNING_INCREMENT = 1*np.pi/180
BACKWARDS_TURNING_INCREMENT = 0.5*np.pi/180
MAX_SPEED = 0.54
LOST_LINE = 0
ALL_GOOD = 1
FOUND_LINE = -1
DUREE_IMP:float = 60/1000 # temps de l'impulsion
TEMPS_BEFORE_INTERRUPT:list[int] = [] # Liste des durée de chacunes des impulsions envoyées



# -----------------------------------------------------------------------------
#@params duree_impulsion : float : Durée qu'on donne pour détecter l'aller-retour
#de l'onde sonore. En secondes (s)
#return : Float: la distance maximale que le son va pouvoir parcourir 
# -----------------------------------------------------------------------------
def duree_Imp(duree_impulsion : float) -> float:
    return duree_impulsion*340/2

# -----------------------------------------------------------------------------
#@params distance_emetteur_obs : float: Distance parcourue avant de causer un 
#interrupt
#    
#return : float : Temps que le son a pris a parcourir la distance
# -----------------------------------------------------------------------------
def get_signal_duration(distance_emetteur_obs : float) -> float:
    return 2*distance_emetteur_obs/340

# -----------------------------------------------------------------------------
#@params vec : Vector : Position en (x,y,z) de l'émetteur
#        obs_bhv : BVHTree : Arbre des polygones pour valider la collision
#        overlap : list[tuple[int,int]] : Les positions ou un overlap a eu lieu
#        
#        Le but de cette fonction est de trouver quelles sont les distances ou 
#        les collisions ont eu lieu (on connait déjà les index des collisions
#        mais il faut les interpréter).
#        
#return : float : La distance minimale isolée.
# -----------------------------------------------------------------------------
def find_Min_In_Cone(vec : Vector, obs_bhv : BVHTree, overlap : list[tuple[int,int]]) -> float:
    distance_max:float = duree_Imp(DUREE_IMP) 
    if not(min_range_list := obs_bhv.find_nearest_range(vec,distance_max)):
        return -1
    _,index_obs = zip(*overlap)
    true_overlap :list[float] = []
    for  (_,_,id,d) in min_range_list:
        if id in index_obs : 
            true_overlap.append(d)
    return min(true_overlap,default=-1) #this to review


# -----------------------------------------------------------------------------
#@params obstacle_list : Liste des objects obstacles
#        cone_detection : Cone qui simule le FOV de notre voiture
#        emetteur_loc:Vector : Position d'émission du cone
#        
#Voir interface : sonar()->distance,temps:float,temps
#
#return : float : retourne l'obstacle le plus proche et dans notre champs de 
#        vision
# -----------------------------------------------------------------------------
def get_Closest_Obstacle(obstacle_list,cone_detection,emetteur_loc:Vector) -> float:
    dmin = -1
    vertexes_cone = [cone_detection.matrix_world @ v.co for v in cone_detection.data.vertices]
    polygons_cone = [p.vertices for p in cone_detection.data.polygons]
    cone_bhv = BVHTree.FromPolygons(vertexes_cone,polygons_cone)
    for obs_mesh in obstacle_list:
        if not obs_mesh.type == 'MESH':
            continue
        vertexes_obs = [ obs_mesh.matrix_world @ v.co for v in obs_mesh.data.vertices]
        polygons_obs = [ p.vertices for p in obs_mesh.data.polygons]
        obs_bhv = BVHTree.FromPolygons(vertexes_obs,polygons_obs)
        if not (overlap := cone_bhv.overlap(obs_bhv)) :
            continue
        if  ((dist := find_Min_In_Cone(emetteur_loc,obs_bhv,overlap)) > 0 and dist < dmin) or dmin == -1 :
                dmin = dist
    return dmin

# -----------------------------------------------------------------------------
#@params dist_min : float : La distance minimale mesurée
#
#return : None : Rien
# -----------------------------------------------------------------------------
def update_interupt_time(dist_min:float)->None:
    if dist_min == -1 :
        TEMPS_BEFORE_INTERRUPT.append(DUREE_IMP)
        return
    TEMPS_BEFORE_INTERRUPT.append(get_signal_duration(dist_min))
    return
# -----------------------------------------------------------------------------
#@params cone : Le cone simulant le FOV, qui est à modifier
#        angle:float : L'angle d'ouverture du cone (rad)
#        length:float : La longueur de l'axe Z (le plus grand), utilisé pour les 
#        calculs
#
#return : None : Rien
# -----------------------------------------------------------------------------
def change_cone(cone,angle:float,length:float)->None:
    (x,y,z) = (length*tan(angle),length*tan(angle),length)        
    cone.dimensions = (x,y,z)
    return


# -----------------------------------------------------------------------------
# read_captors
# checks if cylinders and lines are overlapping
# turns on lights if overlap (broken atm)
# returns the list of active captors
# -----------------------------------------------------------------------------
        
def read_captors(bhv_track, captors_cylinders, captors_lights, current_frame):
    captors_lights_index = 0
    captors_readings = [0, 0, 0, 0, 0]
    
    for captor_cylinder in captors_cylinders:
        captor_cylinder_position = captor_cylinder.matrix_world
        captor_cylinder_vertices = [captor_cylinder_position @ v.co for v in captor_cylinder.data.vertices]
        captor_cylinder_polygons = [p.vertices for p in captor_cylinder.data.polygons]
        bhv_captor = BVHTree.FromPolygons(captor_cylinder_vertices, captor_cylinder_polygons)

        cylinder_track_overlap = False
        for bhv_section in bhv_track:
            if bhv_section.overlap(bhv_captor):
                cylinder_track_overlap = True
        
        if cylinder_track_overlap:
            #captors_lights[captors_lights_index].data.energy = 1000
            captors_readings[captors_lights_index] = 1
        #else:
            #captors_lights[captors_lights_index].data.energy = 0

        #captors_lights[captors_lights_index].data.keyframe_insert(data_path="energy", frame = current_frame)
        captors_lights_index+=1

    return captors_readings

# -----------------------------------------------------------------------------
# get_bhv_track
# returns the path collection as bhv track
# -----------------------------------------------------------------------------

def get_bhv_track():
    path_collection = bpy.data.collections['path_collection'].all_objects
    bhv_track = []

    for section in path_collection:
        section_position = section.matrix_world
        section_vertices = [section_position @ v.co for v in section.data.vertices]
        section_polygons = [p.vertices for p in section.data.polygons]
        bvh_section = BVHTree.FromPolygons(section_vertices, section_polygons)
        bhv_track.append(bvh_section)
    
    return bhv_track

# -----------------------------------------------------------------------------
# get_target_angle
# returns the next target angle, in radians, according the the captors readings
# -----------------------------------------------------------------------------

def get_target_angle(captors_readings):
        readings_total = captors_readings[0] + \
                         captors_readings[1] + \
                         captors_readings[2] + \
                         captors_readings[3] + \
                         captors_readings[4]
        if readings_total == 0: 
            readings_total = 1
        target_angle = (captors_readings[0]/readings_total) * (45 * np.pi / 180) + \
                       (captors_readings[1]/readings_total) * (22.5 * np.pi / 180)  + \
                       (captors_readings[2]/readings_total) * (0) + \
                       (captors_readings[3]/readings_total) * (-22.5 * np.pi / 180) + \
                       (captors_readings[4]/readings_total) * (-45 * np.pi / 180)
        return target_angle

# -----------------------------------------------------------------------------
# rotate_car
# rotate every object in the car around the y axis
# targe_angle should be given in radians
# -----------------------------------------------------------------------------

def rotate_car(car, target_angle, current_state):
    car_current_angle = car[0].rotation_euler[2]
    angle = 0

    if (car_current_angle != target_angle + car_current_angle):
        if (car_current_angle < car_current_angle + target_angle):
            angle = -FORWARD_TURNING_INCREMENT
            #if(current_state == LOST_LINE): angle = -BACKWARDS_TURNING_INCREMENT
            #else:angle = -FORWARD_TURNING_INCREMENT
            
        elif (car_current_angle > car_current_angle + target_angle):
            angle = FORWARD_TURNING_INCREMENT
            #if(current_state == LOST_LINE): angle = BACKWARDS_TURNING_INCREMENT
            #else:angle = FORWARD_TURNING_INCREMENT
    for object in car: object.rotation_euler[2] = object.rotation_euler[2]+angle

# -----------------------------------------------------------------------------
# move_car
# moves the car according to its speed and rotation
# -----------------------------------------------------------------------------

def move_car(car, current_speed):
    for object in car:
        object.location[0] = object.location[0]-current_speed*-np.sin(car[0].rotation_euler[2])
        object.location[1] = object.location[1]-current_speed*np.cos(car[0].rotation_euler[2])

# -----------------------------------------------------------------------------
# change_car_speed
# change the current speed of the car
# -----------------------------------------------------------------------------

def change_car_speed(increment, current_speed):
    if increment > 0 and current_speed + increment <= MAX_SPEED:
        current_speed = current_speed + increment
    elif increment < 0 and abs(current_speed + increment) <= MAX_SPEED :
        current_speed = current_speed + increment

    return current_speed

# -----------------------------------------------------------------------------
# follow_line
# regular algorithm that follows the line
# return the status of the line following, the target angle and current speed
# -----------------------------------------------------------------------------

def follow_line(captors_cylinders, captors_lights, current_speed, target_angle, frame_counter):
    captors_readings = read_captors(get_bhv_track(), captors_cylinders, captors_lights, frame_counter)
    if captors_readings == [0, 0, 0, 0, 0]:
        return LOST_LINE, -target_angle, current_speed
    target_angle = get_target_angle(captors_readings)   
    current_speed = change_car_speed(FORWARD_ACCELERATION, current_speed)
    return ALL_GOOD, target_angle, current_speed

# -----------------------------------------------------------------------------
# searching_line
# algorithm that searches for the line
# -----------------------------------------------------------------------------

def searching_line(captors_cylinders, captors_lights, current_speed, target_angle, frame_counter):    
    captors_readings = read_captors(get_bhv_track(), captors_cylinders, captors_lights, frame_counter)  
    if captors_readings != [0, 0, 0, 0, 0]:
        return FOUND_LINE, -target_angle, current_speed    
    current_speed = change_car_speed(BACKWARD_ACCELERATION, current_speed)
    return LOST_LINE, target_angle, current_speed

# -----------------------------------------------------------------------------
#@params : obs_list,cone,emetteur_pos, les mêmes paramêtre pour appeler get_Closest_Obstacle
#
#return : Tuple[float,float] : Un tuple contenant la distance minimale et le temps que l'appel devrait prendre
# -----------------------------------------------------------------------------
def sonar(obs_list,cone,emetteur_pos)->tuple[float,float]:
    dmin = get_Closest_Obstacle(obs_list,cone,emetteur_pos)
    update_interupt_time(dmin)
    return (dmin,TEMPS_BEFORE_INTERRUPT[-1])



# -----------------------------------------------------------------------------
# main
# simulation of the line following algorithm for the car
# -----------------------------------------------------------------------------              

def main():
    captors_cylinders = bpy.data.collections['invisible_cylinders_collection'].all_objects
    captors_lights = bpy.data.collections['follower_captors_collection'].all_objects
    car = bpy.data.collections['car_collection'].all_objects
    obstacles = bpy.data.collections['obstacles_collection'].all_objects
    cone = bpy.data.objects['invisble_cone']
    emetteur_pos = cone.location

    
    frame_counter = 0
    current_speed = 0
    target_angle = 0
    current_state = ALL_GOOD
    angle_momento = 0
    
    while frame_counter < 2000:
        bpy.context.view_layer.update()
        dmin,temps_dmin = sonar(obs_list=obstacles,cone=cone,emetteur_pos=emetteur_pos) # Do some shit to make it work
        if current_state == ALL_GOOD:
            current_state, target_angle, current_speed = follow_line(captors_cylinders, captors_lights, current_speed, target_angle, frame_counter)

        elif current_state == LOST_LINE and current_speed >= 0:
            
            #if(angle_momento == 0 and target_angle != 0): 
            angle_momento = -target_angle
            target_angle = 0
            current_speed = change_car_speed(BACKWARD_ACCELERATION, current_speed)

        elif current_state == LOST_LINE and current_speed <= 0:
            target_angle = angle_momento
            current_state, target_angle, current_speed = searching_line(captors_cylinders, captors_lights, current_speed, target_angle, frame_counter)
            
        elif current_state == FOUND_LINE and current_speed <= 0:
            current_speed = change_car_speed(FORWARD_ACCELERATION, current_speed)
            angle_momento = 0
        
        elif current_state == FOUND_LINE and current_speed >= 0:
            current_state = ALL_GOOD
                
        move_car(car, current_speed)
        rotate_car(car, target_angle, current_state)
            
        for object in car:
            object.keyframe_insert(data_path="location", frame = frame_counter)
            object.keyframe_insert(data_path="rotation_euler", frame = frame_counter)
        
        frame_counter+=1

# -----------------------------------------------------------------------------
# starts the simulation
# -----------------------------------------------------------------------------  

main()

bpy.ops.screen.animation_play()

# -----------------------------------------------------------------------------
# end
# -----------------------------------------------------------------------------  