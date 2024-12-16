import blenderproc as bproc
import numpy as np
import os
import bpy
import random
import math
import time
import sys
import mathutils

bproc.init()

scene = bproc.loader.load_blend("blender_files/SingleBrick_Environment.blend")
bpy.context.scene.cycles.samples = 2048
bpy.context.scene.cycles.use_light_tree = True
bpy.context.scene.cycles.max_bounces = 12        # Maximum total bounces
bpy.context.scene.cycles.diffuse_bounces = 4      # Maximum diffuse bounces
bpy.context.scene.cycles.glossy_bounces = 4       # Maximum glossy bounces
bpy.context.scene.cycles.transmission_bounces = 12  # Maximum transmission bounces
bpy.context.scene.cycles.volume_bounces = 2       # Maximum volume bounces
bpy.context.scene.render.use_simplify = False
bpy.context.scene.cycles.use_spatial_splits = False
bpy.context.scene.cycles.use_persistent_data = False
bpy.context.scene.view_settings.exposure = -3

greenscreen = bpy.data.objects['GreenScreen']
greenscreen = bproc.filter.one_by_attr(scene, "name", "GreenScreen")

for i, item in enumerate(scene):
        item.set_cp("category_id", 0)
        item.set_shading_mode("AUTO")


def import_brick(path):
    bpy.ops.import_scene.gltf(filepath=path)

    brick_obj = bpy.context.selected_objects[0]
    brick_obj.location = (0, 0, 0)
    place_obj1_on_top_of_obj2(brick_obj, greenscreen)
    bpy.context.view_layer.update()

    return brick_obj

def hide_bricks(brick_hide: list):
    for brick in brick_hide:
        print(brick.get_name(), "Hidden")
        brick.hide()

def place_obj1_on_top_of_obj2(obj1, obj2):
    global touch_z

    # Handle obj1 bounding box
    if isinstance(obj1, bpy.types.Object):  # Native Blender object
        bbox1 = [obj1.matrix_world @ mathutils.Vector(corner) for corner in obj1.bound_box]
    else:  # BlenderProc object
        bbox1 = [mathutils.Vector(corner) for corner in obj1.get_bound_box()]  # Convert to Vector

    # Handle obj2 bounding box
    if isinstance(obj2, bpy.types.Object):  # Native Blender object
        bbox2 = [obj2.matrix_world @ mathutils.Vector(corner) for corner in obj2.bound_box]
    else:  # BlenderProc object
        bbox2 = [mathutils.Vector(corner) for corner in obj2.get_bound_box()]  # Convert to Vector

    # Get min and max Z values for each object
    min_z1 = min(v.z for v in bbox1)
    max_z2 = max(v.z for v in bbox2)
    print(f"Min Z of obj1: {min_z1}, Max Z of obj2: {max_z2}")

    # Calculate the required Z offset
    offset_z = max_z2 - min_z1

    # Update obj1 location
    if isinstance(obj1, bpy.types.Object):
        obj1.location.z += offset_z
    else:
        current_pose = obj1.get_location()
        obj1.set_location([current_pose[0], current_pose[1], current_pose[2] + offset_z])

    touch_z = offset_z
    print(f"Placed {obj1.name if isinstance(obj1, bpy.types.Object) else obj1.get_name()} on top of {obj2.name if isinstance(obj2, bpy.types.Object) else obj2.get_name()} at Z = {offset_z}")

def set_camera(pose=[[0, -0.1, 0.45], [0, 0, 0]]):
    pose_matrix = bproc.math.build_transformation_mat(pose[0], pose[1])
    bproc.camera.add_camera_pose(pose_matrix)
    bproc.camera.set_resolution(1280, 720)
    bproc.camera.set_intrinsics_from_blender_params(lens=1.518, lens_unit="FOV")

def set_lighting(Lights=[{"name": "Light_Area_1", "translation": [-0.36, 0.4, 0.55], "rotation": [-1.13446, -0.15708, 0.66322], "intensity": 20}, 
                         {"name": "Light_Area_2", "translation": [0.36, 0.4, 0.55], "rotation": [-1.13446, -0.05235, -0.75921], "intensity": 20}, 
                        #  {"name": "Light_Area_3", "translation": [-0.36, -0.4, 0.55], "rotation": [-1.03966, 0.20843, 2.12499], "intensity": 10}, 
                        #  {"name": "Light_Area_4", "translation": [0.36, -0.4, 0.55], "rotation": [-1.06702, -0.05213, -2.02201], "intensity": 10}, 
                        #  {"name": "Overhead", "translation": [0, 0, 0.6], "rotation": [0, 0, 0], "intensity": 10}
                         ]):
    for light in Lights:
        # Check if a light with the same name already exists
        existing_light = bpy.data.objects.get(light["name"])
        
        if existing_light:
            # If the light exists, wrap it with BlenderProc
            bpy_light = existing_light
            bproc_light = bproc.types.Light(blender_obj=bpy_light)
        else:
            # If the light doesn't exist, create a new one
            bpy.ops.object.light_add(type='AREA', radius=0.3)
            bpy_light = bpy.context.object
            bpy_light.name = light["name"]  # Set the name
            bproc_light = bproc.types.Light(blender_obj=bpy_light)

        # Update the light properties
        bproc_light.set_location(light["translation"])
        bproc_light.set_rotation_euler(light["rotation"])
        bproc_light.set_energy(light["intensity"])

def place_camera_on_hemisphere(radius=0.2, center=(0, 0, 0)):
    # Generate random spherical coordinates
    theta = random.uniform(0, 2 * math.pi)  # Azimuthal angle (0 to 2*pi)
    phi = random.uniform(0, math.pi / 2)    # Polar angle (0 to pi/2 for hemisphere)

    # Convert spherical coordinates to Cartesian coordinates
    x = radius * math.sin(phi) * math.cos(theta) + center[0]
    y = radius * math.sin(phi) * math.sin(theta) + center[1]
    z = radius * math.cos(phi) + center[2]

    # Get the current camera location from BlenderProc
    camera_location = [x, y, 0.5]
    
    # Calculate the rotation required for the camera to look at the object's center
    direction = (mathutils.Vector(center) - mathutils.Vector(camera_location)).normalized()
    rotation_quaternion = direction.to_track_quat('-Z', 'Y')
    rotation_euler = rotation_quaternion.to_euler()

    # Build the transformation matrix for the camera pose (fixed location with new rotation)
    camera_pose = bproc.math.build_transformation_mat(camera_location, rotation_euler)

    # Register the calculated camera pose with BlenderProc
    bproc.camera.add_camera_pose(camera_pose, frame=0)

def set_greenscreen_color(color):
    if type(color) is not str:
        r, g, b = color
    r, g, b = int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)

    rgb_curves_node = bpy.data.objects.get("GreenScreen").active_material.node_tree.nodes["RGB Curves"]
    rgb_curves_node.inputs[1].default_value = (r / 255.0, g / 255.0, b / 255.0, 1)

set_camera()
set_lighting()

brick_paths = os.listdir("blender_files/bricks")
brick_path = random.choice(brick_paths)
import_brick("blender_files/bricks/" + brick_path)
place_camera_on_hemisphere()
set_greenscreen_color("FFFFFF")
# ## Switch the viewport to camera view
# for area in bpy.context.screen.areas:
#     if area.type == 'VIEW_3D':  # Ensure it's the 3D viewport
#         for space in area.spaces:
#             if space.type == 'VIEW_3D':
#                 space.region_3d.view_perspective = 'CAMERA'
#                 break
# bpy.context.view_layer.update()