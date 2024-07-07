import blenderproc as bproc
import numpy as np
import os
import bpy
import random
import math
import time

start = time.time()
bproc.init()
# bpy.context.scene.view_settings.exposure = -3

# Load room with bricks:
objs = bproc.loader.load_blend("/home/reddy/blender_files/roomPalette.blend")

bricks = []
for i, item in enumerate(objs):
        if item.get_name().startswith("Brick"):
            bricks.append(item)
            item.set_cp("category_id", i + 1)
        else:
            item.set_cp("category_id", 0)
        item.set_shading_mode("AUTO")

def hide_bricks(brick_hide):
    for brick in brick_hide:
        print(brick.get_name(), "Hidden")
        brick.hide()    

pallet = bproc.filter.one_by_attr(objs, "name", "Pallet")
# print(pallet.get_rotation_euler())
# print(pallet.get_location())
# print(pallet.get_rotation_mat())
# print(pallet.get_local2world_mat())
z = 0.17
pallet_corners = [(-0.3047, -0.4447, z), (-0.3247, 0.3142, z), (0.2456, -0.4405, z), (0.2508, 0.3395, z)]
top_sections = [(-0.3197, -0.2549, z), (-0.3147, -0.0652, z), (-0.3197, 0.1244, z)]
bottom_sections = [(0.2469, -0.2455, z), (0.2482, -0.0505, z), (0.2495, 0.1445, z)]
left_mid = (-0.0295, -0.4426, z)
right_mid = (-0.0369, 0.3268, z)


section_mids = [
    (-0.17141, -0.34065+0.06, z),   # Midpoint of Box 1
    (-0.17492, -0.14+0.06, z),      # Midpoint of Box 2
    (-0.17567, 0.040825+0.06, z),   # Midpoint of Box 3
    (-0.17901, 0.2223125+0.06, z),  # Midpoint of Box 4
    (0.107815, -0.33725+0.06, z),   # Midpoint of Box 5
    (0.107455, -0.134025+0.06, z),  # Midpoint of Box 6
    (0.107355, 0.049525+0.06, z),   # Midpoint of Box 7
    (0.107165, 0.2336625+0.06, z)   # Midpoint of Box 8
]

def get_random_pose(theta, z = 2, r = 1.1283):
    get_coordinates = lambda theta: (r * math.cos(theta), r * math.sin(theta))
    x, y= get_coordinates(theta)
    dx = -x
    dy = -y
    dz = -z
    rot_x = math.atan2(dy, dz)
    rot_y = math.atan2(dx, dz)
    rot_z = 0
    return [[x, y, z], [rot_x, rot_y, rot_z]]


def set_lighting(Light1 = [[-0.24915, 0, 2.2222], [-0.5405, -0, 1.57079], 20], Light2 = [[0, -1.1283, 1.829], [0.4450, -0, 0], 50]):
    global light1, light2
    light1.set_location(Light1[0])
    light1.set_rotation_euler(Light1[1])
    light1.set_energy(Light1[2])
    light2.set_location(Light2[0])
    light2.set_rotation_euler(Light2[1])
    light2.set_energy(Light2[2])

def set_camera(mat = None):
    if mat is None:
        mat = [[-0.026167, -0.023818 , 0.7], [0, 0, 1.5707]]
    cam_pose = bproc.math.build_transformation_mat(mat[0], mat[1])
    bproc.camera.add_camera_pose(cam_pose)
    bproc.camera.set_resolution(1920, 1080)
    bproc.camera.set_intrinsics_from_blender_params(lens=1.67552, lens_unit="FOV")

def select_bricks():
    global bricks
    bricks_temp = []
    chosen = []
    for i in range(8):
        bricks_temp.append(bricks.pop(np.random.randint(0, len(bricks))))
        chosen.append(bricks_temp[-1])
    bricks += bricks_temp
    return chosen

def manipulate_brick(brick, place):
    brick.hide(False)
    brick.set_rotation_euler([0, 0, np.random.uniform(-0.2, 0.2)])
    brick.set_location(section_mids[place])

def render_scene():
    # Render the scene
    bproc.renderer.enable_normals_output()
    bproc.renderer.enable_segmentation_output(map_by=["category_id", "instance", "name"], default_values={"category_id": None})
    bproc.renderer.set_max_amount_of_samples(2048)
    bproc.renderer.set_denoiser("INTEL")
    bproc.renderer.set_output_format(file_format="JPEG", jpg_quality=150)
    data = bproc.renderer.render()

    # Write the rendering into an hdf5 file
    bproc.writer.write_coco_annotations(os.path.join("/data/reddy/", 'coco_data'),
                                        instance_segmaps=data["instance_segmaps"],
                                        instance_attribute_maps=data["instance_attribute_maps"],
                                        colors=data["colors"],
                                        color_file_format="JPEG")
    # bproc.writer.write_hdf5("output/", data)
    
#At an angle
bpy.ops.object.light_add(type='AREA')
bpy_light1 = bpy.context.object
light1 = bproc.types.Light(blender_obj=bpy_light1)
#Upper
bpy.ops.object.light_add(type='AREA')
bpy_light2 = bpy.context.object
light2 = bproc.types.Light(blender_obj=bpy_light2)
set_camera()
set_lighting()
hide_bricks(bricks)

for run in range(1000):
    chosen = select_bricks()
    random_list = random.sample(range(0,8), 8)
    for i, j in enumerate(chosen):
        print("Manipulating Brick", j.get_name())
        manipulate_brick(j, random_list[i])
    Light1 = get_random_pose(random.randint(0,359), z = 2.222, r = 0.25) + [20]
    Light2 = get_random_pose(random.randint(0,359)) + [50]
    set_lighting(Light1, Light2)
    render_scene()
    hide_bricks(chosen)

print("Time taken:", time.time() - start)