import blenderproc as bproc
import numpy as np
import os
import bpy

bproc.init()
bpy.context.scene.view_settings.exposure = -3

# Create a simple object:
objs = bproc.loader.load_blend("/home/fsociety/Code/Projects/Project-BRICS/blender_files/roomPalette.blend")
# bricks_gltf = bproc.loader.load_obj("blender_files/Bricks.glb")

for i in objs:
    i.set_shading_mode("AUTO")
    i.set_cp("category_id", 0)
    
# for brick in bricks_gltf:
#     brick.hide()
#     brick.set_cp("category_id", 0)


brick1 = bproc.loader.load_obj("blender_files/Brick_1.glb")[0]
brick2 = bproc.loader.load_obj("blender_files/Brick_2.glb")[0]
brick3 = bproc.loader.load_obj("blender_files/Brick_3.glb")[0]
brick4 = bproc.loader.load_obj("blender_files/Brick_4.glb")[0]
brick5 = bproc.loader.load_obj("blender_files/Brick_5.glb")[0]
brick6 = bproc.loader.load_obj("blender_files/Brick_6.glb")[0]
brick7 = bproc.loader.load_obj("blender_files/Brick_7.glb")[0]
brick8 = bproc.loader.load_obj("blender_files/Brick_8.glb")[0]
brick9 = bproc.loader.load_obj("blender_files/Brick_9.glb")[0]
brick10 = bproc.loader.load_obj("blender_files/Brick_10.glb")[0]


bricks = [brick1, brick2, brick3, brick4, brick5, brick6, brick7, brick8, brick9, brick10]

for i in bricks:
    i.set_location([0, 0, 20])

chosen = []
for i in range(8):
    chosen.append(bricks.pop(np.random.randint(0, len(bricks))))

pallet = objs[0]
z = 0.17
pallet_corners = [(-0.3047, -0.4447, z), (-0.3247, 0.3142, z), (0.2456, -0.4405, z), (0.2508, 0.3395, z)]
top_sections = [(-0.3197, -0.2549, z), (-0.3147, -0.0652, z), (-0.3197, 0.1244, z)]
bottom_sections = [(0.2469, -0.2455, z), (0.2482, -0.0505, z), (0.2495, 0.1445, z)]
left_mid = (-0.0295, -0.4426, z)
right_mid = (-0.0369, 0.3268, z)


section_mids = [
    (-0.17141, -0.34065, z),   # Midpoint of Box 1
    (-0.17492, -0.14, z),      # Midpoint of Box 2
    (-0.17567, 0.040825, z),   # Midpoint of Box 3
    (-0.17901, 0.2223125, z),  # Midpoint of Box 4
    (0.107815, -0.33725, z),   # Midpoint of Box 5
    (0.107455, -0.134025, z),  # Midpoint of Box 6
    (0.107355, 0.049525, z),   # Midpoint of Box 7
    (0.107165, 0.2336625, z)   # Midpoint of Box 8
]

# section_mids = [
#     (-0.16285, -0.31199, z),
#     (-0.16285, -0.15599, z),
#     (-0.16285, 0, z),
#     (-0.16285, 0.15599, z),
#     (-0.16285, 0.31199, z),
#     (0.14377, -0.31199, z),
#     (0.14377, -0.15599, z),
#     (0.14377, 0, z),
#     (0.14377, 0.15599, z),
#     (0.14377, 0.31199, z),
# ]

for i, j in enumerate(chosen):
    print(i)
    j.set_location(section_mids[i])
    j.set_shading_mode("AUTO")
    j.set_cp("category_id", i + 1)

# Create a point light next to it
light1 = bproc.types.Light()
light1.set_type("AREA")
light1.set_location([1.06152, 0, 1.32602])
light1.set_rotation_euler([0, 0.613486, 0])
light1.set_energy(100)

light2 = bproc.types.Light()
light2.set_type("AREA")
light2.set_location([-0.56779, 0, 1.18143])
light2.set_rotation_euler([0, -0.347779, 0])
light2.set_energy(50)

def set_camera(mat = None):
    if mat is None:
        mat = [[-0.046167, -0.063818 , 0.7], [-0.039095, 0, 1.5707]]
    cam_pose = bproc.math.build_transformation_mat(mat[0], mat[1])
    bproc.camera.add_camera_pose(cam_pose)
    bproc.camera.set_resolution(640, 480)
    bproc.camera.set_intrinsics_from_blender_params(lens=1.67552, lens_unit="FOV")
set_camera()
def manipulate_brick(brick):
    pass
# Render the scene
bproc.renderer.enable_normals_output()
bproc.renderer.enable_segmentation_output(map_by=["category_id", "instance", "name"], default_values={"category_id": None})
bproc.renderer.set_max_amount_of_samples(1024)
bproc.renderer.set_denoiser("INTEL")
data = bproc.renderer.render()

# Write the rendering into an hdf5 file
bproc.writer.write_coco_annotations(os.path.join("BlenderProc/examples/advanced/coco_annotations/output", 'coco_data'),
                                    instance_segmaps=data["instance_segmaps"],
                                    instance_attribute_maps=data["instance_attribute_maps"],
                                    colors=data["colors"],
                                    color_file_format="JPEG")
# bproc.writer.write_hdf5("output/", data)
