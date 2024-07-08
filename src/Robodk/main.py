import os
import time
import json
from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox

# Initialize the RoboDK API
RDK = robolink.Robolink()

# Define the path to the folder containing the bricks and the JSON file
brick_folder_path = "D:\\Robodk\\bricks"
json_file_path = "D:\\Robodk\\brick_placement.json"

# Load the JSON file
with open(json_file_path, 'r') as json_file:
    brick_data = json.load(json_file)

# Retrieve items
pallet_frame = RDK.Item("pallet_frame")
usable_frame = RDK.Item("usable_frame")  # Frame for the second pallet
robot = RDK.Item('UR5')
ur5_frame = RDK.Item('UR5_Base')

# Check if the items were found
if not robot.Valid() or not ur5_frame.Valid() or not pallet_frame.Valid() or not usable_frame.Valid():
    raise Exception('One or more items could not be found. Check the item names.')

# Import all bricks at once and set their poses
brick_items = {}
for brick_name, (position, rotation) in brick_data.items():
    brick_file = f"{brick_name}.glb"
    brick_path = os.path.join(brick_folder_path, brick_file)
    brick_item = RDK.AddFile(brick_path, pallet_frame)
    
    if brick_item.Valid():
        # Create a pose from the position and rotation data
        x, y, z = position
        rx, ry, rz = rotation
        
        # Create a 4x4 transformation matrix
        pose = robomath.transl(x * 1000, y * 1000, (z * 1000 + 20)) * robomath.rotz(rz) * robomath.roty(ry) * robomath.rotx(rx)
        
        # Set the pose of the brick 
        # Had to rotx as the blender export was a bit wanky
        brick_item.setPose(pose * robomath.rotx(90 * robomath.pi / 180))
        
        brick_items[brick_name] = brick_item
    else:
        print(f"Failed to import brick: {brick_file}")

# Joint positions for home and scan positions
home_robot_joint_pos = [ 98,-77,-58,-90,90,0 ]

# Got the joint position when kept on the inspection_frame
scan_robot_joint_pos = [80.75,-51.35,7.47,-45.86,-91.05,36.23]

# Process each brick
for brick_name, (position, rotation) in brick_data.items():
    if brick_name not in brick_items:
        print(f"Skipping {brick_name} as it was not imported successfully")
        continue

    brick_item = brick_items[brick_name]
    
    # Determine if the brick is usable or unusable based on its name
    unusable = "unusable" in brick_name.lower()
    
    # Get the absolute pose for all the items
    brick_pose_abs = brick_item.PoseAbs()
    pallet_pose_abs = pallet_frame.PoseAbs()
    usable_pallet_pose_abs = usable_frame.PoseAbs()
    ur5_pose_abs = ur5_frame.PoseAbs()

    # Calculate the pose of the brick with respect to the pallet frame
    brick_pose_pallet = robomath.invH(pallet_pose_abs) * brick_pose_abs


    """
    If usable : 
        Pick the brick -> Go the scan -> Back on the pallet -> Detach

    If unusable :
        Place on the table directly without scanning
    """

    if not unusable:
        # Define a target pose just above the brick
        target_pose_pallet = brick_pose_pallet * robomath.transl(-5, 10, 0)  # Z offset to go above the brick

        # Transform the target pose from the pallet frame to the UR5 frame
        target_pose_ur5 = robomath.invH(ur5_pose_abs) * pallet_pose_abs * target_pose_pallet
        target_pose_ur5_rotated = target_pose_ur5 * robomath.roty(-90 * robomath.pi / 180) * robomath.rotx(90 * robomath.pi / 180) * robomath.rotz(180 * robomath.pi / 180)

        # Add the target to the station
        target = RDK.AddTarget('target', ur5_frame)
        target.setPose(target_pose_ur5_rotated)

        # Move the robot to the new target
        robot.MoveL(target)
        
        closed_brick = RDK.Item("closed_brick")
        RDK.Item("Gripper").MoveJ(closed_brick)

        RDK.Item("gripping_tool").AttachClosest()

        robot.MoveJ(scan_robot_joint_pos)
        time.sleep(0.8)

        robot.MoveJ(home_robot_joint_pos)

        robot.MoveL(target)

        detach_prog = RDK.Item("Detach")
        RDK.RunCode(detach_prog.Name())
        target.Delete()

        robot.MoveJ(home_robot_joint_pos)

        print(f"bricks usable {brick_name} at position: {position}, rotation: {rotation}")
    else:
        # Define a target pose just above the brick
        target_pose_pallet = brick_pose_pallet * robomath.transl(-5, 10, 0)  # Z offset to go above the brick

        # Transform the target pose from the pallet frame to the UR5 frame
        target_pose_ur5 = robomath.invH(ur5_pose_abs) * pallet_pose_abs * target_pose_pallet
        target_pose_ur5_rotated = target_pose_ur5 * robomath.roty(-90 * robomath.pi / 180) * robomath.rotx(90 * robomath.pi / 180) * robomath.rotz(180 * robomath.pi / 180)

        # Add the target to the station
        target = RDK.AddTarget('target', ur5_frame)
        target.setPose(target_pose_ur5_rotated)

        # Move the robot to the new target
        robot.MoveL(target)
        
        closed_brick = RDK.Item("closed_brick")
        RDK.Item("Gripper").MoveJ(closed_brick)

        RDK.Item("gripping_tool").AttachClosest()
        target.Delete()
        robot.MoveJ(home_robot_joint_pos)

        robot.MoveJ([14.590000, -82.700000, -92.430000, -72.970000, 90.000000, 0.000000])

        new_target = RDK.AddTarget("second_pallet", ur5_frame)
        new_target_pose_ur5 = robomath.invH(ur5_pose_abs) * usable_pallet_pose_abs * target_pose_pallet
        new_target_pose_ur5_rotated = new_target_pose_ur5 * robomath.roty(-90 * robomath.pi / 180) * robomath.rotx(90 * robomath.pi / 180) 

        new_target.setPose(new_target_pose_ur5_rotated * robomath.transl(0,0,-10))

        robot.MoveL(new_target)
        
        detach_prog = RDK.Item("Detach")
        RDK.RunCode(detach_prog.Name())

        robot.MoveJ(home_robot_joint_pos)
        new_target.Delete()

        print(f"bricj=k unusable {brick_name} at position: {position}, rotation: {rotation}")

print("All bricks have been processed.")
