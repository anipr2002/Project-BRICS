from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_robot_driver'), '/launch/ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.56.101',
            'launch_rviz': 'false'
        }.items()
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "world", "base_link"],
        output="screen",
    )



    return LaunchDescription([
        ur_control_launch,
        static_transform_publisher,
    ])


    # your_node = Node(
    #     package='your_package_name',
    #     executable='your_node_script',
    #     name='your_node_name',
    #     output='screen'
    # )
