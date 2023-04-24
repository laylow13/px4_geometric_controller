import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ext_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('base_env'),
                'launch/ext_pose_to_fcu.launch.py'))
    )
    mocap_convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('base_env'),
                'launch/mocap_data_convert.launch.py'))
    )
    trajectory_generator_node = Node(
            package='base_env',
            executable='trajectory_generator',
        ) 
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('controller'),
                'launch/se3_controller.launch.py'))
    )
    return LaunchDescription([
        ext_pose_launch,
        mocap_convert_launch,
        trajectory_generator_node,
        controller_launch
    ])