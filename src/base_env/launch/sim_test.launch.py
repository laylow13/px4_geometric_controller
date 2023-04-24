import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    sim_base_node = Node(
            package='base_env',
            executable='sim_base',
        )
    trajectory_generator_node = Node(
            package='base_env',
            executable='trajectory_generator',
        ) 
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('controller'),
                'launch/se3_controller_sim.launch.py'))
    )
    return LaunchDescription([
        sim_base_node,
        trajectory_generator_node,
        launch_include
    ])