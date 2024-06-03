import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('base_env'),
        'cfg',
        'mocap2fcu.yaml'
        )

    return LaunchDescription([
        Node(
            package='base_env',
            executable='external_pose_to_fcu',
            name='external_pose_to_fcu',
            parameters=[param]
        )
    ])