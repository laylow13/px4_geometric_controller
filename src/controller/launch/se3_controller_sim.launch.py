import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('controller'),
        'cfg',
        'sim.yaml'
        )

    return LaunchDescription([
        Node(
            package='controller',
            executable='se3_geometric_controller',
            name='controller_node',
            parameters=[param]
        )
    ])