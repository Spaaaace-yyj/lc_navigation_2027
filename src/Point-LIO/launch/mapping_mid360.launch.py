import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('point_lio'),
        'config', 'mid360.yaml')

    return LaunchDescription([
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[config],
        ),
    ])