import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_bringup')

    livox_pkg_dir = get_package_share_directory('livox_ros_driver2')
    point_lio_pkg_dir = get_package_share_directory('point_lio')

    livox_mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg_dir,'launch', 'msg_MID360_launch.py')
        )
    )

    point_lio_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_pkg_dir,'launch','mapping_mid360.launch.py')
        )
    )

    rviz_config = os.path.join(bringup_dir,'rviz','mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    delayed_point_lio = TimerAction(
        period=3.0,
        actions=[point_lio_mapping_launch]
    )
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        livox_mid360_launch,
        delayed_point_lio,
        delayed_rviz,
    ])

