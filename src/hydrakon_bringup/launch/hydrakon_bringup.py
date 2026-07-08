import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    hydrakon_bringup_dir = get_package_share_directory('hydrakon_bringup')

    zedx_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hydrakon_bringup_dir, 'launch', 'zedx_bringup.py')
        )
    )

    cone_marker_publisher_node = Node(
        package='hydrakon_perception',
        executable='cone_marker_publisher',
        name='cone_marker_publisher',
        output='screen',
    )

    # TODO: Add other bringup files here as they are created

    return LaunchDescription([
        zedx_bringup_launch,
        cone_marker_publisher_node,
    ])
