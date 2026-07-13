import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    gw_monitor_all_topics_arg = DeclareLaunchArgument(
        'gw_monitor_all_topics',
        default_value='true',
        description='Auto-discover and monitor every ROS topic via greenwave_monitor. '
                    'Set false to only track gw_monitored_topics.',
    )

    gw_monitored_topics_arg = DeclareLaunchArgument(
        'gw_monitored_topics',
        default_value='[""]',
        description='List of topics for greenwave_monitor to track (rate/latency).',
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Show greenwave_monitor\'s full per-second rate/latency log instead '
                    'of just genuine WARN/ERROR-level issues.',
    )

    gw_log_level = PythonExpression([
        "'info' if '", LaunchConfiguration('verbose'), "' == 'true' else 'warn'"
    ])

    greenwave_monitor_node = Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name='greenwave_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', gw_log_level],
        parameters=[{
            'gw_monitored_topics': LaunchConfiguration('gw_monitored_topics'),
            'use_sim_time': False,
        }],
    )

    greenwave_auto_discovery_node = Node(
        package='hydrakon_bringup',
        executable='greenwave_auto_discovery',
        name='greenwave_auto_discovery',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gw_monitor_all_topics')),
    )

    return LaunchDescription([
        gw_monitor_all_topics_arg,
        gw_monitored_topics_arg,
        verbose_arg,
        zedx_bringup_launch,
        cone_marker_publisher_node,
        greenwave_monitor_node,
        greenwave_auto_discovery_node,
    ])
