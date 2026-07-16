from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ami_states = LaunchConfiguration('active_ami_states').perform(context).split(',')

    return [Node(
        package='hydrakon_planner',
        executable='skidpad_controller',
        name='skidpad_controller',
        output='screen',
        parameters=[{'active_ami_states': ami_states}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'active_ami_states',
            default_value='SKIDPAD',
            description='Comma-separated AMI mission names this controller is active for.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
