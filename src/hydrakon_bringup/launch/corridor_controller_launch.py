from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('controller_mode').perform(context)
    executable = 'corridor_controller_2d' if mode == '2d' else 'corridor_controller_3d'
    ami_states = LaunchConfiguration('active_ami_states').perform(context).split(',')

    return [Node(
        package='hydrakon_planner',
        executable=executable,
        name='corridor_controller',
        output='screen',
        parameters=[{'active_ami_states': ami_states}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_mode',
            default_value='2d',
            description=(
                "Which corridor controller to run: '2d' (pixel bbox based) or "
                "'3d' (metric position based). Mutually exclusive - both publish "
                "to /hydrakon_can/command, do not run both simultaneously."
            ),
            choices=['2d', '3d'],
        ),
        DeclareLaunchArgument(
            'active_ami_states',
            default_value='TRACKDRIVE,AUTOCROSS',
            description='Comma-separated AMI mission names this controller is active for.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
