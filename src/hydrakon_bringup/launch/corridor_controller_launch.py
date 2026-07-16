from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rosbag_record_action(launch_name):
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            'mkdir -p ~/rosbags && exec ros2 bag record -a --storage mcap '
            f'-o ~/rosbags/{launch_name}_$(date +%Y%m%d_%H%M%S)'
        ],
        prefix='setsid',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )


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
        # rosbag_record_action('corridor_controller'),
    ])
