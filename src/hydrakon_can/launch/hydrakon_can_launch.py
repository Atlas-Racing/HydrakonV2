from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="hydrakon_can",
        executable="hydrakon_can_node",
        name="hydrakon_can",
        parameters=[
            {"use_sim_time": False},
            {"can_debug": 1},
            {"simulate_can": 1},
            {"can_interface": "vcan0"},
            {"loop_rate": 100}, # keep as int
            {"rpm_limit": 300.0}, # must be float!
            {"max_acc": 5.0},
            {"max_braking": 5.0},
            {"cmd_timeout": 0.5}
        ],
        # arguments=['--ros-args', '--log-level', 'debug'],

        # launch the node using ros2 launch hydrakon_can hydrakon_can_launch.py, don't forget to source your workspace first,
        # for sim, use sim_time as True, simulate_can as 1, and can_interface as vcan0, sometimes you may need to set sim_time to False,
        # for the ADS-DV, use sim_time as False, simulate_can as 0, and can_interface as whatever your can interface is called on your system,
        # on the Jetson AGX Orin it was can2 as the Jetson already has 2 can modules built-in
    )
    # when you encounter a make error, go to src/hydrakon_can/include/hydrakon_can/FS-AI_API/FS-AI_API and then run 'make clean' then 'make'
    # if more errors show up, try rm -rf build/ install/ log/ and then run colcon build again, always do colcon build --symlink-install as
    # it's faster
    ld = LaunchDescription()
    ld.add_action(node)
    return ld