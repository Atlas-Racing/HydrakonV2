from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="hydrakon_can",
        executable="hydrakon_can_node",
        name="hydrakon_can",
        parameters=[
            {"use_sim_time": False},
            {"can_debug": 1},
            {"simulate_can": 0},
            {"can_interface": "can0"},
            {"loop_rate": 100}, # keep as int
            {"rpm_limit": 2000.0}, # must be float!
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
    # publishes the driving flag continuously (default 1Hz) so it doesn't matter if this starts
    # before or after the node's subscription comes up (topic QoS is volatile, not transient local)
    driving_flag_pub = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub",
            "/hydrakon_can/driving_flag",
            "std_msgs/msg/Bool",
            "{data: true}",
        ],
        output="screen",
    )
    # small delay so the node is up and subscribed before we start publishing
    delayed_driving_flag_pub = TimerAction(period=2.0, actions=[driving_flag_pub])

    # when you encounter a make error, go to src/hydrakon_can/include/hydrakon_can/FS-AI_API/FS-AI_API and then run 'make clean' then 'make'
    # if more errors show up, try rm -rf build/ install/ log/ and then run colcon build again, always do colcon build --symlink-install as
    # it's faster
    ld = LaunchDescription()
    ld.add_action(node)
    ld.add_action(delayed_driving_flag_pub)
    return ld
