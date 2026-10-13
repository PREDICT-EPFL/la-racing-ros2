from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["udp4", "-p", "8888"],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(micro_ros_agent_node)

    return ld
