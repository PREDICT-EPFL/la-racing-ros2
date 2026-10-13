import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    mocap_config = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'mocap',
        'mocap.yaml'
    )

    mocap_node = Node(
        package='mocap_optitrack',
        executable='mocap_node',
        name='mocap_node',
        parameters=[mocap_config],
        emulate_tty=True,
        output='screen'
    )

    ld.add_action(mocap_node)

    return ld
