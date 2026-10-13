import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    model_params = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'model',
        'amz_mini_car5.yaml'
    )

    simulation_node = Node(
        package='simulation_driving_ros2',
        executable='driving_simulator_node2',
        name='simulation_node',
        parameters=[
                {'model_params': model_params}
        ],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(simulation_node)

    return ld
