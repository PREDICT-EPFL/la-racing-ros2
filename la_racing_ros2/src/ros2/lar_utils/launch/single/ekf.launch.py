import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ekf_config = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'ekf',
        'ekf_tuning.yaml'
    )

    ekf_node = Node(
        package='lar_ros_estimators',
        executable='rigid_body_estimator',
        name='ekf_node',
        parameters=[
                {'tuning_params': ekf_config}
        ],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(ekf_node)

    return ld
