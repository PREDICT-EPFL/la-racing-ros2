import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    track_yaml_path = os.path.join(
        os.path.expanduser('~'), 'driving_tracks', 'la_track_sampled.yaml'
    )

    main_control_node = Node(
        package='main_control_racing_ros2',
        executable='amzmini_control_node2',
        name='main_control',
        parameters=[
            {'track_yaml': track_yaml_path},
            {'state_source': 'real'},
            {'control_target': 'real'},
            {'control_mode': 'pursuit'},
            {'activate': False},
            {'arm_mpc': False},
            {'joy_mode': 'joystick'},
            {'joy_mode_exp': 0.75},
            {'joy_mode_tan': 3.0}
        ],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        main_control_node,
        joy_node
    ])
