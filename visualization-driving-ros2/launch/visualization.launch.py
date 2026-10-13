import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz2_config = os.path.join(get_package_share_directory('visualization_driving_ros2'), 'amzminiVis.rviz')

    rviz2_node = Node(package='rviz2',
                      executable='rviz2',
                      name='rviz2_node', arguments=['-d', rviz2_config])

    track_yaml_path = os.path.join(
        os.path.expanduser('~'), 'driving_tracks', 'la_track_sampled.yaml'
    )

    visualization_node = Node(
        package='visualization_driving_ros2',
        executable='amzmini_visualizer_node2',
        name='visualization_node',
        parameters=[
            {'track_yaml': track_yaml_path},
            {'source': 'sim'}
        ],
        output='both'
    )

    return LaunchDescription([
        rviz2_node,
        visualization_node
    ])
