import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    rviz2_config = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'rviz2',
        'amzminiVis.rviz'
    )

    rviz2_node = Node(package='rviz2', executable='rviz2', name='rviz2_node', arguments=[
                      '-d', rviz2_config])

    # args that can be set from the command line or a default will be used
    source_launch_arg = DeclareLaunchArgument(
        'source',
        description='source of visualization, can be sim or real',
        default_value='sim'
    )

    track_yaml = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'track',
        'spline', 'la_track_spline.yaml'
    )
    raceline_yaml = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'track',
        'spline', 'la_track_spline.yaml'
     #   'spline', 'la_raceline_spline.yaml'
    )

    visualization_node = Node(
        package='visualization_driving_ros2',
        executable='amzmini_visualizer_node2',
        name='visualization_node',
        parameters=[
            {'track_yaml': track_yaml},
            {'raceline_yaml': raceline_yaml},
            {'source': LaunchConfiguration('source')}
        ],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(rviz2_node)
    ld.add_action(source_launch_arg)
    ld.add_action(visualization_node)

    return ld
