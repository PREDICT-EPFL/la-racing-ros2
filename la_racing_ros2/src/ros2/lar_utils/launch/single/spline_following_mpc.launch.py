import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    track_yaml = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'track',
        'la_track_sampled.yaml'
    )

    spline_following_yaml = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'mpc',
        'spline_following_params.yaml'
    )

    # args that can be set from the command line or a default will be used
    state_source_launch_arg = DeclareLaunchArgument(
        'state_source',
        description='source of state x0, can be sim or real',
        default_value='sim'
    )

    spline_following_mpc_node = Node(
        package='mpc_control_ros2',
        executable='spline_following_mpc_MGMI_node2',
        name='spline_following_mpc_node',
        parameters=[
                {'state_source': LaunchConfiguration('state_source')},
                {'track_yaml': track_yaml},
                {'params_yaml': spline_following_yaml}
        ],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(state_source_launch_arg)
    ld.add_action(spline_following_mpc_node)

    return ld
