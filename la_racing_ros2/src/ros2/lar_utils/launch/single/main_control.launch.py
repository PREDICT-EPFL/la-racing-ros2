import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        emulate_tty=True,
        output='both'
    )

    track_yaml = os.path.join(
        get_package_share_directory('lar_utils'),
        'config',
        'track',
        'la_track_sampled.yaml'
    )

    # args that can be set from the command line or a default will be used
    source_target_launch_arg = DeclareLaunchArgument(
        'source_target',
        description='source of visualization & target of control, can be sim or real',
        default_value='sim'
    )

    arm_mpc_launch_arg = DeclareLaunchArgument(
        'arm_mpc',
        description='whether the MPC is armed, can be True or False',
        default_value='False'
    )

    main_control_node = Node(
        package='main_control_racing_ros2',
        executable='amzmini_control_node2',
        name='main_control_node',
        parameters=[
                {'track_yaml': track_yaml},
                {'control_mode': 'pursuit'},
                {'state_source': LaunchConfiguration('source_target')},
                {'control_target': LaunchConfiguration('source_target')},
                {'arm_mpc': LaunchConfiguration('arm_mpc')}
        ],
        emulate_tty=True,
        output='both'
    )

    ld.add_action(joy_node)
    ld.add_action(source_target_launch_arg)
    ld.add_action(arm_mpc_launch_arg)
    ld.add_action(main_control_node)

    return ld
