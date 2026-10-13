from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    simulate_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single/simulate.launch.py']))

    # note that after constructing simulate_ros_launch, the path has been changed to launch/single already.

    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/visualize.launch.py']),
        launch_arguments={
            'source': 'sim'
        }.items())

    arm_mpc_launch_arg = DeclareLaunchArgument(
        'arm_mpc',
        description='whether the MPC is armed, can be True or False',
        default_value='False'
    )

    main_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/main_control.launch.py']),
        launch_arguments={
            'source_target': 'sim',
            'arm_mpc': LaunchConfiguration('arm_mpc')
        }.items())

    ld.add_action(simulate_ros_launch)
    ld.add_action(visualize_launch)
    ld.add_action(arm_mpc_launch_arg)
    ld.add_action(main_control_launch)

    return ld
