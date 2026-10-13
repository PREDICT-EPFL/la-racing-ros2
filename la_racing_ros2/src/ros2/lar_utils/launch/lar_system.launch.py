from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    ld = LaunchDescription()

    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/single/mocap.launch.py']))

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ekf.launch.py']))

    micro_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/micro_ros.launch.py']))
    # note that after constructing micro_ros_launch, the path has been changed to launch/single already.

    ld.add_action(mocap_launch)
    ld.add_action(ekf_launch)
    ld.add_action(micro_ros_launch)

    return ld
