"""공통 하드웨어: base + sensors."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    args = {'use_fake_hw': LaunchConfiguration('use_fake_hw'),
            'debug': LaunchConfiguration('debug')}.items()

    def inc(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)), launch_arguments=args)

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        inc('base.launch.py'),
        inc('sensors.launch.py'),
    ])
