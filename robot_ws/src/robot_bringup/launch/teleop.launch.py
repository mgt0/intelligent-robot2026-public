"""시험 전용. bringup만 띄우고 teleop_twist_keyboard는 별도 터미널에서 실행한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_fake_hw': LaunchConfiguration('use_fake_hw'),
                              'debug': LaunchConfiguration('debug')}.items()),
        LogInfo(msg='시험 전용. 다른 터미널에서: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
    ])
