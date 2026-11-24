"""Pi에서 GUI로 운용·시험: bringup + mode_manager. 미션 스택은 mode_manager가 하위 프로세스로 관리한다.
GUI 창은 별도로 `ros2 run robot_gui gui`."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('default_mode', default_value='drive'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_fake_hw': LaunchConfiguration('use_fake_hw'),
                              'debug': LaunchConfiguration('debug')}.items()),
        Node(package='robot_gui', executable='mode_manager', name='mode_manager', output='screen',
             parameters=[os.path.join(share, 'config', 'gui.yaml'),
                         {'default_mode': ParameterValue(LaunchConfiguration('default_mode'),
                                                         value_type=str)}]),
    ])
