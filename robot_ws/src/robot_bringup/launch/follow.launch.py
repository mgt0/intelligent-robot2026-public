"""종목 2 대회용: bringup + hand_node + sign_node + follow_node. teleop 등 수동 입력 경로는 넣지 않는다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    cfg = os.path.join(share, 'config')
    dbg = {'debug': LaunchConfiguration('debug')}
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_fake_hw': LaunchConfiguration('use_fake_hw'),
                              'debug': LaunchConfiguration('debug')}.items()),
        Node(package='robot_vision', executable='hand_node', name='hand_node', output='screen',
             parameters=[os.path.join(cfg, 'vision.yaml'), dbg]),
        Node(package='robot_vision', executable='sign_node', name='sign_node', output='screen',
             parameters=[os.path.join(cfg, 'vision.yaml'),
                         {'signs_file': os.path.join(cfg, 'signs.yaml')}, dbg]),
        Node(package='robot_mission', executable='follow_node', name='follow_node', output='screen',
             parameters=[os.path.join(cfg, 'mission.yaml')]),
    ])
