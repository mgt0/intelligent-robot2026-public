"""종목 1 대회용: bringup + line_node + sign_node + drive_node + slam_toolbox(use_slam). teleop 등 수동 입력 경로는 넣지 않는다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bringup.slam_launch import slam


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    cfg = os.path.join(share, 'config')
    dbg = {'debug': LaunchConfiguration('debug')}
    # yaml 뒤에 두어 launch 인자가 mission.yaml의 use_slam을 덮어쓴다
    USE_SLAM = {'use_slam': ParameterValue(LaunchConfiguration('use_slam'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('debug', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_fake_hw': LaunchConfiguration('use_fake_hw'),
                              'debug': LaunchConfiguration('debug')}.items()),
        Node(package='robot_vision', executable='line_node', name='line_node', output='screen',
             parameters=[os.path.join(cfg, 'vision.yaml'), dbg]),
        Node(package='robot_vision', executable='sign_node', name='sign_node', output='screen',
             parameters=[os.path.join(cfg, 'vision.yaml'),
                         {'signs_file': os.path.join(cfg, 'signs.yaml')}, dbg]),
        Node(package='robot_mission', executable='drive_node', name='drive_node', output='screen',
             parameters=[os.path.join(cfg, 'mission.yaml'), USE_SLAM]),
        OpaqueFunction(function=slam),
    ])
