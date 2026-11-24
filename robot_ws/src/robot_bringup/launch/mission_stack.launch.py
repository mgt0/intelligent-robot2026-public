"""bringup 없이 미션 노드만. mode:=drive|follow. mode_manager가 모드 전환 시 실행한다.
기존 drive/follow.launch.py(대회용)와 같은 노드·파라미터 구성이며, 그 파일은 건드리지 않는다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bringup.slam_launch import slam


def _nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    cfg = os.path.join(get_package_share_directory('robot_bringup'), 'config')
    dbg = {'debug': LaunchConfiguration('debug')}
    sign = Node(package='robot_vision', executable='sign_node', name='sign_node', output='screen',
                parameters=[os.path.join(cfg, 'vision.yaml'),
                            {'signs_file': os.path.join(cfg, 'signs.yaml')}, dbg])
    if mode == 'drive':
        return [
            Node(package='robot_vision', executable='line_node', name='line_node', output='screen',
                 parameters=[os.path.join(cfg, 'vision.yaml'), dbg]),
            sign,
            Node(package='robot_mission', executable='drive_node', name='drive_node',
                 output='screen',
                 parameters=[os.path.join(cfg, 'mission.yaml'),
                             {'use_slam': ParameterValue(LaunchConfiguration('use_slam'), value_type=bool)}]),
            *slam(context),  # follow 모드는 SLAM 없음
        ]
    if mode == 'follow':
        return [
            Node(package='robot_vision', executable='hand_node', name='hand_node', output='screen',
                 parameters=[os.path.join(cfg, 'vision.yaml'), dbg]),
            sign,
            Node(package='robot_mission', executable='follow_node', name='follow_node',
                 output='screen', parameters=[os.path.join(cfg, 'mission.yaml')]),
        ]
    raise RuntimeError(f"mode는 drive 또는 follow: {mode!r}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='drive'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('debug', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
