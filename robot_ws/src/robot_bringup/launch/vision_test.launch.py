"""인식 튜닝: camera_ros + 비전 노드 전체(debug 기본 on). rosbag 재생 시에는 카메라 노드를 빼고 쓴다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_bringup'), 'config')
    vis = os.path.join(cfg, 'vision.yaml')
    dbg = {'debug': LaunchConfiguration('debug')}
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),  # 모든 launch 공통 인자 (여기서는 미사용)
        DeclareLaunchArgument('debug', default_value='true'),
        Node(package='camera_ros', executable='camera_node', name='camera', output='screen',
             parameters=[os.path.join(cfg, 'camera.yaml')],
             remappings=[('/camera/image_raw', '/image_raw')]),  # 확인 필요
        Node(package='robot_vision', executable='line_node', name='line_node', output='screen',
             parameters=[vis, dbg]),
        Node(package='robot_vision', executable='sign_node', name='sign_node', output='screen',
             parameters=[vis, {'signs_file': os.path.join(cfg, 'signs.yaml')}, dbg]),
        Node(package='robot_vision', executable='hand_node', name='hand_node', output='screen',
             parameters=[vis, dbg]),
    ])
