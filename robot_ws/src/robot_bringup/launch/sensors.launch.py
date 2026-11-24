"""센서만: rplidar_node, camera_ros. 카메라 토픽은 /image_raw로 리매핑."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        # 패키지 rplidar_ros. apt(ros-jazzy-rplidar-ros) 빌드의 실행 파일은 rplidar_composition
        # (소스 ros2 브랜치의 rplidar_node가 아님). 실물 확인 2026-10-27.
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
             output='screen', parameters=[os.path.join(cfg, 'rplidar.yaml')]),
        # 패키지 camera_ros, 실행 파일 camera_node. 기본 발행 토픽은 ~/image_raw (camera_ros README)
        Node(package='camera_ros', executable='camera_node', name='camera', output='screen',
             parameters=[os.path.join(cfg, 'camera.yaml')],
             remappings=[('/camera/image_raw', '/image_raw')]),  # 확인 필요: 리매핑 실제 적용 여부
    ])
