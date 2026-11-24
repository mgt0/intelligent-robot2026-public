"""구동만: base_node + 센서 static TF."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_robot_yaml():
    cfg = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'robot.yaml')
    with open(cfg, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _sensor_tf(context):
    tf = _load_robot_yaml()['sensor_tf']

    def static(child, x, y, z, yaw=0.0, pitch=0.0):
        # 확인 필요: Jazzy tf2_ros static_transform_publisher의 --x/--yaw 플래그 형식
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                               '--yaw', str(yaw), '--pitch', str(pitch), '--roll', '0',
                               '--frame-id', 'base_link', '--child-frame-id', child])

    lz, cm = tf['laser'], tf['camera']
    return [static('laser', lz['x'], lz['y'], lz['z'], yaw=lz['yaw']),
            static('camera_link', cm['x'], cm['y'], cm['z'], pitch=cm['pitch'])]


def generate_launch_description():
    # robot.yaml에는 sensor_tf처럼 ros__parameters가 없는 섹션이 있어 --params-file로 넘기면
    # rcl 파서가 거부한다. base_node 섹션만 dict로 넘긴다.
    base_params = _load_robot_yaml()['base_node']['ros__parameters']
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        Node(package='robot_base', executable='base_node', name='base_node', output='screen',
             parameters=[base_params, {'use_fake_hw': LaunchConfiguration('use_fake_hw')}]),
        OpaqueFunction(function=_sensor_tf),
    ])
