"""지도 작성: bringup + slam_toolbox (async, mapping)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    # 패키지 slam_toolbox, 실행 파일 async_slam_toolbox_node, LifecycleNode
    # (slam_toolbox jazzy 브랜치 online_async_launch.py 기준)
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[os.path.join(share, 'config', 'slam.yaml'),
                                     {'use_lifecycle_manager': False}])
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hw', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_fake_hw': LaunchConfiguration('use_fake_hw'),
                              'debug': LaunchConfiguration('debug')}.items()),
        slam, activate, configure,
    ])
