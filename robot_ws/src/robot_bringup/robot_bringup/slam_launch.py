"""slam_toolbox 실행 구성. drive.launch.py와 mission_stack.launch.py가 공유한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def slam(context):
    """use_slam이 true면 slam_toolbox. map이 비면 mapping, 있으면 localization."""
    if LaunchConfiguration('use_slam').perform(context).lower() != 'true':
        return []
    map_file = LaunchConfiguration('map').perform(context)
    yaml = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'slam.yaml')
    # 확인 필요: 실행 파일명은 slam_toolbox jazzy 기준 async_slam_toolbox_node / localization_slam_toolbox_node
    params = [yaml, {'use_lifecycle_manager': False}]
    exe = 'async_slam_toolbox_node'
    if map_file:
        exe = 'localization_slam_toolbox_node'
        params.append({'mode': 'localization', 'map_file_name': map_file})
    slam = LifecycleNode(package='slam_toolbox', executable=exe, name='slam_toolbox',
                         namespace='', output='screen', parameters=params)
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [slam, activate, configure]
