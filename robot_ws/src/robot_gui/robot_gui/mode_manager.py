"""Pi 쪽 모드 관리 노드. /mode/drive, /mode/follow 서비스로 mission_stack을 하위 프로세스로 전환한다.
/cmd_vel은 발행하지 않는다. 프로세스 처리는 mission_proc.py."""
import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_gui.mission_proc import MissionProc

MODES = ('drive', 'follow')


def has_map(map_file):
    # 확인 필요: slam_toolbox serialize_map이 남기는 확장자(.posegraph, .data) 가정
    return bool(map_file) and os.path.isfile(map_file + '.posegraph')


def map_args(mode, map_file):
    """drive이고 지도 파일이 있을 때만 map:= 인자를 붙인다. 없으면 생략(mapping)."""
    return [f'map:={map_file}'] if mode == 'drive' and has_map(map_file) else []


def slam_state(mode, map_file):
    """/mission/slam 값: follow는 off, drive는 지도 있으면 localization 없으면 mapping."""
    if mode != 'drive':
        return 'off'
    return 'localization' if has_map(map_file) else 'mapping'


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.declare_parameter('default_mode', 'drive')
        self.declare_parameter('ros2_cmd', 'ros2')
        self.declare_parameter('map_file', '')
        grp = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mode_pub = self.create_publisher(String, '/mission/mode', latched)
        self.log_pub = self.create_publisher(String, '/mission/log', 10)
        self.slam_pub = self.create_publisher(String, '/mission/slam', latched)
        # GUI [지도 저장]이 쓰는 경로의 단일 출처
        self.map_pub = self.create_publisher(String, '/mission/map_file', latched)
        self.map_pub.publish(String(data=self.get_parameter('map_file').value))
        self.proc = MissionProc(self.get_parameter('ros2_cmd').value, on_line=self._on_line)
        self.stop_cli = self.create_client(Trigger, '/mission/stop', callback_group=grp)
        self.mode = ''
        self.busy = threading.Lock()
        for m in MODES:
            self.create_service(Trigger, f'/mode/{m}',
                                lambda req, res, m=m: self._request(m, res), callback_group=grp)
        default = self.get_parameter('default_mode').value
        if default in MODES:
            self._request(default, Trigger.Response())
        else:
            self.get_logger().error(f'default_mode는 drive 또는 follow: {default!r}')

    def _on_line(self, line):
        self.log_pub.publish(String(data=line))

    def _log(self, text):
        self.get_logger().info(text)
        self._on_line(f'[mode_manager] {text}')

    def _request(self, mode, res):
        """전환은 최대 7초 걸릴 수 있어 별도 스레드에서 하고, 서비스는 바로 응답한다."""
        if mode == self.mode and self.proc.state()['running']:
            res.success, res.message = True, f'이미 {mode}'
        elif not self.busy.acquire(blocking=False):
            res.success, res.message = False, '전환 중'
        else:
            threading.Thread(target=self._switch, args=(mode,), daemon=True).start()
            res.success, res.message = True, f'{mode} 전환 시작'
        return res

    def _switch(self, mode):
        try:
            if self.mode:
                self._mission_stop()
            self.proc.stop()
            self.mode = ''
            mf = self.get_parameter('map_file').value
            self.proc.start(mode, map_args(mode, mf))
            self.mode = mode
            self.mode_pub.publish(String(data=mode))
            self.slam_pub.publish(String(data=slam_state(mode, mf)))
            self._log(f'{mode} 시작')
        except Exception as e:  # noqa: BLE001 - ros2 명령 없음 등
            self._log(f'{mode} 전환 실패: {e}')
        finally:
            self.busy.release()

    def _mission_stop(self):
        if not self.stop_cli.service_is_ready():
            return
        done = threading.Event()
        fut = self.stop_cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            self.stop_cli.remove_pending_request(fut)

    def shutdown(self):
        self.proc.stop()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
