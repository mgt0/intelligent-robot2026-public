"""종목 1: 입력을 모아 주기마다 DriveFSM.step()을 호출하고 /cmd_vel을 낸다."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_mission.drive_fsm import DEFAULTS, DriveFSM, scan_sectors


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        d = self.declare_parameter
        d('front_half_angle', 0.35)
        d('scan_yaw_offset', 0.0)
        d('auto_start', False)
        d('auto_start_delay', 1.0)
        d('rate', 20.0)
        d('use_slam', True)
        d('slam_tf_timeout', 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        clock = lambda: self.get_clock().now().nanoseconds * 1e-9  # noqa: E731
        self.fsm = DriveFSM(clock, **{k: g(k) for k in DEFAULTS})

        self.found, self.err, self.sign = False, 0.0, None
        self.sectors = (None, None, None)
        self.pose = (0.0, 0.0, 0.0)
        self.last_state = None
        self.last_lap = None
        if g('use_slam'):
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)

        self.create_subscription(Float32, '/line/error', lambda m: setattr(self, 'err', m.data), 10)
        self.found_t = 0.0
        self.create_subscription(Bool, '/line/found', self.on_found, 10)
        self.create_subscription(String, '/sign', lambda m: setattr(self, 'sign', m.data), 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.lap_pub = self.create_publisher(
            Int32, '/mission/lap', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.slam_ok_pub = self.create_publisher(Bool, '/mission/slam_ok', 10) if g('use_slam') else None
        self.create_service(Trigger, '/mission/start', self.on_start)
        self.create_service(Trigger, '/mission/stop', self.on_stop)
        self.create_timer(1.0 / g('rate'), self.on_timer)
        if g('auto_start'):
            self.auto = self.create_timer(g('auto_start_delay'), self.on_auto)

    def on_auto(self):
        self.auto.cancel()
        self.fsm.start()

    def on_start(self, req, res):
        self.fsm.start()
        res.success, res.message = True, self.fsm.state
        return res

    def on_stop(self, req, res):
        self.fsm.stop()
        res.success, res.message = True, self.fsm.state
        return res

    def on_found(self, m):
        self.found, self.found_t = m.data, self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, m):
        self.sectors = scan_sectors(m.ranges, m.angle_min, m.angle_increment, m.range_min,
                                    m.range_max, self.g('front_half_angle'), self.g('scan_yaw_offset'))

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     2.0 * math.atan2(q.z, q.w))

    def map_xy(self):
        """TF map->base_link. 실패·타임아웃이면 None(바퀴 수 판정만 멈춤)."""
        if not self.g('use_slam'):
            return None
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        age = self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)
        if age > Duration(seconds=self.g('slam_tf_timeout')):
            return None
        return t.transform.translation.x, t.transform.translation.y

    def on_timer(self):
        sign, self.sign = self.sign, None  # 확정 이벤트는 한 번만 소비
        # 카메라·line_node가 멈추면 0.5초 뒤 라인 손실로 처리(마지막 값으로 계속 달리지 않게)
        fresh = self.get_clock().now().nanoseconds * 1e-9 - self.found_t < 0.5
        xy = self.map_xy()
        if self.slam_ok_pub:
            self.slam_ok_pub.publish(Bool(data=xy is not None))
        v, w, state = self.fsm.step(self.found and fresh, self.err, sign, *self.sectors, self.pose,
                                       map_xy=xy)
        if self.fsm.laps != self.last_lap:
            self.last_lap = self.fsm.laps
            self.lap_pub.publish(Int32(data=self.last_lap))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)
        if state != self.last_state:
            self.last_state = state
            self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
