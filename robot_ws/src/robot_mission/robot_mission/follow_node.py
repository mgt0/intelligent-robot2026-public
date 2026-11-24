"""종목 2: 입력을 모아 주기마다 FollowFSM.step()을 호출하고 /cmd_vel을 낸다."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_mission.drive_fsm import scan_sectors
from robot_mission.follow_fsm import DEFAULTS, FollowFSM


class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        d = self.declare_parameter
        d('front_half_angle', 0.35)
        d('scan_yaw_offset', 0.0)
        d('rate', 20.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        clock = lambda: self.get_clock().now().nanoseconds * 1e-9  # noqa: E731
        self.fsm = FollowFSM(clock, **{k: g(k) for k in DEFAULTS})

        self.hand, self.hand_t, self.sign = None, None, None
        self.front, self.yaw = None, 0.0
        self.last_state = None

        self.create_subscription(PointStamped, '/hand', self.on_hand, 10)
        self.create_subscription(String, '/sign', lambda m: setattr(self, 'sign', m.data), 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.create_service(Trigger, '/mission/start', self.on_start)
        self.create_service(Trigger, '/mission/stop', self.on_stop)
        self.create_timer(1.0 / g('rate'), self.on_timer)

    def on_hand(self, m):
        self.hand = (m.point.x, m.point.y, m.point.z)
        self.hand_t = self.get_clock().now()

    def on_start(self, req, res):
        self.fsm.start()
        res.success, res.message = True, self.fsm.state
        return res

    def on_stop(self, req, res):
        self.fsm.stop()
        res.success, res.message = True, self.fsm.state
        return res

    def on_scan(self, m):
        self.front = scan_sectors(m.ranges, m.angle_min, m.angle_increment, m.range_min,
                                  m.range_max, self.g('front_half_angle'), self.g('scan_yaw_offset'))[0]

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = 2.0 * math.atan2(q.z, q.w)

    def on_timer(self):
        # /hand는 손이 없을 때 발행되지 않으므로, 한 주기 이상 지난 값은 '없음'으로 본다
        age = None if self.hand_t is None else (self.get_clock().now() - self.hand_t).nanoseconds * 1e-9
        hand = self.hand if age is not None and age < 2.0 / self.g('rate') + 0.1 else None
        sign, self.sign = self.sign, None
        v, w, state = self.fsm.step(hand, sign, self.front, self.yaw)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)
        if state != self.last_state:
            self.last_state = state
            self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
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
