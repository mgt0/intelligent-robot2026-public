"""/cmd_vel -> XL430 속도 명령, 엔코더 -> /odom, TF. 계산은 kinematics.py."""
import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from robot_base import kinematics as kin

# 2.3절 제어 테이블 (Protocol 2.0, X 시리즈)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE = 64
ADDR_GOAL_VEL = 104
ADDR_PRESENT_VEL = 128  # 128(속도 4B) + 132(위치 4B)를 연속 8바이트로 읽는다
ADDR_PRESENT_POS = 132


class DxlHw:
    def __init__(self, port, baud, ids, log):
        from dynamixel_sdk import PacketHandler, PortHandler  # ROS 없는 PC에서 import 실패 방지
        self.ids, self.log = ids, log
        self.port = PortHandler(port)
        self.ph = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud)):
            raise RuntimeError(f'포트 열기 실패: {port} @ {baud}')
        for i in ids:
            _, res, _ = self.ph.ping(self.port, i)
            if res != 0:
                raise RuntimeError(f'XL430 ID {i} ping 실패')
            mode, _, _ = self.ph.read1ByteTxRx(self.port, i, ADDR_OPERATING_MODE)
            if mode != 1:  # 속도 제어가 아니면 토크를 끄고 1로 쓴다
                self.ph.write1ByteTxRx(self.port, i, ADDR_TORQUE, 0)
                self.ph.write1ByteTxRx(self.port, i, ADDR_OPERATING_MODE, 1)
                log.warning(f'ID {i} Operating Mode {mode} -> 1(속도)로 변경')
        self.torque(True)

    def torque(self, on):
        for i in self.ids:
            self.ph.write1ByteTxRx(self.port, i, ADDR_TORQUE, int(on))

    def write_velocity(self, raws):
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite
        gw = GroupSyncWrite(self.port, self.ph, ADDR_GOAL_VEL, 4)
        for i, r in zip(self.ids, raws):
            gw.addParam(i, list(struct.pack('<i', r)))
        ok = gw.txPacket() == COMM_SUCCESS
        gw.clearParam()
        return ok

    def read(self):
        """((속도raw, ...), (위치tick, ...)) 또는 실패 시 None."""
        from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead
        gr = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_VEL, 8)
        for i in self.ids:
            gr.addParam(i)
        if gr.txRxPacket() != COMM_SUCCESS or not all(
                gr.isAvailable(i, ADDR_PRESENT_VEL, 8) for i in self.ids):
            return None
        s32 = lambda u: u - (1 << 32) if u >= (1 << 31) else u  # noqa: E731
        vel = [s32(gr.getData(i, ADDR_PRESENT_VEL, 4)) for i in self.ids]
        pos = [s32(gr.getData(i, ADDR_PRESENT_POS, 4)) for i in self.ids]
        return vel, pos

    def close(self):
        try:
            self.write_velocity([0] * len(self.ids))
            self.torque(False)
        finally:
            self.port.closePort()


class FakeHw:
    """명령 속도를 그대로 적분하는 가짜 하드웨어."""

    def __init__(self, n=2):
        self.vel = [0] * n
        self.pos = [0.0] * n
        self.on = True

    def torque(self, on):
        self.on = on

    def write_velocity(self, raws):
        self.vel = list(raws) if self.on else [0] * len(raws)
        return True

    def step(self, dt):
        # raw 1단위 = 0.229rpm, 1회전 = 4096tick
        self.pos = [p + v * kin.RAW_UNIT_RPM / 60.0 * kin.TICKS_PER_REV * dt
                    for p, v in zip(self.pos, self.vel)]

    def read(self):
        return list(self.vel), [int(round(p)) for p in self.pos]

    def close(self):
        self.vel = [0, 0]


class BaseNode(Node):
    def __init__(self):
        super().__init__('base_node')
        d = self.declare_parameter
        d('port', '/dev/openrb')
        d('baudrate', 1000000)
        d('left_id', 1)
        d('right_id', 2)
        d('left_sign', 1)
        d('right_sign', -1)
        d('wheel_radius', 0.033)
        d('wheel_base', 0.160)
        d('max_linear', 0.30)
        d('max_angular', 2.0)
        d('cmd_timeout', 0.5)
        d('rate', 30.0)
        d('publish_tf', True)
        d('velocity_limit_raw', 265)
        d('use_fake_hw', False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.ls, self.rs = g('left_sign'), g('right_sign')

        if g('use_fake_hw'):
            self.hw = FakeHw()
            self.get_logger().info('가짜 하드웨어 사용')
        else:
            self.hw = DxlHw(g('port'), g('baudrate'), [g('left_id'), g('right_id')], self.get_logger())
        self.odom = kin.Odometry(g('wheel_radius'), g('wheel_base'), self.ls, self.rs)
        self.cmd = (0.0, 0.0)
        self.cmd_time = None
        self.prev_pos = None
        self.prev_t = None
        self.fails = 0

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.vel_pub = self.create_publisher(Float32MultiArray, '/wheel/velocity', 10)
        self.tf = TransformBroadcaster(self)
        self.create_service(SetBool, '/base/torque', self.on_torque)
        self.timer = self.create_timer(1.0 / g('rate'), self.on_timer)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def on_torque(self, req, res):
        self.hw.torque(req.data)
        res.success = True
        return res

    def on_timer(self):
        g, now = self.g, self.get_clock().now()
        stale = self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > g('cmd_timeout')
        v, w = (0.0, 0.0) if stale else self.cmd
        raws = kin.cmd_to_raw(v, w, g('wheel_base'), g('wheel_radius'), g('max_linear'),
                              g('max_angular'), g('velocity_limit_raw'), self.ls, self.rs)
        ok = self.hw.write_velocity(raws)
        if isinstance(self.hw, FakeHw):
            self.hw.step(1.0 / g('rate'))
        data = self.hw.read() if ok else None
        if data is None:
            self.fails += 1
            self.get_logger().warning('모터 통신 실패', throttle_duration_sec=1.0)
            if self.fails >= g('rate'):
                self.get_logger().error('모터 통신 연속 실패', throttle_duration_sec=5.0)
            return
        self.fails = 0
        vel, pos = data
        t = now.nanoseconds * 1e-9
        wv = Float32MultiArray()
        wv.data = [kin.raw_to_rad_s(vel[0]) * self.ls, kin.raw_to_rad_s(vel[1]) * self.rs]
        self.vel_pub.publish(wv)
        if self.prev_pos is None:
            self.prev_pos, self.prev_t = pos, t
            return
        dl = kin.tick_delta(pos[0], self.prev_pos[0])
        dr = kin.tick_delta(pos[1], self.prev_pos[1])
        lin, ang = self.odom.update(dl, dr, t - self.prev_t)
        self.prev_pos, self.prev_t = pos, t
        self.publish_odom(now.to_msg(), lin, ang)

    def publish_odom(self, stamp, lin, ang):
        o = self.odom
        qz, qw = math.sin(o.th / 2), math.cos(o.th / 2)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = o.x, o.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = lin, ang
        self.odom_pub.publish(msg)
        if self.g('publish_tf'):
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = o.x, o.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf.sendTransform(tf)

    def destroy_node(self):
        self.hw.close()  # 속도 0 쓰기 + 토크 끄기
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
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
