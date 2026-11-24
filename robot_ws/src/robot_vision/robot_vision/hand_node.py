"""/image_raw -> /hand, /hand/backend. 계산은 hand_detector.py."""
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_vision.hand_detector import HandDetector


class HandNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        d = self.declare_parameter
        d('backend', 'mediapipe')
        d('process_width', 320)
        d('min_detection_confidence', 0.5)
        d('skin_ycrcb_lower', [0, 133, 77])
        d('skin_ycrcb_upper', [255, 173, 127])
        d('skin_min_area_ratio', 0.01)
        d('skin_roi', [0.3, 1.0])
        d('debug', False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.det = HandDetector(g('backend'), g('process_width'), g('min_detection_confidence'),
                                g('skin_ycrcb_lower'), g('skin_ycrcb_upper'),
                                g('skin_min_area_ratio'), g('skin_roi'))
        if self.det.fallback_reason:
            self.get_logger().warning(f'mediapipe 사용 불가, skin으로 전환: {self.det.fallback_reason}')
        self.bridge = CvBridge()
        # 실제 사용 중인 백엔드를 늦게 붙는 구독자(GUI)도 받도록 transient_local로 한 번 발행
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.backend_pub = self.create_publisher(String, '/hand/backend', qos)
        self.backend_pub.publish(String(data=self.det.backend))
        self.pub = self.create_publisher(PointStamped, '/hand', 10)
        self.dbg_pub = self.create_publisher(Image, '/vision/hand_debug', 1) if g('debug') else None
        self.create_subscription(Image, '/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        det = self.det.detect(bgr)
        if det:
            out = PointStamped()
            out.header = msg.header
            out.point.x, out.point.y, out.point.z = det[0], det[1], det[2]
            self.pub.publish(out)
        if self.dbg_pub:
            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(self.det.draw_debug(bgr, det), 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
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
