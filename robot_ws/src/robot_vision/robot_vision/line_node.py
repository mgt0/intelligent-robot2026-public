"""/image_raw -> /line/error, /line/found, /line/detail. 계산은 line_detector.py."""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from robot_vision.line_detector import LineDetector


class LineNode(Node):
    def __init__(self):
        super().__init__('line_node')
        d = self.declare_parameter
        d('process_width', 320)
        d('hsv_lower', [18, 80, 80])
        d('hsv_upper', [35, 255, 255])
        d('roi_near', [0.80, 1.00])
        d('roi_far', [0.60, 0.80])
        d('weight_near', 0.6)
        d('weight_far', 0.4)
        d('min_area_px', 40)
        d('line_mode', 'lane3')
        d('lane3_pair', 'outer')
        d('lane3_gap_px', 80)
        d('lane3_gap_far_px', 60)
        d('lane3_forget_frames', 10)
        d('merge_gap_px', 15)
        d('lane_half_width_px', 100)
        d('max_jump', 0.5)
        d('debug', False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.debug = g('debug')
        self.det = LineDetector(
            process_width=g('process_width'), hsv_lower=g('hsv_lower'), hsv_upper=g('hsv_upper'),
            roi_near=g('roi_near'), roi_far=g('roi_far'), weight_near=g('weight_near'),
            weight_far=g('weight_far'), min_area_px=g('min_area_px'), line_mode=g('line_mode'),
            lane_half_width_px=g('lane_half_width_px'), max_jump=g('max_jump'),
            lane3_pair=g('lane3_pair'), lane3_gap_px=g('lane3_gap_px'),
            lane3_gap_far_px=g('lane3_gap_far_px'), lane3_forget_frames=g('lane3_forget_frames'), merge_gap_px=g('merge_gap_px'))
        self.bridge = CvBridge()
        self.err_pub = self.create_publisher(Float32, '/line/error', 10)
        self.found_pub = self.create_publisher(Bool, '/line/found', 10)
        self.detail_pub = self.create_publisher(String, '/line/detail', 10)
        self.dbg_pub = self.create_publisher(Image, '/vision/line_debug', 1) if self.debug else None
        self.create_subscription(Image, '/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.det.detect(bgr)
        self.found_pub.publish(Bool(data=res.found))
        if res.found:
            self.err_pub.publish(Float32(data=res.error))
        lcr = res.lcr or (None, None, None)
        self.detail_pub.publish(String(data=json.dumps(
            {'mode': self.det.mode, 'n': res.n, 'L': lcr[0], 'C': lcr[1], 'R': lcr[2]})))
        if self.dbg_pub:
            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(self.det.draw_debug(bgr, res), 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = LineNode()
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
