"""/image_raw -> /sign, /sign/detail. 계산은 sign_detector.py. 클래스 정의는 signs.yaml."""
import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_vision.sign_detector import SignDetector


class SignNode(Node):
    def __init__(self):
        super().__init__('sign_node')
        self.declare_parameter('signs_file', '')  # 비우면 robot_bringup/config/signs.yaml
        self.declare_parameter('debug', False)
        path = self.get_parameter('signs_file').value or os.path.join(
            get_package_share_directory('robot_bringup'), 'config', 'signs.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.det = SignDetector(
            cfg['signs'], process_width=cfg.get('process_width', 320),
            confirm_frames=cfg.get('confirm_frames', 3), release_frames=cfg.get('release_frames', 5),
            template_threshold=cfg.get('template_threshold', 0.5),
            template_dir=os.path.join(get_package_share_directory('robot_vision'), 'templates'))
        self.bridge = CvBridge()
        self.sign_pub = self.create_publisher(String, '/sign', 10)
        self.detail_pub = self.create_publisher(String, '/sign/detail', 10)
        self.dbg_pub = (self.create_publisher(Image, '/vision/sign_debug', 1)
                        if self.get_parameter('debug').value else None)
        self.create_subscription(Image, '/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        det, confirmed = self.det.process(bgr)
        if det:
            self.detail_pub.publish(String(data=json.dumps(
                {'cls': det['cls'], 'area': det['area'], 'cx': det['cx']})))
        if confirmed:
            self.sign_pub.publish(String(data=confirmed))
        if self.dbg_pub:
            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(self.det.draw_debug(bgr, det), 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = SignNode()
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
