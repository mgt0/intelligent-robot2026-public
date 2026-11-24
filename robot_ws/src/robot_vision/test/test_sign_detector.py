import cv2
import numpy as np

from robot_vision.sign_detector import SignDetector

SIGNS = {
    'stop': {'hsv_ranges': [[0, 100, 70, 10, 255, 255], [170, 100, 70, 179, 255, 255]],
             'shape': 'octagon', 'min_area_ratio': 0.004},
    'uturn': {'hsv_ranges': [[100, 120, 60, 130, 255, 255]],
              'shape': 'circle', 'min_area_ratio': 0.004},
}
RED, BLUE = (0, 0, 220), (220, 60, 0)  # BGR


def canvas():
    return np.full((240, 320, 3), 128, np.uint8)


def octagon(img, c, r, color):
    pts = [(c[0] + r * np.cos(np.pi / 8 + i * np.pi / 4), c[1] + r * np.sin(np.pi / 8 + i * np.pi / 4))
           for i in range(8)]
    cv2.fillPoly(img, [np.array(pts, np.int32)], color)


def test_red_octagon_is_stop():
    img = canvas()
    octagon(img, (160, 120), 40, RED)
    det = SignDetector(SIGNS).detect(img)
    assert det and det['cls'] == 'stop'


def test_blue_circle_is_uturn():
    img = canvas()
    cv2.circle(img, (160, 120), 40, BLUE, -1)
    det = SignDetector(SIGNS).detect(img)
    assert det and det['cls'] == 'uturn'


def test_small_shape_ignored():
    img = canvas()
    octagon(img, (160, 120), 8, RED)  # 면적비 0.004 미만
    assert SignDetector(SIGNS).detect(img) is None


def test_wrong_shape_rejected():
    img = canvas()
    cv2.rectangle(img, (100, 80), (200, 160), RED, -1)  # 빨간 사각형은 stop 아님
    assert SignDetector(SIGNS).detect(img) is None


def test_debounce_confirm_once_and_release():
    d = SignDetector(SIGNS, confirm_frames=3, release_frames=5)
    out = [d.debounce('stop') for _ in range(6)]
    assert out == [None, None, 'stop', None, None, None]  # 확정은 1회만
    for _ in range(5):
        d.debounce(None)  # release_frames만큼 미검출 -> 해제
    out = [d.debounce('stop') for _ in range(3)]
    assert out == [None, None, 'stop']


def test_debounce_interrupted_not_confirmed():
    d = SignDetector(SIGNS, confirm_frames=3)
    assert [d.debounce(c) for c in ['stop', 'stop', None, 'stop', 'stop']] == [None] * 5
