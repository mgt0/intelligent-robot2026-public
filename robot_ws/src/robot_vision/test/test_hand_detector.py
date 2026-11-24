from types import SimpleNamespace

import cv2
import numpy as np

from robot_vision.hand_detector import HandDetector, palm_center

SKIN = (120, 150, 200)  # BGR -> YCrCb 약 (161, 155, 105), 기본 범위 안


def skin_frame():
    img = np.full((240, 320, 3), 128, np.uint8)
    cv2.rectangle(img, (130, 110), (190, 170), SKIN, -1)  # 손바닥
    cv2.rectangle(img, (150, 170), (170, 239), SKIN, -1)  # 팔 (아래로 이어짐)
    return img


def test_palm_center_is_mean_of_five_landmarks():
    lm = [SimpleNamespace(x=0.0, y=0.0) for _ in range(21)]
    for i, (x, y) in zip((0, 5, 9, 13, 17), ((0.5, 0.9), (0.3, 0.5), (0.5, 0.4), (0.7, 0.5), (0.9, 0.6))):
        lm[i] = SimpleNamespace(x=x, y=y)
    cx, cy = palm_center(lm)
    assert abs(cx - 0.58) < 1e-9 and abs(cy - 0.58) < 1e-9


def test_skin_center_on_palm_not_arm():
    x, y, size, _ = HandDetector(backend='skin').detect(skin_frame())
    assert abs(x) < 0.15 and y < (170 - 120) / 120 and y > -0.2  # 손바닥(y 110~170) 안, 팔 쪽이 아님
    assert 0.2 < size < 0.7


def test_skin_small_blob_ignored():
    img = np.full((240, 320, 3), 128, np.uint8)
    img[150:156, 100:106] = SKIN  # 36px < 영상의 1%
    assert HandDetector(backend='skin').detect(img) is None


def test_skin_none():
    assert HandDetector(backend='skin').detect(np.full((240, 320, 3), 128, np.uint8)) is None


def test_mediapipe_missing_falls_back_to_skin():
    d = HandDetector(backend='mediapipe')
    if d.backend == 'skin':  # 이 PC에는 mediapipe가 없을 수 있다
        assert d.fallback_reason
