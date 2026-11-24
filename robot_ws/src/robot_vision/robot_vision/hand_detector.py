"""손 검출. mediapipe 우선, 실패하면 피부색(skin)으로 전환. ROS 비의존."""
import math

import cv2
import numpy as np

PALM_IDX = (0, 5, 9, 13, 17)  # 손목, 검지·중지·약지·소지 MCP


def palm_center(lm):
    """랜드마크(.x, .y 정규화) 5점 평균 = 손바닥 중심."""
    return (sum(lm[i].x for i in PALM_IDX) / 5, sum(lm[i].y for i in PALM_IDX) / 5)


class HandDetector:
    def __init__(self, backend='mediapipe', process_width=320, min_detection_confidence=0.5,
                 skin_ycrcb_lower=(0, 133, 77), skin_ycrcb_upper=(255, 173, 127),
                 skin_min_area_ratio=0.01, skin_roi=(0.3, 1.0)):
        self.process_width = process_width
        self.lower = np.array(skin_ycrcb_lower, np.uint8)
        self.upper = np.array(skin_ycrcb_upper, np.uint8)
        self.min_area_ratio = skin_min_area_ratio
        self.skin_roi = tuple(skin_roi)
        self.backend = backend
        self.fallback_reason = None
        self._hands = None
        if backend == 'mediapipe':
            try:
                import mediapipe as mp  # Pi에서 설치 실패할 수 있어 지연 import
                # 확인 필요: Pi의 MediaPipe 버전에서 mp.solutions가 없으면(Tasks API만) 아래가 실패해 skin으로 전환된다
                self._hands = mp.solutions.hands.Hands(
                    max_num_hands=1, model_complexity=0,
                    min_detection_confidence=min_detection_confidence)
            except Exception as e:  # 어떤 실패든 skin으로 전환
                self.backend = 'skin'
                self.fallback_reason = repr(e)

    def detect(self, bgr):
        """(x, y, size, box) 또는 None. x,y는 -1~1, size는 박스 대각선/영상 대각선."""
        h0, w0 = bgr.shape[:2]
        if w0 != self.process_width:
            bgr = cv2.resize(bgr, (self.process_width, int(h0 * self.process_width / w0)))
        H, W = bgr.shape[:2]
        if self.backend == 'mediapipe':
            res = self._hands.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            if not res.multi_hand_landmarks:
                return None
            lm = res.multi_hand_landmarks[0].landmark
            px, py = palm_center(lm)
            cx, cy = px * W, py * H
            xs, ys = [p.x for p in lm], [p.y for p in lm]
            x0, y0, x1, y1 = min(xs) * W, min(ys) * H, max(xs) * W, max(ys) * H
        else:
            mask = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb), self.lower, self.upper)
            k = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
            mask[:int(H * self.skin_roi[0])] = 0  # 얼굴·천장 제외
            mask[int(H * self.skin_roi[1]):] = 0
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < self.min_area_ratio * W * H:
                return None
            blob = np.zeros_like(mask)
            cv2.drawContours(blob, [c], -1, 255, -1)
            # 팔까지 이어진 덩어리라도 가장 두꺼운 곳(손바닥)이 거리 변환 최댓값이다
            _, _, _, (cx, cy) = cv2.minMaxLoc(cv2.distanceTransform(blob, cv2.DIST_L2, 5))
            bx, by, bw, bh = cv2.boundingRect(c)
            x0, y0, x1, y1 = bx, by, bx + bw, by + bh
        size = math.hypot(x1 - x0, y1 - y0) / math.hypot(W, H)
        return ((cx - W / 2) / (W / 2), (cy - H / 2) / (H / 2), size,
                (int(x0), int(y0), int(x1), int(y1)))

    def draw_debug(self, bgr, det):
        img = cv2.resize(bgr, (self.process_width, int(bgr.shape[0] * self.process_width / bgr.shape[1])))
        if det:
            x0, y0, x1, y1 = det[3]
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        return img
