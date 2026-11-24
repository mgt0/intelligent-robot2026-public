"""색·모양 기반 표지판 검출과 디바운스. ROS 비의존."""
import math
import os

import cv2
import numpy as np


class SignDetector:
    def __init__(self, signs, process_width=320, confirm_frames=3, release_frames=5,
                 template_threshold=0.5, template_dir=None):
        self.signs = signs
        self.process_width = process_width
        self.confirm_frames = confirm_frames
        self.release_frames = release_frames
        self.template_threshold = template_threshold
        self.templates = {}
        for name, cfg in signs.items():
            f = cfg.get('template')
            path = os.path.join(template_dir, f) if f and template_dir else None
            if path and os.path.isfile(path):  # 파일이 없으면 2차 확인 없이 색·모양만 사용
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                self.templates[name] = cv2.resize(img, (64, 64))
        self._cand, self._count, self._active, self._miss = None, 0, None, 0

    @staticmethod
    def _shape_ok(shape, cnt, area):
        peri = cv2.arcLength(cnt, True)
        if peri == 0:
            return False
        if shape == 'circle':
            return 4 * math.pi * area / peri ** 2 > 0.75
        n = len(cv2.approxPolyDP(cnt, 0.02 * peri, True))
        return {'octagon': 7 <= n <= 9, 'triangle': n == 3, 'rectangle': n == 4}.get(shape, False)

    @staticmethod
    def _mask(hsv, ranges):
        m = np.zeros(hsv.shape[:2], np.uint8)
        for r in ranges:
            m |= cv2.inRange(hsv, np.array(r[:3], np.uint8), np.array(r[3:], np.uint8))
        k = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)

    def detect(self, bgr):
        """한 프레임에서 면적 최대 후보 {'cls','area','cx','box'} 또는 None.
        area는 영상 면적 대비 비율, cx는 -1~1."""
        h0, w0 = bgr.shape[:2]
        if w0 != self.process_width:
            bgr = cv2.resize(bgr, (self.process_width, int(h0 * self.process_width / w0)))
        H, W = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        best = None
        for name, cfg in self.signs.items():
            cnts, _ = cv2.findContours(self._mask(hsv, cfg['hsv_ranges']),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area / (W * H) < cfg['min_area_ratio'] or not self._shape_ok(cfg['shape'], c, area):
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if name in self.templates:
                    crop = cv2.resize(gray[y:y + h, x:x + w], (64, 64))
                    score = cv2.matchTemplate(crop, self.templates[name], cv2.TM_CCOEFF_NORMED)[0, 0]
                    if score < self.template_threshold:
                        continue
                if best is None or area > best['area_px']:
                    best = {'cls': name, 'area_px': area, 'area': area / (W * H),
                            'cx': (x + w / 2 - W / 2) / (W / 2), 'box': (x, y, w, h)}
        return best

    def debounce(self, cls):
        """프레임별 클래스(없으면 None)를 넣으면, 확정되는 순간에만 클래스명을 돌려준다."""
        if cls is None:
            self._cand, self._count = None, 0
        elif cls == self._cand:
            self._count += 1
        else:
            self._cand, self._count = cls, 1
        if self._active is not None:
            self._miss = 0 if cls == self._active else self._miss + 1
            if self._miss >= self.release_frames:
                self._active, self._miss = None, 0
        if self._active is None and self._cand is not None and self._count >= self.confirm_frames:
            self._active, self._miss = self._cand, 0
            return self._active
        return None

    def process(self, bgr):
        det = self.detect(bgr)
        return det, self.debounce(det['cls'] if det else None)

    def draw_debug(self, bgr, det):
        img = cv2.resize(bgr, (self.process_width, int(bgr.shape[0] * self.process_width / bgr.shape[1])))
        if det:
            x, y, w, h = det['box']
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, det['cls'], (x, max(y - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return img
