"""노란 라인 검출. ROS 비의존."""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LineResult:
    found: bool
    error: float = 0.0
    points: list = field(default_factory=list)  # 디버그용 (x, y) 검출점
    mask: object = None
    n: int = 0  # near ROI에서 보이는 줄 수 (lane3는 합친 뒤)
    lcr: object = None  # lane3: near ROI의 (L, C, R) x, 0~1 정규화. 없으면 None


class LineDetector:
    def __init__(self, process_width=320, hsv_lower=(18, 80, 80), hsv_upper=(35, 255, 255),
                 roi_near=(0.80, 1.00), roi_far=(0.60, 0.80), weight_near=0.6, weight_far=0.4,
                 min_area_px=40, line_mode='lane3', lane_half_width_px=100, max_jump=0.5,
                 lane3_pair='outer', lane3_gap_px=80, lane3_gap_far_px=60, lane3_forget_frames=10,
                 merge_gap_px=15):
        self.process_width = process_width
        self.lower = np.array(hsv_lower, np.uint8)
        self.upper = np.array(hsv_upper, np.uint8)
        self.rois = {'near': tuple(roi_near), 'far': tuple(roi_far)}
        self.weights = {'near': weight_near, 'far': weight_far}
        self.min_area = min_area_px
        self.mode = line_mode
        self.half_w = lane_half_width_px
        self.max_jump = max_jump
        self.pair = lane3_pair
        self.gap_px = {'near': lane3_gap_px, 'far': lane3_gap_far_px}
        self.forget = lane3_forget_frames
        self.merge_gap = merge_gap_px
        self._lane3 = {}  # ROI별 {'g': 학습한 줄 간격, 'p': 직전 [L, C, R] x}
        self._detail = (0, None)
        self._prev = {}  # ROI별 직전 기준점 x
        self._lane = {}  # ROI별 {'l': 왼쪽 경계 x, 'r': 오른쪽 경계 x, 'half': 학습한 반폭}

    def _blobs(self, mask, y0, y1):
        """ROI 안의 (면적, 무게중심 x) 목록. 면적 미만은 버린다."""
        cnts, _ = cv2.findContours(mask[y0:y1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            m = cv2.moments(c)
            if m['m00'] >= self.min_area:
                out.append((m['m00'], m['m10'] / m['m00']))
        return out

    def _ref_x(self, name, blobs, width):
        if not blobs:
            return None
        if self.mode == 'lane':
            return self._lane_x(name, sorted(b[1] for b in blobs), width)
        if self.mode == 'lane3':
            return self._lane3_x(name, sorted(b[1] for b in blobs), width)
        prev = self._prev.get(name)
        if prev is not None:
            near = [b for b in blobs if abs(b[1] - prev) <= self.max_jump * width]
            blobs = near or blobs  # 가까운 덩어리가 없으면 라인이 실제로 이동한 것으로 본다
        return max(blobs)[1]

    def _lane_x(self, name, xs, width):
        """양쪽 경계 2줄 트랙의 차로 중앙 x.
        두 줄이 보이면 ROI별 반폭을 학습하고(원근 때문에 near/far 폭이 다름),
        한 줄만 보이면 직전 경계와 가까운 쪽으로 좌우를 정한다(급커브에서 화면 절반을 넘어가도 유지)."""
        st = self._lane.setdefault(name, {'l': None, 'r': None, 'half': float(self.half_w)})
        l, r = xs[0], xs[-1]
        if r - l > 0.5 * st['half']:  # 두 경계 (이보다 가까우면 한 줄이 끊겨 보인 것으로 본다)
            st['half'] = 0.7 * st['half'] + 0.3 * (r - l) / 2.0
            st['l'], st['r'] = l, r
            return (l + r) / 2.0
        x = (l + r) / 2.0
        known = [(abs(x - st[s]), s) for s in ('l', 'r') if st[s] is not None]
        side = min(known)[1] if known else ('l' if x < width / 2.0 else 'r')
        if side == 'l':
            st['l'], st['r'] = x, x + 2 * st['half']
            return x + st['half']
        st['l'], st['r'] = x - 2 * st['half'], x
        return x - st['half']

    def _forget(self, name, seen):
        """줄이 0개인 프레임이 forget 번 연속이면 직전 L·C·R을 지운다(낡은 위치로 1줄을 판정하지 않게). 간격은 유지."""
        st = self._lane3.setdefault(name, {'g': float(self.gap_px[name]), 'p': None, 'miss': 0})
        st['miss'] = 0 if seen else st['miss'] + 1
        if st['miss'] >= self.forget:
            st['p'] = None

    def _lane3_x(self, name, xs, width):
        """3줄 트랙의 기준점 x. 줄 이름(L·C·R)은 줄 수와 직전 위치로 정한다.
        ponytail: 줄 간격 L-C=C-R 가정, 다르면 간격 2개 학습."""
        st = self._lane3.setdefault(name, {'g': float(self.gap_px[name]), 'p': None, 'miss': 0})
        merged = [[xs[0]]]  # 가까운 덩어리는 끊긴 한 줄(점선 가운데 줄)로 합친다
        for x in xs[1:]:
            if x - merged[-1][-1] < self.merge_gap:
                merged[-1].append(x)
            else:
                merged.append([x])
        xs = [sum(m) / len(m) for m in merged]
        g, p = st['g'], st['p']
        if len(xs) >= 3:
            l, r = xs[0], xs[-1]
            lcr = [l, min(xs[1:-1], key=lambda x: abs(x - (l + r) / 2.0)), r]
            st['g'] = 0.7 * g + 0.3 * (r - l) / 2.0
        elif len(xs) == 2:
            a, b = xs
            if b - a > 1.5 * g:  # L·R
                lcr = [a, (a + b) / 2.0, b]
                st['g'] = 0.7 * g + 0.3 * (b - a) / 2.0
            else:  # 이웃한 두 줄: L·C 또는 C·R
                lc, cr = [a, b, b + g], [a - g, a, b]
                if p:
                    lc_first = abs(a - p[0]) + abs(b - p[1]) <= abs(a - p[1]) + abs(b - p[2])
                else:
                    lc_first = abs(b - width / 2.0) < abs(a - width / 2.0)
                lcr = lc if lc_first else cr
                st['g'] = 0.7 * g + 0.3 * (b - a)
        else:
            if not p:
                return None  # 직전 정보가 없는 1줄은 판단 불가
            x = xs[0]
            i = min(range(3), key=lambda k: abs(x - p[k]))
            lcr = [x + (k - i) * g for k in range(3)]
        st['p'] = lcr
        if name == 'near':
            self._detail = (len(xs), tuple(v / width for v in lcr))
        l, c, r = lcr
        return {'outer': (l + r) / 2.0, 'left': (l + c) / 2.0, 'right': (c + r) / 2.0}[self.pair]

    def detect(self, bgr):
        h0, w0 = bgr.shape[:2]
        if w0 != self.process_width:
            bgr = cv2.resize(bgr, (self.process_width, int(h0 * self.process_width / w0)))
        H, W = bgr.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        errs, pts = {}, []
        self._detail = (0, None)
        for name, (a, b) in self.rois.items():
            y0, y1 = int(H * a), int(H * b)
            blobs = self._blobs(mask, y0, y1)
            if name == 'near':
                self._detail = (len(blobs), None)
            if self.mode == 'lane3':
                self._forget(name, bool(blobs))
            x = self._ref_x(name, blobs, W)
            self._prev[name] = x
            if x is not None:
                errs[name] = max(-1.0, min(1.0, (x - W / 2.0) / (W / 2.0)))
                pts.append((int(x), (y0 + y1) // 2))
        if not errs:
            return LineResult(False, mask=mask, n=self._detail[0])
        wsum = sum(self.weights[n] for n in errs)
        err = sum(self.weights[n] * e for n, e in errs.items()) / wsum
        return LineResult(True, float(err), pts, mask, *self._detail)

    def draw_debug(self, bgr, res):
        H, W = res.mask.shape
        img = cv2.resize(bgr, (W, H))
        img[res.mask > 0] = (0, 255, 0)
        for a, b in self.rois.values():
            cv2.rectangle(img, (0, int(H * a)), (W - 1, int(H * b) - 1), (255, 0, 0), 1)
        for p in res.points:
            cv2.circle(img, p, 4, (0, 0, 255), -1)
        if res.lcr:
            y = int(H * sum(self.rois['near']) / 2)
            for name, v in zip('LCR', res.lcr):
                cv2.putText(img, name, (int(v * W) - 5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return img
