"""종목 2(손 추적 이동) 상태 머신. ROS 비의존: 시계와 센서 입력을 인자로 받는다."""
import math

IDLE, WAIT_HAND, FOLLOW, UTURN = 'IDLE', 'WAIT_HAND', 'FOLLOW', 'UTURN'

DEFAULTS = dict(kx=1.5, kz=1.2, target_size=0.35, v_max=0.20, hand_lost_timeout=0.5,
                uturn_angular=1.2, uturn_tolerance=0.08, use_scan=True, obstacle_dist=0.20)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FollowFSM:
    def __init__(self, clock, **params):
        self.clock = clock
        self.p = {**DEFAULTS, **params}
        self.state = IDLE

    def start(self):
        if self.state == IDLE:
            self.state = WAIT_HAND

    def stop(self):
        self.state = IDLE

    def step(self, hand, sign, front, yaw):
        """반환 (v, ω, state). hand = (x, y, z) 또는 None, sign = 새로 확정된 클래스명 또는 None,
        front = 전방 최소 거리(None = 장애물 없음), yaw = odom 방위각."""
        now, p = self.clock(), self.p
        if self.state == IDLE:
            return 0.0, 0.0, IDLE

        if self.state == UTURN:
            self._turned += _wrap(yaw - self._prev_yaw)
            self._prev_yaw = yaw
            if self._turned >= math.pi - p['uturn_tolerance']:
                self.state = WAIT_HAND
                return 0.0, 0.0, WAIT_HAND
            return 0.0, p['uturn_angular'], UTURN

        if self.state == WAIT_HAND:
            if hand is None:
                return 0.0, 0.0, WAIT_HAND
            self.state, self._last_seen = FOLLOW, now

        # FOLLOW
        if sign == 'uturn':
            self.state, self._turned, self._prev_yaw = UTURN, 0.0, yaw
            return 0.0, p['uturn_angular'], UTURN
        if hand is None:
            if now - self._last_seen > p['hand_lost_timeout']:
                self.state = WAIT_HAND
                return 0.0, 0.0, WAIT_HAND
            return 0.0, 0.0, FOLLOW  # 잠깐 놓친 동안은 정지
        self._last_seen = now
        w = -p['kx'] * hand[0]
        v = min(max(p['kz'] * (p['target_size'] - hand[2]), 0.0), p['v_max'])
        if p['use_scan'] and front is not None and front < p['obstacle_dist']:
            v = 0.0
        return v, w, FOLLOW
