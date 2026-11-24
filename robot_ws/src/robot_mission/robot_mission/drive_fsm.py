"""종목 1(자율주행) 상태 머신. ROS 비의존: 시계와 센서 입력을 인자로 받는다."""
import math

import numpy as np

IDLE, LINE_FOLLOW, STOP_HOLD, LINE_SEARCH, AVOID = 'IDLE', 'LINE_FOLLOW', 'STOP_HOLD', 'LINE_SEARCH', 'AVOID'

DEFAULTS = dict(
    v_max=0.15, v_min_ratio=0.4, k_slow=0.6, kp=1.8, kd=0.15, search_angular=0.8,
    line_lost_grace=0.3, line_lost_timeout=3.0, stop_hold_sec=3.0, stop_cooldown_sec=4.0,
    obstacle_dist=0.30, obstacle_clear_dist=0.40, avoid_mode='stop',
    bypass_turn_angle=0.6, bypass_forward_dist=0.25, avoid_timeout=8.0,
    lap_radius=0.25, lap_min_dist=1.0, laps_to_finish=0,
)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def scan_sectors(ranges, angle_min, angle_inc, range_min, range_max,
                 front_half_angle=0.35, yaw_offset=0.0):
    """LaserScan -> (전방 최솟값, 좌측 평균, 우측 평균). 유효값이 없으면 None.
    좌/우 섹터는 각각 60~120도."""
    r = np.asarray(ranges, float)
    ang = (angle_min + np.arange(len(r)) * angle_inc - yaw_offset + np.pi) % (2 * np.pi) - np.pi
    ok = np.isfinite(r) & (r > range_min) & (r <= range_max)
    lo, hi = math.radians(60), math.radians(120)

    def pick(sel):
        v = r[ok & sel]
        return v

    front = pick(np.abs(ang) <= front_half_angle)
    left = pick((ang >= lo) & (ang <= hi))
    right = pick((ang <= -lo) & (ang >= -hi))
    return (float(front.min()) if front.size else None,
            float(left.mean()) if left.size else None,
            float(right.mean()) if right.size else None)


class DriveFSM:
    def __init__(self, clock, **params):
        self.clock = clock
        self.p = {**DEFAULTS, **params}
        self.state = IDLE
        self._reset()

    def _reset(self):
        self._prev_e = self._prev_t = None
        self._last_e = 0.0
        self._last_cmd = (0.0, 0.0)
        self._lost_since = None
        self._cooldown_until = 0.0
        self._sub = None
        self.laps = 0
        self._away = False

    def start(self):
        if self.state == IDLE:
            self._reset()
            self.state = LINE_FOLLOW

    def stop(self):
        self.state = IDLE
        self._reset()

    def _to(self, state, now):
        self.state, self._t_enter = state, now

    def step(self, line_found, line_error, sign, front, left, right, pose, map_xy=None):
        """반환 (v, ω, state). sign은 이번 주기에 새로 확정된 클래스명 또는 None.
        front/left/right는 scan_sectors 결과(None = 유효 장애물 없음). pose = (x, y, yaw).
        map_xy = SLAM 지도 위치 (x, y) 또는 None(바퀴 수 판정만 멈춤)."""
        now, p = self.clock(), self.p
        if self.state == IDLE:
            return 0.0, 0.0, IDLE

        if self._count_lap(map_xy):
            self.state = IDLE
            return 0.0, 0.0, IDLE

        if self.state == STOP_HOLD:
            if now - self._t_enter < p['stop_hold_sec']:
                return 0.0, 0.0, STOP_HOLD
            self._cooldown_until = now + p['stop_cooldown_sec']
            self._to(LINE_FOLLOW, now)
            self._prev_e = None

        if self.state == AVOID:
            return self._avoid(now, line_found, front, left, right, pose)

        if self.state == LINE_SEARCH:
            if line_found:
                self._to(LINE_FOLLOW, now)
                self._prev_e = None
            elif now - self._lost_since > p['line_lost_timeout']:
                self.state = IDLE
                return 0.0, 0.0, IDLE
            else:
                d = -1.0 if self._last_e > 0 else 1.0  # 마지막 편차 쪽으로 회전
                return 0.0, d * p['search_angular'], LINE_SEARCH

        # LINE_FOLLOW
        if sign == 'stop' and now >= self._cooldown_until:
            self._to(STOP_HOLD, now)
            return 0.0, 0.0, STOP_HOLD
        if front is not None and front < p['obstacle_dist']:
            self._enter_avoid(now, left, right, pose)
            return self._avoid(now, line_found, front, left, right, pose)
        if not line_found:
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since <= p['line_lost_grace']:
                return (*self._last_cmd, LINE_FOLLOW)
            self._to(LINE_SEARCH, now)
            d = -1.0 if self._last_e > 0 else 1.0
            return 0.0, d * p['search_angular'], LINE_SEARCH
        self._lost_since = None

        e = line_error
        de = 0.0
        if self._prev_e is not None and now > self._prev_t:
            de = (e - self._prev_e) / (now - self._prev_t)
        self._prev_e, self._prev_t = e, now
        if e != 0.0:
            self._last_e = e
        w = -(p['kp'] * e + p['kd'] * de)
        v = p['v_max'] * max(p['v_min_ratio'], 1.0 - p['k_slow'] * abs(e))
        self._last_cmd = (v, w)
        return v, w, LINE_FOLLOW

    def _count_lap(self, xy):
        """바퀴 수 갱신. laps_to_finish 도달 시 True. ponytail: 원점 반경 판정만 쓰고 진행 방향은 보지 않음, 역주행 오검출이 문제되면 각도 누적 추가."""
        if xy is None:
            return False
        p = self.p
        # 출발점 = 지도 원점. 지도 작성을 출발 위치에서 시작하는 운용 전제(slam_toolbox가 첫 위치를 원점으로 잡음)
        d = math.hypot(xy[0], xy[1])
        if not self._away:
            self._away = d >= p['lap_min_dist']
        elif d <= p['lap_radius']:
            self._away = False
            self.laps += 1
        return 0 < p['laps_to_finish'] <= self.laps

    def _enter_avoid(self, now, left, right, pose):
        self._to(AVOID, now)
        self._sub = 'TURN_OUT'
        lv = float('inf') if left is None else left
        rv = float('inf') if right is None else right
        self._dir = 1.0 if lv >= rv else -1.0  # 넓은 쪽(왼쪽 +)
        self._yaw0 = pose[2]

    def _avoid(self, now, line_found, front, left, right, pose):
        p = self.p
        if now - self._t_enter > p['avoid_timeout']:
            self.state = IDLE
            return 0.0, 0.0, IDLE
        clear = front is None or front >= p['obstacle_clear_dist']
        if p['avoid_mode'] == 'stop':
            if clear:
                self._to(LINE_FOLLOW, now)
                self._prev_e = None
                self._lost_since = None
                return 0.0, 0.0, LINE_FOLLOW
            return 0.0, 0.0, AVOID
        # bypass
        if self._sub == 'TURN_OUT':
            if clear and abs(_wrap(pose[2] - self._yaw0)) >= p['bypass_turn_angle']:
                self._sub, self._xy0 = 'PASS', (pose[0], pose[1])
            else:
                return 0.0, self._dir * p['search_angular'], AVOID
        if self._sub == 'PASS':
            if math.hypot(pose[0] - self._xy0[0], pose[1] - self._xy0[1]) >= p['bypass_forward_dist']:
                self._sub = 'TURN_IN'
            else:
                return p['v_max'], 0.0, AVOID
        # TURN_IN: 반대쪽으로 돌며 전진, 라인이 보이면 복귀
        if line_found:
            self._to(LINE_FOLLOW, now)
            self._prev_e = None
            self._lost_since = None
            return 0.0, 0.0, LINE_FOLLOW
        return p['v_max'] * p['v_min_ratio'], -self._dir * p['search_angular'], AVOID
