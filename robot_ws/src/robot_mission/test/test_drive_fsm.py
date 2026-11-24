import math

import numpy as np

from robot_mission.drive_fsm import DriveFSM, scan_sectors

POSE = (0.0, 0.0, 0.0)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(**kw):
    c = Clock()
    return DriveFSM(c, **kw), c


def tick(f, c, dt=0.05, found=True, e=0.0, sign=None, front=None, left=None, right=None, pose=POSE,
         xy=None):
    c.t += dt
    return f.step(found, e, sign, front, left, right, pose, map_xy=xy)


def test_idle_before_start():
    f, c = make()
    assert tick(f, c) == (0.0, 0.0, 'IDLE')


def test_pd_sign_and_slowdown():
    f, c = make()
    f.start()
    v0, w0, s = tick(f, c, e=0.0)
    assert s == 'LINE_FOLLOW' and w0 == 0.0 and math.isclose(v0, 0.15)
    v1, w1, _ = tick(f, c, e=0.5)  # 기준선이 오른쪽 -> ω < 0
    assert w1 < 0 and v1 < v0
    v2, w2, _ = tick(f, c, e=-0.5)  # 왼쪽 -> ω > 0
    assert w2 > 0


def test_stop_hold_resume_and_cooldown():
    f, c = make(stop_hold_sec=3.0, stop_cooldown_sec=4.0)
    f.start()
    tick(f, c)
    assert tick(f, c, sign='stop')[2] == 'STOP_HOLD'
    assert tick(f, c, dt=1.0) == (0.0, 0.0, 'STOP_HOLD')
    assert tick(f, c, dt=2.5)[2] == 'LINE_FOLLOW'  # 3초 경과
    assert tick(f, c, sign='stop')[2] == 'LINE_FOLLOW'  # 쿨다운 중 무시
    tick(f, c, dt=4.0)
    assert tick(f, c, sign='stop')[2] == 'STOP_HOLD'


def test_obstacle_stop_and_release():
    f, c = make(avoid_mode='stop')
    f.start()
    assert tick(f, c, front=0.2) == (0.0, 0.0, 'AVOID')
    assert tick(f, c, front=0.35)[2] == 'AVOID'  # 히스테리시스: 0.4 이상이어야 해제
    assert tick(f, c, front=0.5)[2] == 'LINE_FOLLOW'
    assert tick(f, c)[2] == 'LINE_FOLLOW'


def test_avoid_timeout_goes_idle():
    f, c = make(avoid_timeout=8.0)
    f.start()
    tick(f, c, front=0.2)
    assert tick(f, c, dt=9.0, front=0.2) == (0.0, 0.0, 'IDLE')


def test_bypass_sequence():
    f, c = make(avoid_mode='bypass')
    f.start()
    v, w, s = tick(f, c, front=0.2, left=1.0, right=0.3)
    assert s == 'AVOID' and w > 0  # 넓은 왼쪽으로 회전
    # 전방은 비었지만 아직 회전량 부족
    assert tick(f, c, front=1.0, left=1.0, right=0.3, pose=(0, 0, 0.3))[1] > 0
    # 0.6rad 도달 -> PASS 직진
    v, w, s = tick(f, c, front=1.0, pose=(0, 0, 0.65))
    assert v > 0 and w == 0 and s == 'AVOID'
    # 0.25m 직진 후 TURN_IN: 반대쪽(ω<0) 회전 + 전진
    v, w, s = tick(f, c, front=1.0, found=False, pose=(0.3, 0, 0.65))
    assert w < 0 and v > 0 and s == 'AVOID'
    assert tick(f, c, found=True, front=1.0, pose=(0.3, 0, 0.1))[2] == 'LINE_FOLLOW'


def test_line_lost_search_then_timeout():
    f, c = make()
    f.start()
    v, w, _ = tick(f, c, e=0.4)
    out = tick(f, c, found=False)  # grace 이내: 직전 명령 유지
    assert out == (v, w, 'LINE_FOLLOW')
    out = tick(f, c, dt=0.5, found=False)
    assert out[2] == 'LINE_SEARCH' and out[1] < 0  # 마지막 e>0 -> 오른쪽(ω<0)
    assert tick(f, c, found=True)[2] == 'LINE_FOLLOW'
    tick(f, c, found=False)
    tick(f, c, dt=0.5, found=False)
    assert tick(f, c, dt=3.5, found=False) == (0.0, 0.0, 'IDLE')


def test_mission_stop_from_any_state():
    f, c = make()
    f.start()
    tick(f, c, sign='stop')
    f.stop()
    assert tick(f, c)[2] == 'IDLE'


def test_scan_sectors():
    n = 360
    r = np.full(n, 2.0)
    r[0:5] = 0.25  # 0~4도 전방
    r[90] = np.inf
    front, left, right = scan_sectors(r, 0.0, math.radians(1), 0.05, 12.0)
    assert math.isclose(front, 0.25) and math.isclose(left, 2.0) and math.isclose(right, 2.0)
    assert scan_sectors(np.full(n, np.inf), 0.0, math.radians(1), 0.05, 12.0) == (None, None, None)


def _run(f, c, pts):
    for xy in pts:
        out = tick(f, c, xy=xy)
    return out


def test_lap_count_leave_and_return():
    f, c = make()
    f.start()
    _run(f, c, [(0, 0), (0.5, 0), (1.2, 0), (2, 0.5)])
    assert f.laps == 0
    _run(f, c, [(1.0, 1.0), (0.1, 0.1)])
    assert f.laps == 1
    _run(f, c, [(1.5, 0), (0.0, 0.0)])
    assert f.laps == 2


def test_lap_origin_is_map_origin():
    f, c = make()
    f.start()
    _run(f, c, [(2, 0), (0.1, 0.1)])  # 첫 위치가 원점에서 멀어도 원점 기준
    assert f.laps == 1


def test_lap_jitter_near_start_not_counted():
    f, c = make()
    f.start()
    _run(f, c, [(0, 0), (0.2, 0), (0.6, 0), (0.1, 0), (0.9, 0), (0.0, 0.1)])
    assert f.laps == 0


def test_laps_to_finish_goes_idle():
    f, c = make(laps_to_finish=1)
    f.start()
    assert _run(f, c, [(0, 0), (1.5, 0)])[2] == 'LINE_FOLLOW'
    assert _run(f, c, [(0.1, 0)]) == (0.0, 0.0, 'IDLE')
    assert f.laps == 1


def test_no_position_keeps_line_follow():
    f, c = make(laps_to_finish=1)
    f.start()
    v, w, st = _run(f, c, [None] * 5)
    assert st == 'LINE_FOLLOW' and v > 0 and f.laps == 0
