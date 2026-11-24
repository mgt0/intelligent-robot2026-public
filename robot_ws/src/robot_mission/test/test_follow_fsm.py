import math

from robot_mission.follow_fsm import FollowFSM


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(**kw):
    c = Clock()
    f = FollowFSM(c, **kw)
    f.start()
    return f, c


def tick(f, c, hand=None, sign=None, front=None, yaw=0.0, dt=0.05):
    c.t += dt
    return f.step(hand, sign, front, yaw)


def test_idle_before_start():
    c = Clock()
    assert FollowFSM(c).step((0, 0, 0.1), None, None, 0.0) == (0.0, 0.0, 'IDLE')


def test_wait_then_follow():
    f, c = make()
    assert tick(f, c)[2] == 'WAIT_HAND'
    assert tick(f, c, hand=(0.0, 0.0, 0.1))[2] == 'FOLLOW'


def test_rotation_sign():
    f, c = make()
    assert tick(f, c, hand=(0.5, 0, 0.35))[1] < 0   # 손이 오른쪽 -> ω<0
    assert tick(f, c, hand=(-0.5, 0, 0.35))[1] > 0  # 왼쪽 -> ω>0


def test_forward_by_distance():
    f, c = make(v_max=0.2, kz=1.2, target_size=0.35)
    far = tick(f, c, hand=(0, 0, 0.1))[0]
    near = tick(f, c, hand=(0, 0, 0.5))[0]
    assert far > 0 and near == 0.0  # 가까우면 정지, 후진 없음
    assert tick(f, c, hand=(0, 0, 0.0))[0] == 0.2  # v_max로 제한


def test_scan_blocks_forward():
    f, c = make(obstacle_dist=0.2)
    assert tick(f, c, hand=(0, 0, 0.1), front=0.1)[0] == 0.0
    f2, c2 = make(use_scan=False)
    assert tick(f2, c2, hand=(0, 0, 0.1), front=0.1)[0] > 0


def test_hand_lost_stops_within_timeout():
    f, c = make(hand_lost_timeout=0.5)
    tick(f, c, hand=(0, 0, 0.1))
    assert tick(f, c)[:2] == (0.0, 0.0)             # 놓친 직후 정지 명령
    assert tick(f, c, dt=0.6)[2] == 'WAIT_HAND'     # 타임아웃 후 대기


def test_uturn_completes_after_pi():
    f, c = make(uturn_tolerance=0.08)
    tick(f, c, hand=(0, 0, 0.1))
    assert tick(f, c, hand=(0, 0, 0.1), sign='uturn', yaw=0.0)[2] == 'UTURN'
    yaw, state = 0.0, 'UTURN'
    steps = 0
    while state == 'UTURN' and steps < 200:
        yaw += 0.05
        _, w, state = tick(f, c, yaw=math.atan2(math.sin(yaw), math.cos(yaw)))
        steps += 1
    assert state == 'WAIT_HAND'
    assert math.pi - 0.08 - 0.05 <= yaw <= math.pi + 0.05  # 랩어라운드 후에도 누적 회전량 판정


def test_stop_from_uturn():
    f, c = make()
    tick(f, c, hand=(0, 0, 0.1))
    tick(f, c, hand=(0, 0, 0.1), sign='uturn')
    f.stop()
    assert tick(f, c)[2] == 'IDLE'
