import math

from robot_base import kinematics as k

R, L = 0.033, 0.160


def raw(v, w, **kw):
    args = dict(wheel_base=L, wheel_radius=R, max_linear=0.3, max_angular=2.0, limit_raw=265)
    args.update(kw)
    return k.cmd_to_raw(v, w, **args)


def test_straight_wheel_speeds():
    assert k.twist_to_wheels(0.2, 0.0, L) == (0.2, 0.2)


def test_spin_wheel_speeds():
    vl, vr = k.twist_to_wheels(0.0, 1.0, L)
    assert vl == -0.08 and vr == 0.08


def test_raw_conversion_and_sign():
    l, r = raw(0.1, 0.0)
    expect = round(k.wheel_to_raw(0.1, R))
    assert l == expect and r == -expect  # 기본 부호 (1, -1)
    assert 0 < expect < 265


def test_raw_limit_clamp():
    l, r = raw(10.0, 0.0)  # max_linear 0.3 -> 한계(265) 이하로 제한
    assert abs(l) <= 265 and abs(r) <= 265


def test_ratio_scaling_keeps_radius():
    # 한계를 낮춰 비율 감속 유도: 좌우 비가 유지돼야 한다
    free = raw(0.2, 1.0, limit_raw=10000)
    cut = raw(0.2, 1.0, limit_raw=100)
    assert max(abs(cut[0]), abs(cut[1])) == 100
    assert math.isclose(cut[0] / cut[1], free[0] / free[1], rel_tol=0.03)


def test_raw_to_rad_s_roundtrip():
    r_ = k.wheel_to_raw(0.1, R)
    assert math.isclose(k.raw_to_rad_s(r_) * R, 0.1, rel_tol=1e-9)


def test_tick_wraparound():
    assert k.tick_delta(-2**31 + 10, 2**31 - 10) == 20
    assert k.tick_delta(2**31 - 10, -2**31 + 10) == -20
    assert k.tick_delta(100, 40) == 60


def test_odometry_one_meter_straight():
    o = k.Odometry(R, L, 1, -1)
    ticks_per_m = k.TICKS_PER_REV / (2 * math.pi * R)
    n = 100
    for _ in range(n):
        # 오른쪽 바퀴는 부호 -1이라 tick이 반대로 증가
        o.update(ticks_per_m / n, -ticks_per_m / n, 0.05)
    assert math.isclose(o.x, 1.0, abs_tol=1e-6) and abs(o.y) < 1e-9 and abs(o.th) < 1e-9


def test_odometry_ninety_degree_spin():
    o = k.Odometry(R, L, 1, -1)
    arc = (math.pi / 2) * L / 2  # 각 바퀴가 움직일 호 길이
    t = arc / (2 * math.pi * R) * k.TICKS_PER_REV
    n = 90
    for _ in range(n):
        o.update(-t / n, -t / n, 0.05)  # 왼쪽 뒤로(-), 오른쪽 앞으로(sign -1 이라 tick -)
    assert math.isclose(o.th, math.pi / 2, abs_tol=1e-6)
    assert abs(o.x) < 1e-9
