"""차동 구동 기구학과 오도메트리. ROS 비의존 순수 함수."""
import math

TICKS_PER_REV = 4096
RAW_UNIT_RPM = 0.229  # Goal/Present Velocity 1단위 = 0.229 rpm


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def twist_to_wheels(v, w, wheel_base):
    """(v, ω) -> (왼쪽, 오른쪽) 바퀴 선속도 m/s."""
    return v - w * wheel_base / 2.0, v + w * wheel_base / 2.0


def wheel_to_raw(v_wheel, wheel_radius):
    """바퀴 선속도 m/s -> Goal Velocity 원시값(부호 적용 전, 실수)."""
    rpm = (v_wheel / wheel_radius) * 60.0 / (2.0 * math.pi)
    return rpm / RAW_UNIT_RPM


def raw_to_rad_s(raw):
    return raw * RAW_UNIT_RPM * 2.0 * math.pi / 60.0


def cmd_to_raw(v, w, wheel_base, wheel_radius, max_linear, max_angular,
               limit_raw, left_sign=1, right_sign=-1):
    """/cmd_vel -> (좌, 우) 원시값 int. v, ω를 clamp하고, 그래도 한계를 넘으면 비율 축소."""
    v = clamp(v, -max_linear, max_linear)
    w = clamp(w, -max_angular, max_angular)
    vl, vr = twist_to_wheels(v, w, wheel_base)
    rl, rr = wheel_to_raw(vl, wheel_radius), wheel_to_raw(vr, wheel_radius)
    peak = max(abs(rl), abs(rr))
    if peak > limit_raw:
        k = limit_raw / peak
        rl, rr = rl * k, rr * k
    return int(round(rl)) * left_sign, int(round(rr)) * right_sign


def tick_delta(cur, prev):
    """int32 랩어라운드를 처리한 tick 차이."""
    d = (cur - prev) & 0xFFFFFFFF
    return d - (1 << 32) if d >= (1 << 31) else d


class Odometry:
    """엔코더 tick 증분으로 (x, y, θ)를 적분한다."""

    def __init__(self, wheel_radius, wheel_base, left_sign=1, right_sign=-1):
        self.r, self.L = wheel_radius, wheel_base
        self.ls, self.rs = left_sign, right_sign
        self.x = self.y = self.th = 0.0

    def update(self, d_tick_l, d_tick_r, dt):
        """반환: (v, ω) 이번 주기의 선속도·각속도."""
        k = 2.0 * math.pi * self.r / TICKS_PER_REV
        ds_l, ds_r = d_tick_l * k * self.ls, d_tick_r * k * self.rs
        ds = (ds_r + ds_l) / 2.0
        dth = (ds_r - ds_l) / self.L
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = normalize_angle(self.th + dth)
        if dt <= 0:
            return 0.0, 0.0
        return ds / dt, dth / dt
