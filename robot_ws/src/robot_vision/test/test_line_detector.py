import numpy as np

from robot_vision.line_detector import LineDetector

YELLOW = (0, 230, 255)  # BGR, H=약 27


def frame(bands, w=320, h=240):
    img = np.full((h, w, 3), 120, np.uint8)
    for x0, x1, y0 in bands:
        img[y0:, x0:x1] = YELLOW
    return img


def test_center_zero():
    r = LineDetector(line_mode='center').detect(frame([(150, 170, 100)]))
    assert r.found and abs(r.error) < 0.05


def test_left_negative_right_positive():
    left = LineDetector(line_mode='center').detect(frame([(40, 60, 100)]))
    right = LineDetector(line_mode='center').detect(frame([(260, 280, 100)]))
    assert left.error < -0.5 and right.error > 0.5
    assert abs(left.error + 0.6875) < 0.05  # 중심 x=50 -> (50-160)/160


def test_no_line():
    r = LineDetector(line_mode='center').detect(frame([]))
    assert not r.found


def test_small_blob_ignored():
    img = frame([])
    img[220:224, 100:104] = YELLOW  # 16px < min_area 40
    assert not LineDetector(line_mode='center').detect(img).found


def test_lane_two_lines_midpoint():
    r = LineDetector(line_mode='lane').detect(frame([(60, 80, 100), (240, 260, 100)]))
    assert r.found and abs(r.error) < 0.05


def test_lane_single_line_offsets_inward():
    # 왼쪽 경계 1줄(x=70) -> 기준점은 70+100=170 -> 오른쪽 약간
    r = LineDetector(line_mode='lane', lane_half_width_px=100).detect(frame([(60, 80, 100)]))
    assert r.found and abs(r.error - (170 - 160) / 160) < 0.05
    r = LineDetector(line_mode='lane', lane_half_width_px=100).detect(frame([(240, 260, 100)]))
    assert r.found and abs(r.error - (150 - 160) / 160) < 0.05


def test_lane_learns_half_width():
    # 두 줄(70, 250 -> 반폭 90)을 본 뒤 왼쪽 한 줄만 보이면 학습한 반폭으로 중앙을 잡는다
    d = LineDetector(line_mode='lane', lane_half_width_px=100)
    for _ in range(20):
        d.detect(frame([(60, 80, 100), (240, 260, 100)]))
    r = d.detect(frame([(60, 80, 100)]))
    assert abs(r.error - (160 - 160) / 160) < 0.03


def test_lane_curve_keeps_side_past_center():
    # 오른쪽 라인이 사라진 뒤 왼쪽 라인이 화면 중앙을 넘어가도 왼쪽 경계로 유지 -> 기준점은 더 오른쪽
    d = LineDetector(line_mode='lane', lane_half_width_px=90)
    d.detect(frame([(60, 80, 100), (240, 260, 100)]))
    for x in range(70, 211, 10):
        r = d.detect(frame([(x - 10, x + 10, 100)]))
    assert r.found and r.error > (210 - 160) / 160  # 오른쪽 경계로 오인하면 음수 쪽이 된다


def test_lane_broken_line_is_single():
    # 한 줄이 두 덩어리로 끊겨도(간격 < 반폭/2) 한 줄로 처리
    r = LineDetector(line_mode='lane', lane_half_width_px=100).detect(frame([(60, 70, 100), (80, 90, 100)]))
    assert r.found and abs(r.error - (175 - 160) / 160) < 0.05


def test_far_only_uses_far():
    img = frame([(250, 270, 150)])
    img[200:, :] = 120  # near ROI(192~240) 비움
    r = LineDetector(line_mode='center').detect(img)
    assert r.found and r.error > 0.4


# ---- lane3 (L=60, C=160, R=260 근처, 띠 폭 10) ----
L3, C3, R3 = (55, 65, 100), (155, 165, 100), (255, 265, 100)


def lane3_primed(**kw):
    d = LineDetector(line_mode='lane3', **kw)
    for _ in range(20):
        d.detect(frame([L3, C3, R3]))
    return d


def test_lane3_three_lines_outer_midpoint():
    r = LineDetector(line_mode='lane3').detect(frame([L3, C3, R3]))
    assert r.found and abs(r.error) < 0.05 and r.n == 3
    assert [round(v * 320) for v in r.lcr] == [60, 160, 260]


def test_lane3_pair_left_and_right():
    assert abs(LineDetector(line_mode='lane3', lane3_pair='left').detect(frame([L3, C3, R3])).error - (-0.3125)) < 0.05
    assert abs(LineDetector(line_mode='lane3', lane3_pair='right').detect(frame([L3, C3, R3])).error - 0.3125) < 0.05


def test_lane3_two_lines_identified():
    # L·R: 간격이 1.5g보다 큼
    r = lane3_primed().detect(frame([L3, R3]))
    assert r.n == 2 and abs(r.error) < 0.05
    # L·C: 직전 위치와 가까운 쪽 -> R은 g만큼 떨어진 곳으로 추정
    r = lane3_primed().detect(frame([L3, C3]))
    assert abs(r.error) < 0.06 and abs(r.lcr[2] * 320 - 260) < 15
    # C·R
    r = lane3_primed().detect(frame([C3, R3]))
    assert abs(r.error) < 0.06 and abs(r.lcr[0] * 320 - 60) < 15


def test_lane3_two_neighbors_without_prev_uses_center():
    # 직전 없음: 화면 중앙에 가까운 줄(x=160)을 C로 본다 -> C·R
    r = LineDetector(line_mode='lane3', lane3_gap_px=100).detect(frame([C3, R3]))
    assert abs(r.lcr[0] * 320 - 60) < 15


def test_lane3_one_line_tracks_previous():
    for band, expect in ((L3, 60), (C3, 160), (R3, 260)):
        r = lane3_primed().detect(frame([band]))
        assert r.found and r.n == 1 and abs(r.error) < 0.06
    # 바깥 줄이 화면 밖으로 밀려도 계산은 이어지고 편차는 clamp
    r = lane3_primed().detect(frame([(285, 295, 100)]))
    assert r.found and -1.0 <= r.error <= 1.0


def test_lane3_dashed_center_merged():
    img = frame([L3, R3])
    img[192:210, 150:170] = YELLOW  # 가운데 점선: 같은 x의 두 토막
    img[222:240, 150:170] = YELLOW
    r = LineDetector(line_mode='lane3').detect(img)
    assert r.n == 3 and abs(r.error) < 0.05


def test_lane3_forgets_stale_track():
    d = lane3_primed(lane3_forget_frames=3)
    for _ in range(3):
        assert not d.detect(frame([])).found
    assert not d.detect(frame([C3])).found


def test_lane3_one_line_without_prev_not_found():
    r = LineDetector(line_mode='lane3').detect(frame([C3]))
    assert not r.found
