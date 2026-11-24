import math

from robot_gui.scan_util import reduce_scan, to_xy


def test_reduce_takes_min_per_bin():
    r = reduce_scan([1.0, 2.0, 3.0, 4.0], 0.0, 0.1, 2)
    assert r['ranges'] == [1.0, 3.0] and len(r['angles']) == 2


def test_invalid_values_become_none():
    r = reduce_scan([math.inf, math.nan, 9.0, 0.0], 0.0, 0.1, 2, range_max=5.0)
    assert r['ranges'] == [None, None]


def test_points_capped_and_empty():
    assert len(reduce_scan([1.0] * 5, 0.0, 0.1, 180)['ranges']) == 5
    assert reduce_scan([], 0.0, 0.1, 10) == {'angles': [], 'ranges': []}


def test_to_xy_skips_none_and_forward_is_x():
    pts = to_xy({'angles': [0.0, math.pi / 2, 1.0], 'ranges': [2.0, 1.0, None]})
    assert len(pts) == 2 and pts[0] == (2.0, 0.0)
    assert abs(pts[1][0]) < 1e-9 and pts[1][1] == 1.0
