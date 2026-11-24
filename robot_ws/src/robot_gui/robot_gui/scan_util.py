"""LaserScan을 points개 구간으로 줄인다. 구간마다 가장 가까운 유효 거리를 쓴다."""
import math


def reduce_scan(ranges, angle_min, angle_inc, points, range_max=math.inf):
    n = len(ranges)
    points = min(points, n)
    if points <= 0:
        return {'angles': [], 'ranges': []}
    angles, out = [], []
    for i in range(points):
        lo, hi = i * n // points, (i + 1) * n // points
        valid = [r for r in ranges[lo:hi] if math.isfinite(r) and 0 < r <= range_max]
        angles.append(round(angle_min + (lo + hi - 1) / 2 * angle_inc, 4))
        out.append(round(min(valid), 3) if valid else None)  # 무효 구간은 JSON null
    return {'angles': angles, 'ranges': out}


def to_xy(scan):
    """reduce_scan 결과 -> [(x 전방, y 왼쪽)] 미터. 무효 점은 뺀다."""
    return [(r * math.cos(a), r * math.sin(a))
            for a, r in zip(scan['angles'], scan['ranges']) if r is not None]
