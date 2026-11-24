"""경기 탭 경고 목록. ROS 비의존. 입력은 GuiNode.snapshot()의 dict."""

# 모드와 상관없이 끊기면 안 되는 토픽
COMMON = ('/scan', '/odom', '/wheel/velocity', '/mission/state')
# 모드별로 더 보는 토픽. /hand는 손이 없으면 원래 발행되지 않으므로 넣지 않는다.
BY_MODE = {'drive': ('/line/found',), 'follow': ()}


def alerts(s):
    out = []
    mode = s.get('mode')
    if mode is None:
        out.append('모드 정보 없음 — Pi에서 mode_manager가 떠 있는지, ROS_DOMAIN_ID가 같은지 확인')
    topics = s.get('topics', {})
    for t in COMMON + BY_MODE.get(mode, ()):
        if topics.get(t, {}).get('stale', True):
            out.append(f'{t} 끊김')
    if mode == 'drive' and s.get('line', {}).get('found') is False:
        out.append('라인 안 보임')
    if mode == 'drive' and (topics.get('/mission/slam_ok', {}).get('stale', True) or s.get('slam_ok') is False):
        out.append('SLAM 위치 없음 — 주행은 계속, 지도 위치만 끊김')
    text, bad = s.get('status') or ('', False)
    if bad:
        out.append(text)
    return out


SLAM_LABEL = {'localization': '지도 불러옴', 'mapping': '지도 작성 중'}


def slam_label(v):
    """/mission/slam 값 -> [경기] 탭 표시. off·없음은 표시 안 함."""
    return SLAM_LABEL.get(v, '')


def lap_text(lap, total):
    """바퀴 수 표시. total(laps_to_finish)이 0이면 숫자만."""
    n = '-' if lap is None else str(lap)
    return f'{n} / {total}' if total > 0 else n
