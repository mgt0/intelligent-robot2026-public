import pytest

from robot_gui.alerts import COMMON, alerts, lap_text, slam_label

OK = {'stale': False}


def state(mode='drive', **kw):
    s = {'mode': mode, 'topics': {t: OK for t in COMMON + ('/line/found', '/mission/slam_ok')},
         'line': {'found': True}, 'slam_ok': True, 'status': ('', False)}
    s.update(kw)
    return s


def test_all_ok_no_alerts():
    assert alerts(state()) == []
    assert alerts(state('follow')) == []


def test_no_mode():
    assert any('mode_manager' in a for a in alerts(state(None)))


def test_stale_and_missing_topic():
    s = state()
    s['topics'] = dict(s['topics'], **{'/scan': {'stale': True}})
    del s['topics']['/odom']
    a = alerts(s)
    assert '/scan 끊김' in a and '/odom 끊김' in a


def test_line_only_in_drive():
    assert '라인 안 보임' in alerts(state(line={'found': False}))
    s = state('follow', line={'found': False})
    s['topics'] = {t: OK for t in COMMON}  # follow에서는 /line/found 끊김도 경고 아님
    assert alerts(s) == []


def test_bad_status_shown():
    assert '/mission/start 실패: 2초 안에 응답 없음' in alerts(
        state(status=('/mission/start 실패: 2초 안에 응답 없음', True)))


def test_slam_alert_only_in_drive():
    assert any('SLAM 위치 없음' in a for a in alerts(state(slam_ok=False)))
    s = state()
    s['topics']['/mission/slam_ok'] = {'stale': True}
    assert any('SLAM 위치 없음' in a for a in alerts(s))
    s = state('follow', slam_ok=False)
    s['topics'] = {t: OK for t in COMMON}
    assert alerts(s) == []


def test_lap_topic_never_alerts():
    s = state()
    s['topics']['/mission/lap'] = {'stale': True}  # 바뀔 때만 발행하므로 오래돼도 정상
    assert alerts(s) == []


def test_slam_label_and_lap_text():
    assert slam_label('localization') == '지도 불러옴'
    assert slam_label('mapping') == '지도 작성 중'
    assert slam_label('off') == slam_label(None) == ''
    assert lap_text(2, 3) == '2 / 3' and lap_text(2, 0) == '2' and lap_text(None, 3) == '- / 3'


def test_map_args_and_slam_state(tmp_path):
    mm = pytest.importorskip('robot_gui.mode_manager')  # rclpy 없으면(Windows) 건너뜀
    base = str(tmp_path / 'track')
    assert mm.map_args('drive', base) == [] and mm.slam_state('drive', base) == 'mapping'
    (tmp_path / 'track.posegraph').write_text('x')
    assert mm.map_args('drive', base) == [f'map:={base}'] and mm.slam_state('drive', base) == 'localization'
    assert mm.map_args('follow', base) == [] and mm.slam_state('follow', base) == 'off'
    assert mm.map_args('drive', '') == []
