from robot_gui.topic_stats import TopicStats


def test_hz_and_age():
    s = TopicStats()
    for i in range(11):
        s.mark('/a', now=i * 0.1)
    r = s.snapshot(['/a'], stale_sec=1.0, now=1.2)['/a']
    assert r['hz'] == 10.0 and r['age'] == 0.2 and not r['stale']


def test_stale_and_never_received():
    s = TopicStats()
    s.mark('/a', now=0.0)
    s.mark('/a', now=0.1)
    snap = s.snapshot(['/a', '/b'], stale_sec=1.0, now=5.0)
    assert snap['/a']['stale'] and snap['/a']['hz'] == 0.0
    assert snap['/b'] == {'hz': 0.0, 'age': None, 'stale': True}
