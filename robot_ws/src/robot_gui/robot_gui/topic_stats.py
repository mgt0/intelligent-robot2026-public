"""토픽별 수신 시각으로 Hz와 마지막 수신 후 경과 시간을 계산한다."""
import time
from collections import deque


class TopicStats:
    def __init__(self, window=20, clock=time.monotonic):
        self._window = window
        self._clock = clock
        self._ts = {}

    def mark(self, name, now=None):
        now = self._clock() if now is None else now
        self._ts.setdefault(name, deque(maxlen=self._window)).append(now)

    def snapshot(self, names, stale_sec, now=None):
        """{name: {hz, age, stale}}. 한 번도 못 받았으면 age None, stale True."""
        now = self._clock() if now is None else now
        out = {}
        for n in names:
            ts = self._ts.get(n)
            if not ts:
                out[n] = {'hz': 0.0, 'age': None, 'stale': True}
                continue
            age = now - ts[-1]
            span = ts[-1] - ts[0]
            stale = age > stale_sec
            hz = 0.0 if stale or len(ts) < 2 or span <= 0 else (len(ts) - 1) / span
            out[n] = {'hz': round(hz, 1), 'age': round(age, 2), 'stale': stale}
        return out
