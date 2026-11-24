"""mission_stack launch를 하위 프로세스(새 프로세스 그룹)로 실행하고 종료한다. Linux 전용."""
import collections
import os
import signal
import subprocess
import threading


class MissionProc:
    def __init__(self, ros2_cmd='ros2', log_lines=40, int_wait=5.0, term_wait=2.0, on_line=None):
        self._on_line = on_line  # 출력 한 줄마다 호출(예: 토픽 발행)
        self._cmd = ros2_cmd
        self._int_wait = int_wait
        self._term_wait = term_wait
        self._log = collections.deque(maxlen=log_lines)
        self._lock = threading.Lock()
        self._proc = None
        self._mode = ''
        self._reader = None

    def start(self, mode, extra=()):
        with self._lock:
            self._stop_locked()
            self._log.clear()
            self._mode = mode
            self._proc = subprocess.Popen(
                [self._cmd, 'launch', 'robot_bringup', 'mission_stack.launch.py',
                 f'mode:={mode}', 'debug:=true', *extra],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                errors='replace', start_new_session=True)
            self._reader = threading.Thread(target=self._read, args=(self._proc,), daemon=True)
            self._reader.start()

    def stop(self):
        with self._lock:
            self._stop_locked()
            self._mode = ''

    def state(self):
        p = self._proc
        rc = p.poll() if p else None
        return {'mode': self._mode, 'running': p is not None and rc is None,
                'returncode': rc, 'log': list(self._log)}

    def _read(self, proc):
        for line in proc.stdout:
            self._log.append(line.rstrip())
            if self._on_line:
                self._on_line(line.rstrip())

    def _stop_locked(self):
        p = self._proc
        if p is None:
            return
        # SIGINT -> 대기 -> SIGTERM -> 대기 -> SIGKILL. 세션 리더라 pgid == pid
        for sig, wait in ((signal.SIGINT, self._int_wait), (signal.SIGTERM, self._term_wait),
                          (signal.SIGKILL, None)):
            if p.poll() is not None:
                break
            try:
                os.killpg(p.pid, sig)
                p.wait(wait)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                pass
        try:
            os.killpg(p.pid, signal.SIGKILL)  # 리더가 먼저 죽고 남은 자식 정리
        except ProcessLookupError:
            pass
        p.wait()
        self._reader.join(2.0)
        p.stdout.close()
        self._proc = None
