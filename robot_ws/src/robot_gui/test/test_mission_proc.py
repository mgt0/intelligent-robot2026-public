import os
import stat
import sys
import time

import pytest

from robot_gui.mission_proc import MissionProc

pytestmark = pytest.mark.skipif(not hasattr(os, 'killpg'), reason='Linux 전용(os.killpg 없음)')


def fake_ros2(tmp_path, body):
    p = tmp_path / 'ros2'
    p.write_text(f'#!{sys.executable}\nimport signal, sys, time\n{body}\n')
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return str(p)


def wait_for(cond, t=5.0):
    end = time.time() + t
    while time.time() < end and not cond():
        time.sleep(0.05)
    return cond()


def test_log_ring_and_sigint_stop(tmp_path):
    cmd = fake_ros2(tmp_path, 'for i in range(10): print(i, flush=True)\ntime.sleep(60)')
    mp = MissionProc(cmd, log_lines=3)
    mp.start('drive')
    assert wait_for(lambda: mp.state()['log'][-1:] == ['9'])
    assert mp.state()['log'] == ['7', '8', '9'] and mp.state()['running']
    mp.stop()
    st = mp.state()
    assert not st['running']


def test_escalates_to_sigkill(tmp_path):
    cmd = fake_ros2(tmp_path, 'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                              'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
                              'print("ready", flush=True)\ntime.sleep(60)')
    mp = MissionProc(cmd, int_wait=0.3, term_wait=0.3)
    mp.start('follow')
    assert wait_for(lambda: mp.state()['log'] == ['ready'])
    t0 = time.time()
    mp.stop()
    assert time.time() - t0 < 5 and not mp.state()['running']
