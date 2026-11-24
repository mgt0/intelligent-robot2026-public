"""Qt GUI. MainWindow는 dict 상태만 받아 그리고(ROS 비의존), GuiNode가 구독·서비스 호출을 맡는다.
수동 조종 입력은 없고 /cmd_vel은 구독만 한다. rclpy는 별도 스레드, 화면은 QTimer 20Hz로 최신 값만 읽는다."""
import collections
import json
import math
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import (QColor, QFont, QImage, QPainter, QPainterPath, QPen,
                                     QPixmap)
from python_qt_binding.QtWidgets import (QAbstractItemView, QApplication, QComboBox, QFrame,
                                         QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit,
                                         QPushButton, QScrollArea, QSizePolicy, QSplitter, QTableWidget,
                                         QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String
from std_srvs.srv import Trigger

from robot_gui.alerts import alerts, lap_text, slam_label
from robot_gui.scan_util import reduce_scan, to_xy
from robot_gui.topic_stats import TopicStats

try:
    from slam_toolbox.srv import SerializePoseGraph
    SAVE_OFF = ''
except ImportError:  # 노트북에 slam_toolbox가 없으면 [지도 저장]만 끈다
    SerializePoseGraph = None
    SAVE_OFF = 'slam_toolbox.srv를 import할 수 없어 비활성(ros-jazzy-slam-toolbox 설치 확인)'

ACCENT, BAD, INK, MUTE, RULE = '#E85D04', '#C1121F', '#222222', '#666666', '#CCCCCC'
IMAGES = {'raw': '/image_raw', 'line_debug': '/vision/line_debug',
          'sign_debug': '/vision/sign_debug', 'hand_debug': '/vision/hand_debug'}
CAM_LABEL = {'raw': '원본', 'line_debug': '라인', 'sign_debug': '표지판', 'hand_debug': '손'}
CAM_NODE = {'raw': 'camera_ros', 'line_debug': 'line_node debug:=true',
            'sign_debug': 'sign_node debug:=true', 'hand_debug': 'hand_node debug:=true'}
SERVICES = {'drive': '/mode/drive', 'follow': '/mode/follow',
            'start': '/mission/start', 'stop': '/mission/stop',
            'save_map': '/slam_toolbox/serialize_map'}
CALL_TIMEOUT = 2.0
SAVE_TIMEOUT = 10.0  # serialize_map은 지도 크기에 따라 2초를 넘길 수 있다

STYLE = f"""
QWidget {{ background:#fff; color:{INK}; font-size:13px; }}
QLabel#title {{ font-weight:bold; font-size:14px; }}
QLabel#cap {{ color:{MUTE}; font-size:12px; }}
QFrame#sec {{ border:none; border-bottom:1px solid {RULE}; }}
QLabel#cam {{ border:1px solid {RULE}; color:{MUTE}; }}
QLabel#cam[empty="true"] {{ border:1px dashed #999999; }}
QLabel#status[bad="true"] {{ color:{BAD}; }}
QPushButton {{ border:1px solid {INK}; border-radius:2px; padding:6px 12px; min-height:20px; }}
QPushButton:pressed {{ background:#eeeeee; }}
QPushButton[active="true"] {{ border:2px solid {ACCENT}; color:{ACCENT}; font-weight:bold; }}
QPushButton#start {{ background:{ACCENT}; border:1px solid {ACCENT}; color:#fff; font-weight:bold; }}
QPushButton#stop {{ border:3px solid {INK}; font-weight:bold; }}
QComboBox {{ border:1px solid {INK}; border-radius:2px; padding:3px 8px; }}
QTableWidget {{ border:none; }}
QTableWidget::item {{ border-bottom:1px solid #eeeeee; padding:0 4px; }}
QHeaderView {{ background:#fff; }}
QTableCornerButton::section {{ background:#fff; border:none; }}
QHeaderView::section {{ background:#fff; border:none; border-bottom:1px solid {RULE}; color:{MUTE}; padding:2px 4px;
                        font-size:12px; }}
QPlainTextEdit {{ border:1px solid {RULE}; }}
QScrollArea {{ border:none; }}
QSplitter::handle {{ background:{RULE}; width:1px; }}
QTabWidget::pane {{ border:none; border-top:1px solid {RULE}; }}
QTabBar::tab {{ background:#fff; border:none; padding:8px 20px; color:{MUTE}; font-size:14px; }}
QTabBar::tab:selected {{ color:{INK}; font-weight:bold; border-bottom:2px solid {ACCENT}; }}
QWidget#match QPushButton {{ font-size:20px; min-height:56px; }}
QWidget#match QPushButton#start, QWidget#match QPushButton#stop {{ font-size:24px; min-height:96px; }}
QLabel#bigstate {{ font-size:40px; font-weight:bold; }}
QLabel#alert {{ font-size:16px; }}
QLabel#alert[bad="true"] {{ color:{BAD}; font-weight:bold; }}
"""


def mono(size=None):
    f = QFont('Consolas')
    f.setStyleHint(QFont.Monospace)  # Consolas, D2Coding이 없으면 시스템 고정폭
    if hasattr(f, 'setFamilies'):
        f.setFamilies(['Consolas', 'D2Coding', 'monospace'])
    if size:
        f.setPointSize(size)
    return f


def fx(v, d):
    return '-' if v is None else f'{v:.{d}f}'


def repolish(w):
    w.style().unpolish(w)
    w.style().polish(w)


class Table(QTableWidget):
    """항목 | 값 | 단위. 값은 오른쪽 정렬 고정폭."""
    ROW_H = 22

    def __init__(self, heads=('항목', '값', '단위'), unit_col=True):
        super().__init__(0, 3)
        self.unit_col = unit_col  # False면 3열도 값처럼 오른쪽 정렬(토픽 표)
        self.setHorizontalHeaderLabels(list(heads))
        self.verticalHeader().setVisible(False)
        self.verticalHeader().setDefaultSectionSize(self.ROW_H)
        h = self.horizontalHeader()
        h.setSectionResizeMode(0, QHeaderView.Stretch)
        h.setSectionResizeMode(1, QHeaderView.Fixed)
        h.setSectionResizeMode(2, QHeaderView.Fixed)
        h.setHighlightSections(False)
        self.setColumnWidth(1, 96)
        self.setColumnWidth(2, 70 if unit_col else 110)
        for j in range(3):
            self.horizontalHeaderItem(j).setTextAlignment(
                (Qt.AlignRight if j == 1 or (j == 2 and not unit_col) else Qt.AlignLeft) | Qt.AlignVCenter)
        self.setShowGrid(False)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QAbstractItemView.NoSelection)
        self.setFocusPolicy(Qt.NoFocus)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def set_rows(self, rows):
        """rows: (이름, 값, 단위, 이름이 코드인지, 경고인지)"""
        if self.rowCount() != len(rows):
            self.setRowCount(len(rows))
            self.setFixedHeight(self.horizontalHeader().height() + 4 + self.ROW_H * len(rows))
        for i, (name, val, unit, code, bad) in enumerate(rows):
            for j, text in enumerate((name, val, unit)):
                it = self.item(i, j)
                if it is None:
                    it = QTableWidgetItem()
                    self.setItem(i, j, it)
                    it.setTextAlignment((Qt.AlignRight if j == 1 or (j == 2 and not self.unit_col) else Qt.AlignLeft)
                                        | Qt.AlignVCenter)
                    if (j == 0 and code) or j == 1 or (j == 2 and not self.unit_col):
                        it.setFont(mono())
                if it.text() != str(text):
                    it.setText(str(text))
                it.setForeground(QColor(BAD if bad else (MUTE if j == 2 and self.unit_col else INK)))


class ErrorBar(QWidget):
    """라인 error(-1~1) 막대. 중앙 기준 주황."""

    def __init__(self):
        super().__init__()
        self.value = 0.0
        self.setFixedHeight(16)

    def set_value(self, v):
        if v != self.value:
            self.value = v
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        w, h, half = self.width(), self.height(), self.width() / 2
        v = max(-1.0, min(1.0, self.value))
        p.fillRect(QRectF(half, 1, v * half, h - 2).normalized(), QColor(ACCENT))
        p.setPen(QPen(QColor(INK), 1))
        p.drawRect(0, 0, w - 1, h - 1)
        p.drawLine(int(half), 0, int(half), h)


class LidarView(QWidget):
    RANGE = 3.0  # 표시 반경(m)

    def __init__(self):
        super().__init__()
        self.xy, self.half, self.obst = [], 0.35, 0.30
        self.setMinimumSize(220, 220)

    def set_data(self, xy, half, obst):
        self.xy, self.half, self.obst = xy, half, obst
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        c = QPointF(w / 2, h / 2)
        k = (min(w, h) / 2 - 16) / self.RANGE
        p.setPen(QPen(QColor(RULE), 1))
        p.drawRect(0, 0, w - 1, h - 1)
        p.setFont(mono(7))
        n = int(self.RANGE / 0.5)
        for i in range(1, n + 1):
            r = i * 0.5 * k
            p.setPen(QPen(QColor(RULE), 1))
            p.drawEllipse(c, r, r)
            p.setPen(QColor(MUTE))
            p.drawText(QPointF(c.x() + 3, c.y() - r - 2), f'{i * 0.5:.1f}m')
        p.setPen(QPen(QColor(RULE), 1))
        p.drawLine(c, QPointF(c.x(), c.y() - self.RANGE * k))  # 전방 표시
        p.setPen(QColor(INK))
        p.setFont(QFont())
        p.drawText(QPointF(c.x() - 30, c.y() - self.RANGE * k + 12), '전방')
        # 전방 섹터와 obstacle_dist 원(주황)
        d, deg = self.obst * k, math.degrees(self.half)
        path = QPainterPath(c)
        path.arcTo(QRectF(c.x() - d, c.y() - d, 2 * d, 2 * d), 90 - deg, 2 * deg)
        path.closeSubpath()
        fill = QColor(ACCENT)
        fill.setAlpha(50)
        p.setPen(QPen(QColor(ACCENT), 1))
        p.setBrush(fill)
        p.drawPath(path)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(c, d, d)
        for s in (-1, 1):
            a = s * self.half
            p.drawLine(c, QPointF(c.x() - math.sin(a) * self.RANGE * k, c.y() - math.cos(a) * self.RANGE * k))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(INK))
        for x, y in self.xy:
            if math.hypot(x, y) <= self.RANGE:
                p.drawRect(QRectF(c.x() - y * k - 1, c.y() - x * k - 1, 2, 2))


class ImageView(QLabel):
    """크기가 바뀌면 원본 QImage를 다시 맞춘다."""

    def __init__(self):
        super().__init__()
        self._img = None

    def set_image(self, img):
        self._img = img
        self._fit()

    def _fit(self):
        if self._img is None:
            self.setPixmap(QPixmap())
        else:
            self.setPixmap(QPixmap.fromImage(self._img).scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if self._img is not None:
            self._fit()


def section(title, *widgets, grow=None):
    f = QFrame()
    f.setObjectName('sec')
    lay = QVBoxLayout(f)
    lay.setContentsMargins(12, 8, 12, 10)
    lay.setSpacing(6)
    t = QLabel(title)
    t.setObjectName('title')
    lay.addWidget(t)
    for w in widgets:
        lay.addWidget(w, 1 if w is grow else 0)
    return f


def caption(text=''):
    c = QLabel(text)
    c.setObjectName('cap')
    return c


class MainWindow(QWidget):
    """상태 dict를 받아 그린다. 콜백: cb_mode(name), cb_start(), cb_stop(), cb_cam(name)."""

    def __init__(self, cb_mode, cb_start, cb_stop, cb_cam, display_fps=10.0, cb_save=None, save_off=''):
        super().__init__()
        self.setWindowTitle('로봇 상태 화면')
        self.setMinimumSize(1024, 640)
        self.setStyleSheet(STYLE)
        self.display_fps = display_fps
        self._frame = None
        self._log = ''

        # 왼쪽: 1. 카메라, 2. 라이다
        self.combo = QComboBox()
        for k, v in CAM_LABEL.items():
            self.combo.addItem(v, k)
        self.combo.currentIndexChanged.connect(lambda _: cb_cam(self.combo.currentData()))
        self.camlabel = ImageView()
        self.camlabel.setObjectName('cam')
        self.camlabel.setAlignment(Qt.AlignCenter)
        self.camlabel.setMinimumSize(320, 200)
        self.camlabel.setWordWrap(True)
        self.camlabel.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.cap1 = caption()
        self.lidar = LidarView()
        self.cap2 = caption()
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(0)
        s1 = section('1. 카메라', self.combo, self.camlabel, self.cap1, grow=self.camlabel)
        s2 = section('2. 라이다', self.lidar, self.cap2, grow=self.lidar)
        ll.addWidget(s1, 1)
        ll.addWidget(s2, 1)

        # 오른쪽: 3. 인식, 4. 주행·모터, 5. 토픽 상태(스크롤), 6. 모드·조작(고정)
        self.bar = ErrorBar()
        self.t_vis, self.t_drv, self.t_top = Table(), Table(), Table(('토픽', '주기', '경과(s)'), unit_col=False)
        scroll_body = QWidget()
        sl = QVBoxLayout(scroll_body)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.setSpacing(0)
        sl.addWidget(section('3. 인식', self.bar, self.t_vis))
        sl.addWidget(section('4. 주행과 모터', self.t_drv))
        sl.addWidget(section('5. 토픽 상태', self.t_top))
        sl.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(scroll_body)

        self.btn_mode = {}
        mrow = QHBoxLayout()
        for key, text in (('drive', '자율주행'), ('follow', '트래킹')):
            b = QPushButton(text)
            b.clicked.connect(lambda _, k=key: cb_mode(k))
            self.btn_mode[key] = b
            mrow.addWidget(b)
        b_start, b_stop = QPushButton('출발'), QPushButton('정지')
        b_start.setObjectName('start')
        b_stop.setObjectName('stop')
        b_start.clicked.connect(lambda _: cb_start())
        b_stop.clicked.connect(lambda _: cb_stop())
        crow = QHBoxLayout()
        crow.addWidget(b_start)
        crow.addWidget(b_stop)
        self.status = QLabel('')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)
        self.logbox = QPlainTextEdit()
        self.logbox.setReadOnly(True)
        self.logbox.setFont(mono(8))
        self.logbox.setFixedHeight(80)
        self.logbox.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.btn_save = QPushButton('지도 저장')
        self.btn_save.clicked.connect(lambda _: cb_save())
        self.save_off = save_off
        self.btn_save.setEnabled(False)  # update_state가 /mission/map_file 수신 여부로 켠다
        self.cap_save = caption(save_off or '/slam_toolbox/serialize_map · 파일 경로는 mode_manager map_file')
        mrow_w, crow_w = QWidget(), QWidget()
        mrow_w.setLayout(mrow)
        crow_w.setLayout(crow)
        for r in (mrow, crow):
            r.setContentsMargins(0, 0, 0, 0)
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(0)
        rl.addWidget(scroll, 1)
        rl.addWidget(section('6. 모드와 조작', mrow_w, crow_w, self.btn_save, self.cap_save, self.status, self.logbox))

        split = QSplitter(Qt.Horizontal)
        split.addWidget(left)
        split.addWidget(right)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 1)

        tabs = QTabWidget()
        tabs.addTab(self._match_tab(cb_mode, cb_start, cb_stop), '경기')
        tabs.addTab(split, '점검')
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(tabs)

    def _match_tab(self, cb_mode, cb_start, cb_stop):
        """경기 중 볼 것만: 모드, 미션 상태, 경고, 출발·정지. 수동 조종 입력은 없다."""
        w = QWidget()
        w.setObjectName('match')
        self.btn_mode_big = {}
        mrow = QHBoxLayout()
        mrow.setContentsMargins(0, 0, 0, 0)
        for key, text in (('drive', '자율주행'), ('follow', '트래킹')):
            b = QPushButton(text)
            b.clicked.connect(lambda _, k=key: cb_mode(k))
            self.btn_mode_big[key] = b
            mrow.addWidget(b)
        mrow_w = QWidget()
        mrow_w.setLayout(mrow)
        self.bigstate = QLabel('-')
        self.bigstate.setObjectName('bigstate')
        self.bigstate.setFont(mono())
        self.bigstate.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.alert = QLabel('')
        self.alert.setObjectName('alert')
        self.alert.setWordWrap(True)
        self.alert.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        b_start, b_stop = QPushButton('출발'), QPushButton('정지')
        b_start.setObjectName('start')
        b_stop.setObjectName('stop')
        b_start.clicked.connect(lambda _: cb_start())
        b_stop.clicked.connect(lambda _: cb_stop())
        crow = QHBoxLayout()
        crow.setContentsMargins(0, 0, 0, 0)
        crow.addWidget(b_start)
        crow.addWidget(b_stop)
        crow_w = QWidget()
        crow_w.setLayout(crow)
        self.cap_state = caption()
        self.cap_slam = caption()  # 모드 옆 작은 글씨: SLAM 상태
        self.biglap = QLabel('')
        self.biglap.setObjectName('bigstate')
        self.biglap.setFont(mono())
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(section('1. 모드', mrow_w, self.cap_slam))
        lay.addWidget(section('2. 미션 상태', self.bigstate, self.cap_state, self.biglap))
        lay.addWidget(section('3. 경고', self.alert, grow=self.alert), 1)
        lay.addWidget(section('4. 출발과 정지', crow_w))
        return w

    def _set_prop(self, w, name, on):
        if w.property(name) != on:
            w.setProperty(name, on)
            repolish(w)

    def update_state(self, s):
        for k in self.btn_mode:
            for b in (self.btn_mode[k], self.btn_mode_big[k]):
                self._set_prop(b, 'active', s.get('mode') == k)
        self._status(s.get('status'))
        state = s['mission_state'] or '-'
        if self.bigstate.text() != state:
            self.bigstate.setText(state)
        self.cap_state.setText('/mission/state · 모드 ' + {'drive': '자율주행', 'follow': '트래킹'}.get(s.get('mode'), '-'))
        self.cap_slam.setText(slam_label(s.get('slam')))
        if not self.save_off:
            have = bool(s.get('map_file'))
            self.btn_save.setEnabled(have)
            self.cap_save.setText('/slam_toolbox/serialize_map · ' + (s['map_file'] if have else 'mode_manager map_file 미설정'))
        self.biglap.setVisible(s.get('mode') == 'drive')
        self.biglap.setText('바퀴 ' + lap_text(s['lap'], s['params']['laps_to_finish']))
        al = alerts(s)
        text = '\n'.join(al) if al else '없음'
        if self.alert.text() != text:
            self.alert.setText(text)
        self._set_prop(self.alert, 'bad', bool(al))
        self._camera(s)
        p = s['params']
        self.lidar.set_data(to_xy(s['scan']), p['front_half_angle'], p['obstacle_dist'])
        self.cap2.setText(f"그림 2. /scan ({len(s['scan']['angles'])}점, 눈금 0.5m)")

        ln, sg, hd = s['line'], s['sign'], s['hand']
        det, ld = sg['detail'], ln['detail']
        self.bar.set_value(ln['error'] or 0.0)
        self.t_vis.set_rows([
            ('/line/found', '-' if ln['found'] is None else ('예' if ln['found'] else '아니오'), '',
             True, ln['found'] is False),
            ('/line/error', fx(ln['error'], 3), '', True, False),
            ('표지판(확정)', sg['last'] or '-', '', False, False),
            ('표지판 경과', fx(sg['age'], 1), 's', False, False),
            ('후보 클래스', det['cls'] if det else '-', '', False, False),
            ('후보 면적', str(det['area']) if det else '-', 'px', False, False),
            ('후보 cx', fx(det['cx'], 2) if det else '-', '', False, False),
            ('/line/detail 줄 수', '-' if ld is None else str(ld.get('n')), '', True, False),
            ('줄 L', fx(ld and ld.get('L'), 2), '', False, False),
            ('줄 C', fx(ld and ld.get('C'), 2), '', False, False),
            ('줄 R', fx(ld and ld.get('R'), 2), '', False, False),
            ('/hand/backend', s['hand_backend'] or '-', '', True, False),
            ('손 x', fx(hd['x'], 2) if hd else '-', '', False, False),
            ('손 y', fx(hd['y'], 2) if hd else '-', '', False, False),
            ('손 z', fx(hd['z'], 2) if hd else '-', '', False, False)])
        v, w, o = s['cmd_vel'] or {}, s['wheel'] or [], s['odom'] or {}
        self.t_drv.set_rows([
            ('/mission/state', s['mission_state'] or '-', '', True, False),
            ('/cmd_vel v', fx(v.get('v'), 3), 'm/s', True, False),
            ('/cmd_vel ω', fx(v.get('w'), 3), 'rad/s', True, False),
            ('바퀴 좌', fx(w[0] if w else None, 2), 'rad/s', False, False),
            ('바퀴 우', fx(w[1] if len(w) > 1 else None, 2), 'rad/s', False, False),
            ('odom x', fx(o.get('x'), 3), 'm', True, False),
            ('odom y', fx(o.get('y'), 3), 'm', True, False),
            ('odom yaw', fx(o.get('yaw'), 3), 'rad', True, False)])
        self.t_top.set_rows([
            (n, f"{t['hz']:.1f} Hz", ('-' if t['age'] is None else f"{t['age']:.2f}")
             + (' STALE' if t['stale'] else ''), True, t['stale'])
            for n, t in s['topics'].items()])
        text = '\n'.join(s['log'])
        if text != self._log:
            self._log = text
            self.logbox.setPlainText(text)
            self.logbox.verticalScrollBar().setValue(self.logbox.verticalScrollBar().maximum())

    def _status(self, st):
        text, bad = st or ('', False)
        if self.status.text() != text:
            self.status.setText(text)
        self._set_prop(self.status, 'bad', bool(bad))

    def _camera(self, s):
        name, frame = s['cam'], s['frame']
        topic = s['cam_topic']
        self.cap1.setText(f'그림 1. {topic} ({self.display_fps:g}fps)')
        empty = frame is None
        self._set_prop(self.camlabel, 'empty', empty)
        if empty:
            self._frame = None
            self.camlabel.set_image(None)
            self.camlabel.setText(f'영상 없음 — {CAM_LABEL[name]} 영상: {CAM_NODE[name]}와 {topic} 확인')
        elif frame is not self._frame:
            self._frame = frame
            hgt, wid = frame.shape[:2]
            img = QImage(frame.data, wid, hgt, 3 * wid, QImage.Format_BGR888).copy()
            self.camlabel.setText('')
            self.camlabel.set_image(img)


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui')
        d = self.declare_parameter
        d('display_fps', 10.0)
        d('stale_sec', 1.0)
        d('log_lines', 40)
        d('front_half_angle', 0.35)    # mission.yaml과 같은 값(표시용)
        d('obstacle_dist', 0.30)
        d('laps_to_finish', 0)         # 0이면 바퀴 수 숫자만
        # compressed는 Pi에서 토픽마다 republish가 떠 있어야 동작한다(없으면 영상 끊김):
        #   ros2 run image_transport republish raw compressed --ros-args -r in:=<topic> -r out/compressed:=<topic>/compressed
        # 확인 필요: republish 인자 문법은 Jazzy 기준 실기 확인
        d('image_transport', 'raw')    # raw 또는 compressed
        self.g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stats = TopicStats()
        self.msg = {}
        self.topics = []
        self.bridge = CvBridge()
        self.log = collections.deque(maxlen=self.g('log_lines'))
        self.mode = None
        self.cam = 'raw'
        self.cam_sub = None
        self.frame = self.frame_msg = None
        self.frame_t = 0.0
        self.pending = []
        self.status = ('', False)

        for typ, topic, qos in (
                (Float32, '/line/error', 10), (Bool, '/line/found', 10), (String, '/sign', 10),
                (String, '/sign/detail', 10), (String, '/line/detail', 10), (PointStamped, '/hand', 10),
                (String, '/mission/state', 10), (Bool, '/mission/slam_ok', 10), (Twist, '/cmd_vel', 10),
                (Float32MultiArray, '/wheel/velocity', 10), (Odometry, '/odom', 10),
                (LaserScan, '/scan', qos_profile_sensor_data)):
            self.topics.append(topic)
            self._sub(typ, topic, qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/mission/mode', self._on_mode, latched)
        # 바뀔 때만 발행하는 토픽이라 stats(끊김 판정)에 넣지 않는다
        for typ, topic in ((Int32, '/mission/lap'), (String, '/mission/slam'), (String, '/hand/backend'),
                           (String, '/mission/map_file')):
            self.create_subscription(typ, topic, lambda m, t=topic: self.msg.__setitem__(t, m), latched)
        self.create_subscription(String, '/mission/log', lambda m: self.log.append(m.data), 10)
        self.cli = {k: self.create_client(SerializePoseGraph if k == 'save_map' else Trigger, v)
                    for k, v in SERVICES.items() if k != 'save_map' or SerializePoseGraph}
        self.set_camera('raw')

    def _sub(self, typ, topic, qos):
        def cb(m):
            self.msg[topic] = m
            self.stats.mark(topic)
        return self.create_subscription(typ, topic, cb, qos)

    def _on_mode(self, m):
        self.mode = m.data

    def set_camera(self, name):
        """선택한 영상 토픽만 구독한다(Wi-Fi 대역폭 절약)."""
        if self.cam_sub is not None:
            self.destroy_subscription(self.cam_sub)
        self.cam = name
        self.frame = self.frame_msg = None
        if self.g('image_transport') == 'compressed':
            self.cam_sub = self._sub(CompressedImage, self.cam_topic(), qos_profile_sensor_data)
        else:
            self.cam_sub = self._sub(Image, self.cam_topic(), qos_profile_sensor_data)

    def cam_topic(self):
        return IMAGES[self.cam] + ('/compressed' if self.g('image_transport') == 'compressed' else '')

    def call(self, key):
        if key == 'save_map' and SerializePoseGraph is None:
            return
        cli = self.cli[key]
        if not cli.service_is_ready():
            self.status = (f'{SERVICES[key]} 서비스 없음(같은 ROS_DOMAIN_ID인지, Pi 노드가 떠 있는지 확인)', True)
            return
        if key == 'save_map':
            req = SerializePoseGraph.Request()
            req.filename = self._val('/mission/map_file', 'data', '')  # 확인 필요: 요청 필드명 filename
            if not req.filename:
                return
        else:
            req = Trigger.Request()
        self.pending.append((key, cli.call_async(req), time.monotonic() + (SAVE_TIMEOUT if key == 'save_map' else CALL_TIMEOUT)))
        self.status = (f'{SERVICES[key]} 호출 중', False)

    def _poll_calls(self):
        keep = []
        for key, fut, deadline in self.pending:
            if fut.done():
                try:
                    r = fut.result()
                    if key == 'save_map':  # 확인 필요: 응답 필드 result(int8, 0=성공)
                        ok = r.result == 0
                        r = Trigger.Response(success=ok, message='지도 저장됨' if ok else f'지도 저장 실패(result={r.result})')
                    self.status = (f'{SERVICES[key]}: {r.message or ("성공" if r.success else "실패")}',
                                   not r.success)
                except Exception as e:  # noqa: BLE001
                    self.status = (f'{SERVICES[key]} 실패: {e}', True)
            elif time.monotonic() > deadline:
                self.cli[key].remove_pending_request(fut)
                self.status = (f'{SERVICES[key]} 실패: {SAVE_TIMEOUT if key == "save_map" else CALL_TIMEOUT:g}초 안에 응답 없음', True)
            else:
                keep.append((key, fut, deadline))
        self.pending = keep

    def _val(self, topic, attr, default=None):
        m = self.msg.get(topic)
        return getattr(m, attr) if m is not None else default

    def snapshot(self):
        """UI 스레드에서 20Hz로 호출. 최신 값만 dict로 묶는다."""
        self._poll_calls()
        sec = self.g('stale_sec')
        cam_topic = self.cam_topic()
        snap = self.stats.snapshot(self.topics + [cam_topic], sec)
        now = time.monotonic()
        m = self.msg.get(cam_topic)
        if m is None or snap[cam_topic]['stale']:
            self.frame = self.frame_msg = None
        elif m is not self.frame_msg and now - self.frame_t >= 1.0 / self.g('display_fps'):
            try:
                if isinstance(m, CompressedImage):
                    self.frame = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
                else:
                    self.frame = self.bridge.imgmsg_to_cv2(m, 'bgr8')
                self.frame_msg, self.frame_t = m, now
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f'영상 변환 실패: {e}', throttle_duration_sec=5.0)
        hand = self.msg.get('/hand')
        odom, tw = self.msg.get('/odom'), self.msg.get('/cmd_vel')
        yaw = None
        if odom:
            q = odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            detail = json.loads(self._val('/sign/detail', 'data', 'null'))
        except ValueError:
            detail = None
        try:
            ldet = json.loads(self._val('/line/detail', 'data', 'null')) if not snap['/line/detail']['stale'] else None
        except ValueError:
            ldet = None
        sc = self.msg.get('/scan')
        wv = self._val('/wheel/velocity', 'data')
        return {
            'mode': self.mode, 'status': self.status, 'cam': self.cam, 'frame': self.frame,
            'cam_topic': cam_topic, 'slam': self._val('/mission/slam', 'data'),
            'slam_ok': self._val('/mission/slam_ok', 'data'), 'map_file': self._val('/mission/map_file', 'data', ''),
            'lap': self._val('/mission/lap', 'data'),
            'hand_backend': self._val('/hand/backend', 'data'),
            'params': {'front_half_angle': self.g('front_half_angle'),
                       'obstacle_dist': self.g('obstacle_dist'),
                       'laps_to_finish': self.g('laps_to_finish')},
            'scan': reduce_scan(sc.ranges, sc.angle_min, sc.angle_increment, 360, sc.range_max)
            if sc is not None and not snap['/scan']['stale'] else {'angles': [], 'ranges': []},
            'line': {'found': self._val('/line/found', 'data'), 'error': self._val('/line/error', 'data'),
                     'detail': ldet},
            'sign': {'last': self._val('/sign', 'data'), 'age': snap['/sign']['age'], 'detail': detail},
            'hand': {'x': hand.point.x, 'y': hand.point.y, 'z': hand.point.z}
            if hand is not None and not snap['/hand']['stale'] else None,
            'mission_state': self._val('/mission/state', 'data'),
            'cmd_vel': {'v': tw.linear.x, 'w': tw.angular.z} if tw else None,
            'wheel': list(wv) if wv is not None else None,
            'odom': {'x': odom.pose.pose.position.x, 'y': odom.pose.pose.position.y, 'yaw': yaw}
            if odom else None,
            'topics': snap, 'log': list(self.log),
        }


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = QApplication(sys.argv)
    win = MainWindow(lambda k: node.call(k), lambda: node.call('start'), lambda: node.call('stop'),
                     node.set_camera, node.g('display_fps'),
                     lambda: node.call('save_map'), SAVE_OFF)
    win.show()
    timer = QTimer()
    timer.timeout.connect(lambda: win.update_state(node.snapshot()))
    timer.start(50)  # 20Hz
    code = app.exec_()
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
