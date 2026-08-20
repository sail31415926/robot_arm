#!/usr/bin/env python3
"""
@file   motor_debug_gui.py
@brief  RB200-CA 单电机调试平台——PyQt5 前端 v2（仪表盘式布局，只做界面，不碰 CAN）

v2 重设计（2026-08-17 用户拍板）：左右分栏仪表盘式
  - 顶栏：状态灯 + 402 状态大字 + 模式/故障 + 常驻急停
  - 左栏：大字体实时读数（位置/速度/转矩）+ 伺服按钮 + 工具入口（置零/限制/SDO 弹窗）
  - 右栏：模式分段按钮（PP/PV/PT）→ 参数区随模式切换，执行/停止按钮位置固定;
          PP 带 ±步进点动、PV 带按住即动松开即停的方向点动;下方实时曲线
  - 底部：日志（紧凑）
  - 未使能时运动区自动置灰;后端断线时除日志外全部置灰（无后端功能置灰原则）

与 motor_debug_backend（C++ 总线后端）配对，由 motor_debug.launch.py 一起拉起。
所有操作经后端 ROS 服务/话题完成;单位换算权威在后端（motor_unit_converter.hpp），
本 GUI 启动时经参数服务取 counts_per_rad，自己不带换算常数。

用法（正式入口是 launch，一个 launch 拉一个电机）：
  ros2 launch robot_arm_driver motor_debug.launch.py node_id:=1
  ros2 run robot_arm_driver motor_debug_gui.py [--ros-args -p backend:=motor_debug_backend_1]

⚠️ 后端独占总线：先停 ros2_control 栈。
⚠️ 力矩模式下「0」不是「停」，是自由下垂；失能/急停会让重力关节下沉。

@date 2026-08-17
@copyright Copyright (c) 2026 EMEET
"""

import math
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node

from canopen_interfaces.srv import COReadID, COTargetDouble, COWriteID
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt16MultiArray
from std_srvs.srv import Trigger

from PyQt5.QtCore import QObject, Qt, QTime, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDialog, QDoubleSpinBox,
    QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMessageBox, QPlainTextEdit, QPushButton, QSlider, QSpinBox, QStackedWidget,
    QVBoxLayout, QWidget,
)

# 限制参数表（与后端/set_motor_limits 同一套对象；kind 决定换算与符号）
LIMIT_ROWS = [
    ('607F 最大轮廓速度',  0x607F, 0, 'vel',    'rad/s'),
    ('6081 PP 轮廓速度',   0x6081, 0, 'vel',    'rad/s'),
    ('6083 轮廓加速度',    0x6083, 0, 'vel',    'rad/s²'),
    ('6084 轮廓减速度',    0x6084, 0, 'vel',    'rad/s²'),
    ('6085 快停减速度',    0x6085, 0, 'vel',    'rad/s²'),
    ('607D:01 软限位下限', 0x607D, 1, 'pos',    'rad'),
    ('607D:02 软限位上限', 0x607D, 2, 'pos',    'rad'),
    ('6072 最大转矩',      0x6072, 0, 'torque', '%'),
]
MODE_NAMES = {0: '无', 1: 'PP', 3: 'PV', 4: 'PT', 6: 'HM', 7: 'IP'}

GREEN, RED, GRAY, AMBER = '#2e7d32', '#c62828', '#616161', '#e65100'
SERIES = (('位置 rad', '#1976d2'), ('速度 rad/s', '#f57c00'),
          ('转矩 %', '#7b1fa2'), ('电流 A', '#00838f'))


def decode_402(sw: int) -> str:
    if (sw & 0x004F) == 0x0000:
        return 'Not ready'
    if (sw & 0x004F) == 0x0040:
        return 'Switch on disabled'
    if (sw & 0x006F) == 0x0021:
        return 'Ready to switch on'
    if (sw & 0x006F) == 0x0023:
        return 'Switched on'
    if (sw & 0x006F) == 0x0027:
        return 'Operation enabled'
    if (sw & 0x006F) == 0x0007:
        return 'Quick stop active'
    if (sw & 0x004F) == 0x000F:
        return 'Fault reaction'
    if (sw & 0x004F) == 0x0008:
        return 'Fault'
    return '?'


def to_i32(u: int) -> int:
    return u - 0x100000000 if u >= 0x80000000 else u


class RosSignals(QObject):
    state = pyqtSignal(float, float, float)          # pos rad / vel rad/s / torque %
    drive_status = pyqtSignal(int, int, int, float)  # statusword / mode / fault / 电流 A
    log = pyqtSignal(str)
    limit_value = pyqtSignal(int, float, int)        # 行号 / 换算值 / 原始 pp
    sdo_value = pyqtSignal(int)                      # SDO 读结果（有符号）


class GuiNode(Node):
    """rclpy 侧：订阅后端话题、持有服务客户端。回调都在 spin 线程，只发 Qt 信号。"""

    def __init__(self, signals: RosSignals):
        super().__init__('motor_debug_gui')
        self.sig = signals
        backend = self.declare_parameter('backend', 'motor_debug_backend').value
        ns = f'/{backend}'

        self.create_subscription(JointState, f'{ns}/state', self._on_state, 10)
        self.create_subscription(UInt16MultiArray, f'{ns}/drive_status', self._on_status, 10)
        self.create_subscription(String, f'{ns}/log', lambda m: self.sig.log.emit(m.data), 10)

        self.triggers = {
            name: self.create_client(Trigger, f'{ns}/{name}')
            for name in ('enable', 'disable', 'fault_reset', 'nmt_reset', 'halt', 'estop',
                         'zero_target', 'zero_calibrate', 'save_eeprom')
        }
        self.move_pp = self.create_client(COTargetDouble, f'{ns}/move_pp')
        self.run_pv = self.create_client(COTargetDouble, f'{ns}/run_pv')
        self.run_pt = self.create_client(COTargetDouble, f'{ns}/run_pt')
        self.sdo_read = self.create_client(COReadID, f'{ns}/sdo_read')
        self.sdo_write = self.create_client(COWriteID, f'{ns}/sdo_write')
        self.get_params = self.create_client(GetParameters, f'{ns}/get_parameters')
        self.set_params = self.create_client(SetParameters, f'{ns}/set_parameters')

        self.counts_per_rad = None
        self.rated_current_a = 0.0
        self.rated_torque_nm = 0.0
        self.backend_desc = backend
        self._fetch_backend_info()

    def _fetch_backend_info(self):
        if not self.get_params.wait_for_service(timeout_sec=0.0):
            self._retry = self.create_timer(1.0, self._retry_fetch)
            return
        rq = GetParameters.Request(
            names=['counts_per_rad', 'can_interface', 'node_id',
                   'rated_current_a', 'rated_torque_nm'])
        self.get_params.call_async(rq).add_done_callback(self._on_backend_info)

    def _retry_fetch(self):
        self._retry.cancel()
        self._fetch_backend_info()

    def _on_backend_info(self, fut):
        try:
            vals = fut.result().values
            self.counts_per_rad = vals[0].double_value
            self.backend_desc = f'{vals[1].string_value} 节点 {vals[2].integer_value}'
            self.rated_current_a = vals[3].double_value if len(vals) > 3 else 0.0
            self.rated_torque_nm = vals[4].double_value if len(vals) > 4 else 0.0
            self.sig.log.emit(f'[ OK ] 已连接后端（{self.backend_desc}，'
                              f'counts_per_rad={self.counts_per_rad:.3f}）')
        except Exception as e:  # noqa: BLE001
            self.sig.log.emit(f'[FAIL] 取后端参数失败：{e}')

    def _on_state(self, msg: JointState):
        if msg.position:
            self.sig.state.emit(msg.position[0],
                                msg.velocity[0] if msg.velocity else 0.0,
                                msg.effort[0] if msg.effort else 0.0)

    def _on_status(self, msg: UInt16MultiArray):
        if len(msg.data) >= 3:
            cur = 0.0
            if len(msg.data) >= 4:      # [3] 电流 0.01A，int16 按位打包
                raw = msg.data[3]
                cur = (raw - 0x10000 if raw >= 0x8000 else raw) / 100.0
            self.sig.drive_status.emit(msg.data[0], msg.data[1], msg.data[2], cur)

    # ── 服务调用（全部异步；Trigger 的结果后端会发 ~/log）─────────────────────
    def call_trigger(self, name: str):
        cli = self.triggers[name]
        if not cli.service_is_ready():
            self.sig.log.emit(f'[FAIL] {name}：后端服务不在线')
            return
        cli.call_async(Trigger.Request())

    def call_target(self, cli, value: float, what: str):
        if not cli.service_is_ready():
            self.sig.log.emit(f'[FAIL] {what}：后端服务不在线')
            return
        cli.call_async(COTargetDouble.Request(target=float(value)))

    def set_pp_profile(self, vel: float, acc: float):
        if not self.set_params.service_is_ready():
            return
        def p(name, v):
            return Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
        self.set_params.call_async(SetParameters.Request(
            parameters=[p('pp_profile_velocity', vel), p('pp_profile_accel', acc)]))

    def read_object(self, index: int, sub: int, size: int, on_value):
        if not self.sdo_read.service_is_ready():
            self.sig.log.emit('[FAIL] SDO 读：后端服务不在线')
            return
        rq = COReadID.Request(index=index, subindex=sub,
                              canopen_datatype={1: 8, 2: 16, 4: 32}[size])
        def done(fut):
            try:
                rs = fut.result()
                if rs.success:
                    on_value(to_i32(rs.data), rs.data)
            except Exception as e:  # noqa: BLE001
                self.sig.log.emit(f'[FAIL] SDO 读回调：{e}')
        self.sdo_read.call_async(rq).add_done_callback(done)

    def write_object(self, index: int, sub: int, value: int, size: int):
        if not self.sdo_write.service_is_ready():
            self.sig.log.emit('[FAIL] SDO 写：后端服务不在线')
            return
        self.sdo_write.call_async(COWriteID.Request(
            index=index, subindex=sub, data=value & 0xFFFFFFFF,
            canopen_datatype={1: 8, 2: 16, 4: 32}[size]))


# ═══ 实时曲线（无第三方依赖的轻量示波器）═══════════════════════════════════════
class ScopeWidget(QWidget):
    """30s 滚动窗口，位置/速度/转矩三条曲线各自独立归一化（看趋势用）。"""

    WINDOW_S = 30.0

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(140)
        self.samples = deque()          # (t, pos, vel, tq, cur)
        self.enabled = [True] * len(SERIES)

    def add_sample(self, pos, vel, tq, cur):
        now = time.monotonic()
        self.samples.append((now, pos, vel, tq, cur))
        while self.samples and now - self.samples[0][0] > self.WINDOW_S:
            self.samples.popleft()
        self.update()

    def set_series_enabled(self, i, on):
        self.enabled[i] = on
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#fafafa'))
        p.setPen(QPen(QColor('#e0e0e0')))
        for frac in (0.25, 0.5, 0.75):
            y = int(self.height() * frac)
            p.drawLine(0, y, self.width(), y)
        if len(self.samples) < 2:
            p.setPen(QPen(QColor(GRAY)))
            p.drawText(self.rect(), Qt.AlignCenter, '等待数据…')
            return
        t1 = self.samples[-1][0]
        t0 = t1 - self.WINDOW_S
        w, h, pad = self.width(), self.height(), 6
        for si, (label, color) in enumerate(SERIES):
            if not self.enabled[si]:
                continue
            vals = [s[si + 1] for s in self.samples]
            lo, hi = min(vals), max(vals)
            span = (hi - lo) or 1.0
            pts = [
                (int((s[0] - t0) / self.WINDOW_S * w),
                 int(h - pad - (s[si + 1] - lo) / span * (h - 2 * pad)))
                for s in self.samples
            ]
            p.setPen(QPen(QColor(color), 2))
            for a, b in zip(pts, pts[1:]):
                p.drawLine(a[0], a[1], b[0], b[1])
            p.drawText(8 + si * 145, 16, f'{label.split()[0]} {vals[-1]:+.3f}')


# ═══ 工具弹窗：限制参数 / SDO（从主界面移出，保持主线干净）═══════════════════════
class LimitsDialog(QDialog):
    def __init__(self, parent, node: GuiNode):
        super().__init__(parent)
        self.node = node
        self.setWindowTitle('限制参数（限速 / 软限位 / 最大转矩）')
        g = QGridLayout(self)
        for col, text in enumerate(('对象', '当前值', '新值（留空不写）')):
            g.addWidget(QLabel(f'<b>{text}</b>'), 0, col)
        self.cur, self.new = [], []
        for i, (label, _idx, _sub, _kind, unit) in enumerate(LIMIT_ROWS):
            g.addWidget(QLabel(label), i + 1, 0)
            cur = QLineEdit()
            cur.setReadOnly(True)
            cur.setMinimumWidth(180)
            cur.setStyleSheet('font-family:monospace;')
            g.addWidget(cur, i + 1, 1)
            new = QLineEdit()
            new.setPlaceholderText(unit)
            g.addWidget(new, i + 1, 2)
            self.cur.append(cur)
            self.new.append(new)
        h = QHBoxLayout()
        for text, fn in (('读取全部', self.read_all), ('写入非空项', self.write_all),
                         ('固化 EEPROM', lambda: node.call_trigger('save_eeprom'))):
            b = QPushButton(text)
            b.clicked.connect(fn)
            h.addWidget(b)
        h.addStretch()
        g.addLayout(h, len(LIMIT_ROWS) + 1, 0, 1, 3)
        note = QLabel('注意：6081/6083/6084/6085 每次起栈会被 bus.yml SDO 段重写，长期生效要把\n'
                      '日志里的 pp 值回填 bus.yml；软限位 607D 用这里 + 固化即为正式配置。')
        note.setStyleSheet(f'color:{GRAY};')
        g.addWidget(note, len(LIMIT_ROWS) + 2, 0, 1, 3)
        node.sig.limit_value.connect(self._on_value)
        self.read_all()

    def read_all(self):
        for i, (_label, idx, sub, kind, _unit) in enumerate(LIMIT_ROWS):
            size = 2 if kind == 'torque' else 4
            def on_value(signed, raw, row=i, k=kind):
                if k == 'torque':
                    val = (raw & 0xFFFF) / 10.0
                elif self.node.counts_per_rad:
                    val = signed / self.node.counts_per_rad
                else:
                    val = float('nan')
                self.node.sig.limit_value.emit(row, val, signed)
            self.node.read_object(idx, sub, size, on_value)

    def _on_value(self, row, val, raw_pp):
        unit = LIMIT_ROWS[row][4]
        if math.isnan(val):
            self.cur[row].setText(f'{raw_pp} pp（等 counts_per_rad）')
        elif LIMIT_ROWS[row][3] == 'torque':
            self.cur[row].setText(f'{val:.1f} {unit}')
        else:
            self.cur[row].setText(f'{val:.4f} {unit}  ({raw_pp} pp)')

    def write_all(self):
        scale = self.node.counts_per_rad
        for i, (label, idx, sub, kind, _unit) in enumerate(LIMIT_ROWS):
            text = self.new[i].text().strip()
            if not text:
                continue
            try:
                v = float(text)
            except ValueError:
                self.node.sig.log.emit(f'[FAIL] {label}：「{text}」不是数字')
                continue
            if kind == 'torque':
                self.node.write_object(idx, sub, int(v * 10.0 + 0.5), 2)
            elif scale is None:
                self.node.sig.log.emit('[FAIL] 换算系数未就绪（counts_per_rad 还没取到）')
                return
            elif kind == 'vel' and v < 0:
                self.node.sig.log.emit(f'[FAIL] {label}：速度/加速度类不能为负')
            else:
                self.node.write_object(idx, sub, int(round(v * scale)), 4)
        self.read_all()


class SdoDialog(QDialog):
    def __init__(self, parent, node: GuiNode):
        super().__init__(parent)
        self.node = node
        self.setWindowTitle('SDO 裸读写')
        f = QFormLayout(self)
        self.idx = QLineEdit('603F')
        self.idx.setPlaceholderText('十六进制,如 6064')
        self.sub = QSpinBox()
        self.sub.setRange(0, 255)
        self.size = QComboBox()
        self.size.addItems(['1 字节', '2 字节', '4 字节'])
        self.size.setCurrentIndex(2)
        self.val = QLineEdit()
        self.val.setPlaceholderText('十进制或 0x 十六进制')
        f.addRow('索引 (hex)', self.idx)
        f.addRow('子索引', self.sub)
        f.addRow('长度', self.size)
        f.addRow('值', self.val)
        h = QHBoxLayout()
        rd = QPushButton('读')
        rd.clicked.connect(self._read)
        wr = QPushButton('写')
        wr.clicked.connect(self._write)
        h.addWidget(rd)
        h.addWidget(wr)
        h.addStretch()
        f.addRow(h)
        hint = QLabel('常用：603F 故障码 | 6064 位置 | 606C 速度 | 6077 转矩 | 6041 状态字 | 6061 当前模式')
        hint.setStyleSheet(f'color:{GRAY};')
        f.addRow(hint)
        node.sig.sdo_value.connect(lambda v: self.val.setText(str(v)))

    def _parse_index(self):
        try:
            return int(self.idx.text().strip(), 16)
        except ValueError:
            self.node.sig.log.emit('[FAIL] 索引不是合法十六进制')
            return None

    def _read(self):
        idx = self._parse_index()
        if idx is None:
            return
        size = 1 << self.size.currentIndex()
        self.node.read_object(idx, self.sub.value(), size,
                              lambda signed, _raw: self.node.sig.sdo_value.emit(signed))

    def _write(self):
        idx = self._parse_index()
        if idx is None:
            return
        text = self.val.text().strip()
        try:
            value = int(text, 16) if text.lower().startswith('0x') else int(text)
        except ValueError:
            self.node.sig.log.emit('[FAIL] 值格式不对（十进制或 0x 十六进制）')
            return
        self.node.write_object(idx, self.sub.value(), value, 1 << self.size.currentIndex())


# ═══ 主窗口（仪表盘式）═══════════════════════════════════════════════════════
class MainWindow(QWidget):
    def __init__(self, node: GuiNode, signals: RosSignals):
        super().__init__()
        self.node = node
        self.cur_pos = 0.0
        self.cur_amps = 0.0
        self.enabled402 = False
        self.last_status_t = 0.0
        self.setWindowTitle('RB200-CA 单电机调试平台')
        self._build_ui()
        signals.state.connect(self._on_state)
        signals.drive_status.connect(self._on_drive_status)
        signals.log.connect(self._append_log)
        # 后端断线看门狗：2s 没有 drive_status 就整体置灰
        self._watchdog = QTimer(self)
        self._watchdog.timeout.connect(self._check_alive)
        self._watchdog.start(1000)
        self._set_online(False)

    # ── 布局 ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # 顶栏：状态灯 + 402 状态 + 模式/故障 + 急停
        bar = QHBoxLayout()
        self.led = QLabel('●')
        self.led.setStyleSheet(f'color:{GRAY}; font-size:22pt;')
        self.state_lbl = QLabel('等待后端…')
        self.state_lbl.setStyleSheet(f'font-weight:bold; font-size:16pt; color:{GRAY};')
        self.mode_lbl = QLabel('模式 --')
        self.mode_lbl.setStyleSheet('font-size:12pt;')
        self.fault_lbl = QLabel('')
        estop = QPushButton('急 停')
        estop.setMinimumHeight(52)
        estop.setStyleSheet(f'background:{RED}; color:white; font-weight:bold;'
                            'font-size:17pt; padding:6px 30px; border-radius:6px;')
        estop.setToolTip('清零 PV/PT 目标 + Shutdown 掉力矩。⚠️ 重力关节会下沉')
        estop.clicked.connect(lambda: self.node.call_trigger('estop'))
        bar.addWidget(self.led)
        bar.addWidget(self.state_lbl)
        bar.addSpacing(20)
        bar.addWidget(self.mode_lbl)
        bar.addSpacing(20)
        bar.addWidget(self.fault_lbl)
        bar.addStretch()
        bar.addWidget(estop)
        root.addLayout(bar)

        # 中部：左仪表列 + 右操作列
        mid = QHBoxLayout()
        mid.setSpacing(10)
        mid.addWidget(self._build_left(), 0)
        mid.addWidget(self._build_right(), 1)
        root.addLayout(mid, 1)

        # 底部日志（紧凑）
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setFixedHeight(110)
        self.log.setStyleSheet('font-family:monospace; font-size:9pt;')
        root.addWidget(self.log)
        self._append_log('⚠️ 后端独占总线控制字：请确认 ros2_control 栈已停止。')

    def _build_left(self):
        col = QWidget()
        col.setFixedWidth(250)
        v = QVBoxLayout(col)
        v.setContentsMargins(0, 0, 0, 0)

        meas = QGroupBox('实时反馈')
        mv = QVBoxLayout(meas)
        self.pos_big = QLabel('--')
        self.pos_big.setStyleSheet('font-family:monospace; font-size:22pt; font-weight:bold;')
        self.pos_big.setAlignment(Qt.AlignCenter)
        self.pos_cap = QLabel('位置 rad')
        self.pos_cap.setAlignment(Qt.AlignCenter)
        self.pos_cap.setStyleSheet(f'color:{GRAY};')
        pos_cap = self.pos_cap
        self.vel_lbl = QLabel('速度  --')
        self.tq_lbl = QLabel('转矩  --')
        self.cur_lbl = QLabel('电流  --')
        self.sw_lbl = QLabel('状态字 --')
        for lbl in (self.vel_lbl, self.tq_lbl, self.cur_lbl, self.sw_lbl):
            lbl.setStyleSheet('font-family:monospace; font-size:11pt;')
        mv.addWidget(self.pos_big)
        mv.addWidget(pos_cap)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet('color:#e0e0e0;')
        mv.addWidget(line)
        mv.addWidget(self.vel_lbl)
        mv.addWidget(self.tq_lbl)
        mv.addWidget(self.cur_lbl)
        mv.addWidget(self.sw_lbl)
        v.addWidget(meas)

        servo = QGroupBox('伺服')
        sg = QGridLayout(servo)
        en = QPushButton('使 能')
        en.setMinimumHeight(40)
        en.setStyleSheet(f'background:{GREEN}; color:white; font-weight:bold; font-size:12pt;')
        en.clicked.connect(lambda: self.node.call_trigger('enable'))
        dis = QPushButton('失 能')
        dis.setMinimumHeight(40)
        dis.setToolTip('Shutdown 掉力矩。⚠️ 重力关节会下沉')
        dis.clicked.connect(lambda: self.node.call_trigger('disable'))
        fr = QPushButton('故障复位')
        fr.clicked.connect(lambda: self.node.call_trigger('fault_reset'))
        nmt = QPushButton('NMT 复位')
        nmt.setToolTip('驱动器重启（置零后用），1~2s 后恢复通信')
        nmt.clicked.connect(lambda: self.node.call_trigger('nmt_reset'))
        sg.addWidget(en, 0, 0)
        sg.addWidget(dis, 0, 1)
        sg.addWidget(fr, 1, 0)
        sg.addWidget(nmt, 1, 1)
        v.addWidget(servo)

        tools = QGroupBox('工具')
        tv = QVBoxLayout(tools)
        z = QPushButton('编码器置零…')
        z.clicked.connect(self._on_zero)
        lim = QPushButton('限制参数…')
        lim.clicked.connect(lambda: LimitsDialog(self, self.node).exec_())
        sdo = QPushButton('SDO 读写…')
        sdo.clicked.connect(lambda: SdoDialog(self, self.node).exec_())
        for b in (z, lim, sdo):
            tv.addWidget(b)
        v.addWidget(tools)

        # 一键软限位：配合点动找边界——jog 到边界后把「当前位置」直接写成 607D
        soft = QGroupBox('软限位（当前位置 →）')
        sg2 = QGridLayout(soft)
        lo = QPushButton('设为下限')
        lo.setToolTip('607D:01 ← 当前实测位置')
        lo.clicked.connect(lambda: self._set_soft_limit(1, '下限'))
        hi = QPushButton('设为上限')
        hi.setToolTip('607D:02 ← 当前实测位置')
        hi.clicked.connect(lambda: self._set_soft_limit(2, '上限'))
        clr = QPushButton('清除限位')
        clr.setToolTip('607D 写回默认满量程（±2³¹，手册口径 = 软限位不生效）')
        clr.clicked.connect(self._clear_soft_limit)
        sv2 = QPushButton('固化 EEPROM')
        sv2.setToolTip('1010:02h "save"——软限位断电保持的正式配置方式')
        sv2.clicked.connect(lambda: self.node.call_trigger('save_eeprom'))
        sg2.addWidget(lo, 0, 0)
        sg2.addWidget(hi, 0, 1)
        sg2.addWidget(clr, 1, 0)
        sg2.addWidget(sv2, 1, 1)
        v.addWidget(soft)
        v.addStretch()
        self.left_col = col
        return col

    def _build_right(self):
        col = QWidget()
        v = QVBoxLayout(col)
        v.setContentsMargins(0, 0, 0, 0)

        # 模式分段按钮
        seg = QHBoxLayout()
        self.mode_group = QButtonGroup(self)
        for i, text in enumerate(('位置 PP', '速度 PV', '力矩 PT')):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setMinimumHeight(38)
            b.setStyleSheet(
                'QPushButton { font-size:12pt; }'
                f'QPushButton:checked {{ background:{"#1565c0"}; color:white; font-weight:bold; }}')
            self.mode_group.addButton(b, i)
            seg.addWidget(b)
        self.mode_group.button(0).setChecked(True)
        self.mode_group.idClicked.connect(lambda i: self.stack.setCurrentIndex(i))
        v.addLayout(seg)

        # 参数区（随模式切换）
        self.stack = QStackedWidget()
        self.stack.addWidget(self._pp_page())
        self.stack.addWidget(self._pv_page())
        self.stack.addWidget(self._pt_page())
        v.addWidget(self.stack)

        # 执行/停止（位置固定,不随模式漂移）
        run_row = QHBoxLayout()
        self.run_btn = QPushButton('▶ 执 行')
        self.run_btn.setMinimumHeight(46)
        self.run_btn.setStyleSheet(f'background:{GREEN}; color:white; font-size:14pt; font-weight:bold;')
        self.run_btn.clicked.connect(self._on_run)
        self.stop_btn = QPushButton('■ 停 止')
        self.stop_btn.setMinimumHeight(46)
        self.stop_btn.setStyleSheet('font-size:14pt; font-weight:bold;')
        self.stop_btn.clicked.connect(self._on_stop)
        self.home_btn = QPushButton('⌂ 回 零')
        self.home_btn.setMinimumHeight(46)
        self.home_btn.setStyleSheet('font-size:14pt; font-weight:bold;')
        self.home_btn.setToolTip('PP 模式走回 0 rad（用位置页的轮廓速度/加速度），任何模式下可点')
        self.home_btn.clicked.connect(self._go_home)
        run_row.addWidget(self.run_btn, 2)
        run_row.addWidget(self.home_btn, 1)
        run_row.addWidget(self.stop_btn, 2)
        v.addLayout(run_row)

        # 实时曲线
        scope_box = QGroupBox('实时曲线（30s 窗口，各曲线独立归一化）')
        sv = QVBoxLayout(scope_box)
        self.scope = ScopeWidget()
        sv.addWidget(self.scope)
        ck_row = QHBoxLayout()
        for i, (label, color) in enumerate(SERIES):
            ck = QCheckBox(label)
            ck.setChecked(True)
            ck.setStyleSheet(f'color:{color}; font-weight:bold;')
            ck.toggled.connect(lambda on, si=i: self.scope.set_series_enabled(si, on))
            ck_row.addWidget(ck)
        ck_row.addStretch()
        sv.addLayout(ck_row)
        v.addWidget(scope_box, 1)

        self.right_col = col
        return col

    def _pp_page(self):
        w = QWidget()
        f = QFormLayout(w)
        self.pp_target = self._spin(-6.283, 6.283, 0.01, 0.0, ' rad')
        self.pp_vel = self._spin(0.01, 3.0, 0.05, 0.3, ' rad/s')
        self.pp_acc = self._spin(0.01, 6.0, 0.1, 1.0, ' rad/s²')
        f.addRow('目标位置', self.pp_target)
        f.addRow(self._slider_for(self.pp_target, self._send_pp))
        f.addRow('轮廓速度', self.pp_vel)
        f.addRow('加/减速度', self.pp_acc)
        jog = QHBoxLayout()
        self.pp_step = self._spin(0.001, 1.0, 0.01, 0.05, ' rad')
        jm = QPushButton('◀ 点动 −')
        jm.setToolTip('从当前实测位置反向走一个步进（立即执行）')
        jm.clicked.connect(lambda: self._jog_pp(-1))
        jp = QPushButton('点动 + ▶')
        jp.setToolTip('从当前实测位置正向走一个步进（立即执行）')
        jp.clicked.connect(lambda: self._jog_pp(+1))
        jog.addWidget(jm)
        jog.addWidget(self.pp_step)
        jog.addWidget(jp)
        jog.addStretch()
        f.addRow('步进点动', jog)
        return w

    def _pv_page(self):
        w = QWidget()
        f = QFormLayout(w)
        self.pv_vel = self._spin(-3.0, 3.0, 0.05, 0.2, ' rad/s')
        f.addRow('目标速度', self.pv_vel)
        f.addRow(self._slider_for(self.pv_vel, self._send_pv))
        jog = QHBoxLayout()
        self.pv_jog = self._spin(0.01, 3.0, 0.05, 0.2, ' rad/s')
        jm = QPushButton('◀ 按住反转')
        jp = QPushButton('按住正转 ▶')
        for b, sign in ((jm, -1), (jp, +1)):
            b.setToolTip('按住即动、松开即停（速度清零）')
            b.pressed.connect(lambda s=sign: self.node.call_target(
                self.node.run_pv, s * self.pv_jog.value(), 'PV 点动'))
            b.released.connect(lambda: self.node.call_trigger('zero_target'))
        jog.addWidget(jm)
        jog.addWidget(self.pv_jog)
        jog.addWidget(jp)
        jog.addStretch()
        f.addRow('方向点动', jog)
        f.addRow(QLabel('驱动器内部速度环（60FF）。持续转动，注意行程与线缆缠绕。'))
        return w

    def _pt_page(self):
        w = QWidget()
        f = QFormLayout(w)
        self.pt_tq = self._spin(-100.0, 100.0, 0.5, 3.0, ' %')
        self.pt_nm = QLabel('')          # 目标的 N·m 等效值（额定力矩 6076 读到后显示）
        self.pt_nm.setStyleSheet('font-family:monospace;')
        row = QHBoxLayout()
        row.addWidget(self.pt_tq)
        row.addWidget(self.pt_nm)
        row.addStretch()
        f.addRow('目标转矩（额定 %）', row)
        self.pt_tq.valueChanged.connect(self._update_pt_nm)
        f.addRow(self._slider_for(self.pt_tq, self._send_pt))
        warn = QLabel('⚠️ 力矩模式下「停止」= 力矩清零 = 自由下垂！\n带负载/重力关节要停请用急停或切位置 PP。')
        warn.setStyleSheet(f'color:{RED}; font-weight:bold;')
        f.addRow(warn)
        return w

    @staticmethod
    def _spin(lo, hi, step, init, suffix):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setValue(init)
        s.setDecimals(3)
        s.setSuffix(suffix)
        s.setMinimumWidth(140)
        return s

    @staticmethod
    def _slider_for(spin, on_release, ticks=1000):
        """滑杆↔数值框双向同步。拖动只改数值，**松手才发送一次**（on_release）——
        PP 拖动即发会触发驱动器不停重规划（joint_position_gui v1.1 的坑）。"""
        sl = QSlider(Qt.Horizontal)
        sl.setRange(0, ticks)
        lo, hi = spin.minimum(), spin.maximum()
        span = hi - lo

        def to_tick(v):
            return int(round((v - lo) / span * ticks))

        sl.setValue(to_tick(spin.value()))

        def on_slider(t):
            spin.blockSignals(True)
            spin.setValue(lo + span * t / ticks)
            spin.blockSignals(False)

        def on_spin(v):
            sl.blockSignals(True)
            sl.setValue(to_tick(v))
            sl.blockSignals(False)

        sl.valueChanged.connect(on_slider)
        spin.valueChanged.connect(on_spin)
        sl.sliderReleased.connect(on_release)
        sl.setToolTip('拖动改数值，松手即发送')
        return sl

    # ── 操作 ────────────────────────────────────────────────────────────────
    def _send_pp(self):
        self.node.set_pp_profile(self.pp_vel.value(), self.pp_acc.value())
        self.node.call_target(self.node.move_pp, self.pp_target.value(), 'PP 运动')

    def _send_pv(self):
        self.node.call_target(self.node.run_pv, self.pv_vel.value(), 'PV 运动')

    def _send_pt(self):
        self.node.call_target(self.node.run_pt, self.pt_tq.value(), 'PT 运动')

    def _update_pt_nm(self, pct):
        rated = self.node.rated_torque_nm
        self.pt_nm.setText(f'= {pct / 100.0 * rated:+.2f} N·m' if rated > 0 else '')

    def _on_run(self):
        (self._send_pp, self._send_pv, self._send_pt)[self.mode_group.checkedId()]()

    def _on_stop(self):
        # PP=halt（沿减速度停住保持使能）;PV/PT=目标清零
        if self.mode_group.checkedId() == 0:
            self.node.call_trigger('halt')
        else:
            self.node.call_trigger('zero_target')

    def _go_home(self):
        self.node.set_pp_profile(self.pp_vel.value(), self.pp_acc.value())
        self.pp_target.setValue(0.0)
        self.node.call_target(self.node.move_pp, 0.0, '回零')

    def _set_soft_limit(self, sub, label):
        scale = self.node.counts_per_rad
        if not scale:
            self._append_log('[FAIL] 软限位：换算系数未就绪（counts_per_rad 还没取到）')
            return
        pp = int(round(self.cur_pos * scale))
        self.node.write_object(0x607D, sub, pp, 4)
        self._append_log(f'[ OK ] 软限位{label} ← 当前位置 {self.cur_pos:+.4f} rad（{pp} pp），'
                         '断电保持记得点「固化 EEPROM」')

    def _clear_soft_limit(self):
        r = QMessageBox.warning(
            self, '清除软限位',
            '把 607D 恢复为默认满量程（-2³¹ ~ +2³¹-1），软限位保护随之失效，\n'
            '行程只剩机械限位挡着。确认清除？\n\n（断电保持需再点「固化 EEPROM」）',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return
        self.node.write_object(0x607D, 1, -2147483648, 4)
        self.node.write_object(0x607D, 2, 2147483647, 4)
        self._append_log('[ OK ] 软限位已清除（607D 满量程 = 不生效），断电保持记得「固化 EEPROM」')

    def _jog_pp(self, sign):
        self.node.set_pp_profile(self.pp_vel.value(), self.pp_acc.value())
        target = self.cur_pos + sign * self.pp_step.value()
        self.pp_target.setValue(target)
        self.node.call_target(self.node.move_pp, target, 'PP 点动')

    def _on_zero(self):
        r = QMessageBox.warning(
            self, '编码器置零',
            'HM 方法 35：把「当前位置」写为绝对零点（编码器片内保存，断电不丢）。\n\n'
            '确认：\n  1. ros2_control 栈已停止\n  2. 关节已摆到机械零位且静止\n\n'
            '置零会短暂使能电机（无运动），完成后自动失能，\n'
            '之后请点「NMT 复位」或断电重启驱动器。继续？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r == QMessageBox.Yes:
            self.node.call_trigger('zero_calibrate')

    # ── 状态刷新 ────────────────────────────────────────────────────────────
    def _on_state(self, pos, vel, tq):
        self.cur_pos = pos
        cpr = self.node.counts_per_rad
        rated_nm = self.node.rated_torque_nm
        self.pos_big.setText(f'{pos:+.4f}')
        if cpr:
            self.pos_cap.setText(f'位置 rad（{int(round(pos * cpr))} pp）')
        self.vel_lbl.setText(f'速度  {vel:+8.4f} rad/s')
        self.tq_lbl.setText(f'转矩  {tq:+6.1f} %'
                            + (f' = {tq / 100.0 * rated_nm:+.2f} N·m' if rated_nm > 0 else ''))
        self.scope.add_sample(pos, vel, tq, self.cur_amps)

    def _on_drive_status(self, sw, mode, fault, cur):
        self.last_status_t = time.monotonic()
        self.cur_amps = cur
        rated = self.node.rated_current_a
        self.cur_lbl.setText(f'电流  {cur:+6.2f} A'
                             + (f'（{abs(cur) / rated * 100:3.0f}%额定）' if rated > 0 else ''))
        if not self.pt_nm.text() and self.node.rated_torque_nm > 0:
            self._update_pt_nm(self.pt_tq.value())   # 额定力矩异步到位后补显示
        self._set_online(True)
        self.enabled402 = (sw & 0x006F) == 0x0027
        arrived = '（到位）' if self.enabled402 and (sw & 0x0400) else ''
        color = GREEN if self.enabled402 else (RED if sw & 0x0008 else GRAY)
        self.led.setStyleSheet(f'color:{color}; font-size:22pt;')
        self.state_lbl.setText(decode_402(sw) + arrived)
        self.state_lbl.setStyleSheet(f'font-weight:bold; font-size:16pt; color:{color};')
        self.mode_lbl.setText(f'模式 {MODE_NAMES.get(mode, "?")}')
        self.sw_lbl.setText(f'状态字 0x{sw:04X}')
        if sw & 0x0008:
            self.fault_lbl.setText(f'故障 0x{fault:04X}')
            self.fault_lbl.setStyleSheet(f'color:{RED}; font-weight:bold; font-size:12pt;')
        else:
            self.fault_lbl.setText('')
        # 未使能 → 运动区置灰（工具/伺服/急停不受影响）
        self.stack.setEnabled(self.enabled402)
        self.run_btn.setEnabled(self.enabled402)
        self.stop_btn.setEnabled(self.enabled402)
        self.home_btn.setEnabled(self.enabled402)
        if not self.enabled402:
            self.run_btn.setToolTip('先点左侧「使能」')
        else:
            self.run_btn.setToolTip('')
        if '节点' in self.node.backend_desc and '节点' not in self.windowTitle():
            self.setWindowTitle(f'RB200-CA 单电机调试平台 — {self.node.backend_desc}')

    def _check_alive(self):
        if time.monotonic() - self.last_status_t > 2.0:
            self._set_online(False)

    def _set_online(self, on: bool):
        # 后端断线：除日志外整体置灰（无后端功能置灰）
        self.left_col.setEnabled(on)
        self.right_col.setEnabled(on)
        if not on:
            self.led.setStyleSheet(f'color:{AMBER}; font-size:22pt;')
            self.state_lbl.setText('后端离线')
            self.state_lbl.setStyleSheet(f'font-weight:bold; font-size:16pt; color:{AMBER};')

    def _append_log(self, s: str):
        self.log.appendPlainText(f'[{QTime.currentTime().toString("HH:mm:ss")}] {s}')


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    signals = RosSignals()
    node = GuiNode(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    win = MainWindow(node, signals)
    win.resize(880, 720)
    win.show()
    rc = app.exec_()

    # 关窗即退出;电机失能由后端在 launch 关停路径里兜底（safeShutdown）
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    return rc


if __name__ == '__main__':
    sys.exit(main())
