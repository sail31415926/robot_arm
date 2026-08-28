#!/usr/bin/env python3
"""
@file   teach_gui.py
@brief  robot_arm_teach 示教操作面板（PyQt5）
@version 1.0
@date   2026-08-28

把本包的 13 个服务 + 3 个话题包成一个面板，替掉一长串 ros2 service call。
GUI 是**纯客户端**：不碰任何总线，不自己算轨迹，一个字节的控制指令都不直发 ——
录制/回放/校验全部由 arm_teach_node 执行，这里只发服务请求和 JogCommand。
所以 GUI 崩了对机械臂没有任何影响（点动会因断流看门狗在 0.3s 内停住）。

用法：
  终端①  ros2 launch robot_arm_bringup bringup.launch.py backend:=gazebo controller:=commander
  终端②  ros2 launch robot_arm_teach   teach.launch.py gui:=true
  单独起  ros2 run robot_arm_teach teach_gui

三个实现要点：

1. 服务调用全部异步（call_async + add_done_callback），绝不在 Qt 主线程等应答。
   回放的前置校验要跑自碰撞抽样（最多 20 次跨进程往返），同步等会把界面冻住几秒。
   done_callback 跑在 rclpy 的 spin 线程里，所以它只 emit 信号，绝不碰任何 widget
   —— Qt 的控件只能在主线程动。

2. /robot_arm/teach/state 是 latched（节点侧 QoS(1).transient_local().reliable()），
   订阅端必须用同样的 durability 才收得到那一帧历史值，否则 GUI 启动后要等到
   下一次 5Hz 广播才知道当前状态。QoS 不匹配在 ROS2 里**不报错，只是静默收不到**。

3. 点动滑块松手自动归零（弹簧回中），且只在 RECORDING 且未暂停时发布。
   这是点动该有的手感：手离开就停。真正保证停住的是节点侧 0.3s 断流看门狗，
   归零只是让它立刻停而不用等看门狗超时。

@copyright Copyright (c) 2026 eMeet
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robot_arm_teach.msg import JogCommand, MotionType, MotionMeta
from robot_arm_teach.msg import TeachState, PlaybackState
from robot_arm_teach.srv import (StartTeach, StopTeach, SaveTrajectory,
                                 LoadTrajectory, PlayTrajectory,
                                 ListTrajectories, DeleteTrajectory,
                                 SetPlaybackSpeed)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QDoubleSpinBox, QSpinBox, QPushButton, QGroupBox,
    QLineEdit, QComboBox, QCheckBox, QTableWidget, QTableWidgetItem,
    QProgressBar, QTextEdit, QStatusBar, QHeaderView, QStackedWidget,
    QAbstractItemView, QMessageBox,
)
from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QColor

# ── 与 msg 常量对齐的显示名 ──────────────────────────────────────────────────
# 这些顺序即 MotionType / TeachState / ControlMode 的 uint8 取值，改 msg 要同步改这里。
MOTION_TYPES = ['FREEFORM', 'DOLLY', 'TRUCK', 'ARC', 'CRANE']
MOTION_HINTS = ['自由示教，无运镜语义', '推/拉（沿光轴前后）', '横移（垂直光轴左右）',
                '环绕（绕被摄主体画圆弧）', '升降（沿竖直方向）']
STATE_NAMES = ['IDLE', 'RECORDING', 'RECORD_PAUSED', 'PLAYING', 'PLAY_PAUSED']
STATE_CN = ['空闲', '录制中', '录制暂停', '回放中', '回放暂停']
STATE_COLORS = ['#7F8C8D', '#E74C3C', '#E67E22', '#2980B9', '#E67E22']
CONTROL_MODES = ['TRAJECTORY', 'JOINT_VELOCITY', 'JOINT_EFFORT', 'ADMITTANCE']
PLAYBACK_STATUS = ['PLAYING', 'PAUSED', 'FINISHED', 'STOPPED', 'ABORTED']
PLAYBACK_CN = ['回放中', '已暂停', '播完', '被停止', '被中断']
FOLLOW_POLICIES = ['NONE（纯平移）', 'LOCK_SUBJECT（锁主体在画面中心）',
                   'PARALLEL（保持光轴方向）']

# 点动限速。与 config/teach_params.yaml 的 jog.max_joint_speed 一致 —— 改那里要同步改这里。
# 超了也不危险（中继会夹到该值），只是滑块刻度会与实际生效值不符。
JOG_MAX_SPEED = 0.4
JOG_SLIDER_SCALE = 1000     # 滑块整数刻度 → rad/s 的换算因子
JOG_PUBLISH_HZ = 20         # 输入频率。中继本身是 50Hz 节拍，20Hz 足够喂饱 0.3s 看门狗

# 回放倍率区间 = playback.min_speed_scale / max_speed_scale。
# 上限 1.0 意味着**只允许降速**：提速要缩放后仍过速度/加速度闸，节点侧不做静默限幅。
SPEED_MIN, SPEED_MAX = 0.1, 1.0

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

BOX_STYLE = 'QGroupBox{font-weight:bold; margin-top:6px;} ' \
            'QGroupBox::title{subcontrol-origin:margin; left:8px; padding:0 4px;}'
RO_STYLE = 'background:#F4F6F6; border:1px solid #CCD1D1; padding:2px;'


class RosSignals(QObject):
    """rclpy 线程 → Qt 主线程的唯一通道。所有跨线程数据都走信号，不共享可变状态。"""
    teach_state = pyqtSignal(object)
    playback_state = pyqtSignal(object)
    joint_state = pyqtSignal(list)
    service_done = pyqtSignal(str, object)
    log = pyqtSignal(str, str)          # (文本, 级别: info|ok|warn|err)


class TeachGuiNode(Node):
    """ROS 接线层。只做订阅/发布/服务调用，不含任何界面逻辑与状态判断。"""

    def __init__(self, signals: RosSignals):
        super().__init__('arm_teach_gui')
        self.signals = signals

        # latched：必须与节点侧 QoS(1).transient_local().reliable() 匹配，
        # 否则 GUI 启动后收不到那一帧历史状态（且不会有任何报错）。
        latched = QoSProfile(depth=1,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(TeachState, '/robot_arm/teach/state',
                                 lambda m: signals.teach_state.emit(m), latched)
        self.create_subscription(PlaybackState, '/robot_arm/teach/playback',
                                 lambda m: signals.playback_state.emit(m), 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_state, 10)

        self.jog_pub = self.create_publisher(JogCommand, '/robot_arm/teach/jog', 10)

        self.clients_map = {
            'start_teach':    self.create_client(StartTeach,      '/robot_arm/teach/start_teach'),
            'stop_teach':     self.create_client(StopTeach,       '/robot_arm/teach/stop_teach'),
            'pause_teach':    self.create_client(Trigger,         '/robot_arm/teach/pause_teach'),
            'resume_teach':   self.create_client(Trigger,         '/robot_arm/teach/resume_teach'),
            'save':           self.create_client(SaveTrajectory,  '/robot_arm/teach/save_trajectory'),
            'load':           self.create_client(LoadTrajectory,  '/robot_arm/teach/load_trajectory'),
            'play':           self.create_client(PlayTrajectory,  '/robot_arm/teach/play_trajectory'),
            'stop_playback':  self.create_client(Trigger,         '/robot_arm/teach/stop_playback'),
            'pause_playback': self.create_client(Trigger,         '/robot_arm/teach/pause_playback'),
            'resume_playback':self.create_client(Trigger,         '/robot_arm/teach/resume_playback'),
            'set_speed':      self.create_client(SetPlaybackSpeed,'/robot_arm/teach/set_playback_speed'),
            'list':           self.create_client(ListTrajectories,'/robot_arm/teach/list_trajectories'),
            'delete':         self.create_client(DeleteTrajectory,'/robot_arm/teach/delete_trajectory'),
        }

    def _on_joint_state(self, msg: JointState):
        """按名字取 6 轴 —— /joint_states 的顺序不保证（实测是 J2,J3,J1,J4..），不能按下标读。"""
        table = dict(zip(msg.name, msg.position))
        if not all(n in table for n in JOINT_NAMES):
            return          # 回读不全（云台未上电等）：不更新显示，节点侧也会暂停采样
        self.signals.joint_state.emit([table[n] for n in JOINT_NAMES])

    def call(self, tag: str, request):
        """异步调服务。应答经 service_done 信号回到主线程，绝不在此等待。"""
        client = self.clients_map[tag]
        if not client.service_is_ready():
            self.signals.log.emit(f'服务未就绪：{tag}（arm_teach_node 在跑吗？）', 'err')
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f, t=tag: self._on_done(t, f))

    def _on_done(self, tag: str, future):
        # 跑在 spin 线程：只 emit，不碰 widget。
        try:
            self.signals.service_done.emit(tag, future.result())
        except Exception as exc:
            self.signals.log.emit(f'{tag} 调用异常：{exc}', 'err')

    def publish_jog(self, velocities):
        msg = JogCommand()
        msg.velocities = [float(v) for v in velocities]
        self.jog_pub.publish(msg)


class MainWindow(QMainWindow):

    def __init__(self, node: TeachGuiNode, signals: RosSignals):
        super().__init__()
        self.node = node
        self.state = 0              # 最近一次 TeachState.state
        self.jog_enabled = False    # 最近一次 TeachState.jog_relay_enabled
        self._buffer_ready = False  # 内存缓冲区里是否有一条可回放的轨迹（未必落盘）

        self.setWindowTitle('eMeet 机械臂示教面板 — robot_arm_teach')
        self.resize(1180, 880)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 6)
        root.setSpacing(6)

        root.addWidget(self._build_status_bar_group())

        columns = QHBoxLayout()
        columns.setSpacing(8)
        left = QVBoxLayout()
        left.setSpacing(6)
        left.addWidget(self._build_record_group())
        left.addWidget(self._build_jog_group())
        left.addStretch(1)
        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(self._build_list_group())
        right.addWidget(self._build_save_group())
        right.addWidget(self._build_play_group())
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)
        root.addLayout(columns, 1)

        root.addWidget(self._build_log_group())

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage(
            '服务 /robot_arm/teach/*  |  话题 state（latched）/ playback / jog')

        # 点动发布节拍。只在 RECORDING 且未暂停时 start()，见 _sync_enabled()。
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(int(1000 / JOG_PUBLISH_HZ))
        self._jog_timer.timeout.connect(self._publish_jog)

        signals.teach_state.connect(self._on_teach_state)
        signals.playback_state.connect(self._on_playback_state)
        signals.joint_state.connect(self._on_joint_state)
        signals.service_done.connect(self._on_service_done)
        signals.log.connect(self._log)

        self._sync_enabled()

        # 等服务发现完成再拉列表。DDS 的服务发现耗时不确定（与节点一起 launch 时快，
        # 单独 ros2 run 时慢得多），所以轮询而不是定死一个延时 —— 定死的那版在慢的
        # 那次会直接报"服务未就绪"，而且不会再重试，界面就一直空着。
        self._boot_tries = 0
        self._boot_timer = QTimer(self)
        self._boot_timer.setInterval(400)
        self._boot_timer.timeout.connect(self._await_services)
        self._boot_timer.start()

    # ══════════════════════════════════════════════════════════════════════
    # 界面构建
    # ══════════════════════════════════════════════════════════════════════
    def _build_status_bar_group(self) -> QGroupBox:
        box = QGroupBox('示教状态（/robot_arm/teach/state，latched）')
        box.setStyleSheet(BOX_STYLE)
        grid = QGridLayout(box)
        grid.setSpacing(6)

        self.lbl_state = QLabel('—')
        f = QFont()
        f.setBold(True)
        f.setPointSize(13)
        self.lbl_state.setFont(f)
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setFixedWidth(150)
        self.lbl_state.setStyleSheet('color:white; background:#7F8C8D; padding:4px;')
        grid.addWidget(self.lbl_state, 0, 0, 2, 1)

        self.status_fields = {}
        fields = [('轨迹名', 0, 1), ('运镜', 0, 3), ('点数', 0, 5), ('时长', 0, 7),
                  ('底层模式', 1, 1), ('示教方式', 1, 3), ('点动闸', 1, 5), ('说明', 1, 7)]
        for name, row, col in fields:
            grid.addWidget(QLabel(f'{name}:'), row, col, Qt.AlignRight)
            lbl = QLabel('—')
            lbl.setStyleSheet(RO_STYLE)
            lbl.setMinimumWidth(110)
            self.status_fields[name] = lbl
            grid.addWidget(lbl, row, col + 1)
        grid.setColumnStretch(8, 1)
        return box

    def _build_record_group(self) -> QGroupBox:
        box = QGroupBox('① 录制')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        v.setSpacing(6)

        form = QGridLayout()
        form.setSpacing(6)
        form.addWidget(QLabel('轨迹名:'), 0, 0, Qt.AlignRight)
        self.ed_name = QLineEdit('teach_01')
        self.ed_name.setToolTip('须满足 [A-Za-z0-9_-]{1,64}；留空则由节点生成 teach_<时间戳>')
        form.addWidget(self.ed_name, 0, 1, 1, 3)

        form.addWidget(QLabel('运镜类型:'), 1, 0, Qt.AlignRight)
        self.cb_motion = QComboBox()
        for i, name in enumerate(MOTION_TYPES):
            self.cb_motion.addItem(f'{i}  {name}')
        self.cb_motion.currentIndexChanged.connect(self._on_motion_changed)
        form.addWidget(self.cb_motion, 1, 1)
        self.lbl_motion_hint = QLabel(MOTION_HINTS[0])
        self.lbl_motion_hint.setStyleSheet('color:#566573;')
        form.addWidget(self.lbl_motion_hint, 1, 2, 1, 2)

        form.addWidget(QLabel('示教方式:'), 2, 0, Qt.AlignRight)
        self.cb_teach_mode = QComboBox()
        self.cb_teach_mode.addItem('0  JOG（点动，实机唯一可用）')
        self.cb_teach_mode.addItem('1  DRAG（手拖，需 allow_drag 且仅仿真）')
        form.addWidget(self.cb_teach_mode, 2, 1, 1, 2)
        self.chk_fallback = QCheckBox('DRAG 不可用时降级 JOG')
        self.chk_fallback.setToolTip(
            'allow_jog_fallback。默认关 = 宁可失败也不做"看起来能用"的静默降级')
        form.addWidget(self.chk_fallback, 2, 3)

        form.addWidget(QLabel('备注:'), 3, 0, Qt.AlignRight)
        self.ed_desc = QLineEdit()
        form.addWidget(self.ed_desc, 3, 1, 1, 3)
        form.setColumnStretch(2, 1)
        v.addLayout(form)

        row = QHBoxLayout()
        self.btn_start = self._button('开始录制', '#27AE60', self._start_teach)
        self.btn_pause_rec = self._button('暂停', '#E67E22',
                                          lambda: self._trigger('pause_teach'))
        self.btn_resume_rec = self._button('继续', '#2980B9',
                                           lambda: self._trigger('resume_teach'))
        self.btn_stop_rec = self._button('结束录制', '#C0392B', self._stop_teach)
        for b in (self.btn_start, self.btn_pause_rec, self.btn_resume_rec, self.btn_stop_rec):
            row.addWidget(b)
        v.addLayout(row)

        self.lbl_buffer = QLabel('内存缓冲区：空')
        self.lbl_buffer.setStyleSheet(RO_STYLE)
        self.lbl_buffer.setToolTip(
            'stop_teach 不自动落盘 —— 可以先回放看看，满意再存。'
            '缓冲区只留一条，下次 start_teach 会覆盖')
        v.addWidget(self.lbl_buffer)
        return box

    def _build_jog_group(self) -> QGroupBox:
        box = QGroupBox(f'② 点动 J1-3（松手自动归零；上限 ±{JOG_MAX_SPEED:.2f} rad/s）')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        v.setSpacing(4)

        note = QLabel('★ 产品总线固定 3 轴，云台 J4-6 无法点动 —— '
                      '示教期间它们保持当前角度，但会被原样录进轨迹')
        note.setWordWrap(True)
        note.setStyleSheet('color:#B9770E;')
        v.addWidget(note)

        grid = QGridLayout()
        grid.setSpacing(6)
        for col, text in enumerate(['关节', '速度滑块', '指令 (rad/s)', '当前角 (rad)']):
            lbl = QLabel(text)
            lbl.setAlignment(Qt.AlignCenter)
            bold = QFont()
            bold.setBold(True)
            lbl.setFont(bold)
            grid.addWidget(lbl, 0, col)

        self.jog_sliders, self.jog_values, self.joint_labels = [], [], []
        for i in range(3):
            grid.addWidget(QLabel(f'Joint{i + 1}', alignment=Qt.AlignCenter), i + 1, 0)
            s = QSlider(Qt.Horizontal)
            s.setRange(int(-JOG_MAX_SPEED * JOG_SLIDER_SCALE),
                       int(JOG_MAX_SPEED * JOG_SLIDER_SCALE))
            s.setValue(0)
            s.setTickInterval(int(JOG_MAX_SPEED * JOG_SLIDER_SCALE / 4))
            s.setTickPosition(QSlider.TicksBelow)
            # 弹簧回中：手一离开就归零，不用等 0.3s 断流看门狗
            s.sliderReleased.connect(lambda sl=s: sl.setValue(0))
            s.valueChanged.connect(self._update_jog_labels)
            self.jog_sliders.append(s)
            grid.addWidget(s, i + 1, 1)

            val = QLabel('0.000', alignment=Qt.AlignCenter)
            val.setStyleSheet(RO_STYLE)
            val.setFixedWidth(80)
            self.jog_values.append(val)
            grid.addWidget(val, i + 1, 2)

            cur = QLabel('—', alignment=Qt.AlignCenter)
            cur.setStyleSheet(RO_STYLE)
            cur.setFixedWidth(90)
            self.joint_labels.append(cur)
            grid.addWidget(cur, i + 1, 3)
        grid.setColumnStretch(1, 1)
        v.addLayout(grid)

        # 云台 J4-6 只读回显：点动动不了它们，但会被录进轨迹，所以值得看见
        gimbal = QHBoxLayout()
        gimbal.addWidget(QLabel('云台（只读）:'))
        for i in range(3, 6):
            gimbal.addWidget(QLabel(f'J{i + 1}'))
            cur = QLabel('—', alignment=Qt.AlignCenter)
            cur.setStyleSheet(RO_STYLE)
            cur.setFixedWidth(80)
            self.joint_labels.append(cur)
            gimbal.addWidget(cur)
        gimbal.addStretch(1)
        self.btn_jog_zero = self._button('全部归零', '#7F8C8D', self._jog_zero)
        self.btn_jog_zero.setFixedWidth(90)
        gimbal.addWidget(self.btn_jog_zero)
        v.addLayout(gimbal)
        return box

    def _build_list_group(self) -> QGroupBox:
        box = QGroupBox('③ 轨迹列表（磁盘）')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        v.setSpacing(6)

        bar = QHBoxLayout()
        self.chk_filter = QCheckBox('按运镜过滤')
        self.cb_filter = QComboBox()
        for i, name in enumerate(MOTION_TYPES):
            self.cb_filter.addItem(f'{i}  {name}')
        self.cb_filter.setEnabled(False)
        self.chk_filter.toggled.connect(self.cb_filter.setEnabled)
        bar.addWidget(self.chk_filter)
        bar.addWidget(self.cb_filter)
        bar.addStretch(1)
        bar.addWidget(self._button('刷新', '#2980B9', self._refresh_list))
        self.btn_load = self._button('载入缓冲区', '#16A085', self._load_selected)
        self.btn_delete = self._button('删除', '#C0392B', self._delete_selected)
        bar.addWidget(self.btn_load)
        bar.addWidget(self.btn_delete)
        v.addLayout(bar)

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['轨迹名', '运镜', '点数', '时长(s)', '创建时间'])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setMinimumHeight(150)
        self.table.itemSelectionChanged.connect(self._on_row_selected)
        v.addWidget(self.table)
        return box

    def _build_save_group(self) -> QGroupBox:
        box = QGroupBox('④ 保存（落盘前跑完整静态校验，不合格拒绝写入）')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        v.setSpacing(6)

        row = QHBoxLayout()
        row.addWidget(QLabel('保存为:'))
        self.ed_save_name = QLineEdit()
        self.ed_save_name.setPlaceholderText('留空 = 用缓冲区里的名字')
        row.addWidget(self.ed_save_name, 1)
        self.chk_overwrite = QCheckBox('覆盖同名')
        row.addWidget(self.chk_overwrite)
        v.addLayout(row)

        meta_row = QHBoxLayout()
        self.chk_meta = QCheckBox('写入运镜元数据')
        self.chk_meta.setToolTip(
            'meta_override。元数据只是标签，不参与回放执行 —— '
            '回放跑的永远是录下来的 6 轴关节轨迹')
        meta_row.addWidget(self.chk_meta)
        meta_row.addWidget(QLabel('主体标签:'))
        self.ed_subject = QLineEdit()
        self.ed_subject.setPlaceholderText('如 person_0，可空')
        meta_row.addWidget(self.ed_subject, 1)
        v.addLayout(meta_row)

        self.meta_stack = QStackedWidget()
        self.meta_stack.addWidget(self._meta_page_freeform())
        self.meta_stack.addWidget(self._meta_page_dolly())
        self.meta_stack.addWidget(self._meta_page_truck())
        self.meta_stack.addWidget(self._meta_page_arc())
        self.meta_stack.addWidget(self._meta_page_crane())
        self.meta_stack.setMaximumHeight(90)
        v.addWidget(self.meta_stack)

        self.btn_save = self._button('保存到磁盘', '#27AE60', self._save)
        v.addWidget(self.btn_save)
        return box

    def _spin(self, lo, hi, val, step=0.05, dec=3, width=76):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(dec)
        s.setSingleStep(step)
        s.setValue(val)
        s.setFixedWidth(width)
        return s

    def _meta_page_freeform(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel('FREEFORM 无运镜参数。元数据全空不影响回放。')
        lbl.setStyleSheet('color:#566573;')
        lay.addWidget(lbl)
        lay.addStretch(1)
        return w

    def _meta_page_dolly(self) -> QWidget:
        w = QWidget()
        g = QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(4)
        g.addWidget(QLabel('推进方向 (base_link 单位向量):'), 0, 0)
        self.dolly_dir = [self._spin(-1, 1, v) for v in (1.0, 0.0, 0.0)]
        for i, s in enumerate(self.dolly_dir):
            g.addWidget(s, 0, 1 + i)
        g.addWidget(QLabel('距离 (m，拉远为负):'), 1, 0)
        self.dolly_dist = self._spin(-5, 5, 0.2)
        g.addWidget(self.dolly_dist, 1, 1)
        self.dolly_facing = QCheckBox('全程保持相机朝向主体')
        g.addWidget(self.dolly_facing, 1, 2, 1, 2)
        g.setColumnStretch(4, 1)
        return w

    def _meta_page_truck(self) -> QWidget:
        w = QWidget()
        g = QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(4)
        g.addWidget(QLabel('横移方向 (base_link 单位向量):'), 0, 0)
        self.truck_dir = [self._spin(-1, 1, v) for v in (0.0, 1.0, 0.0)]
        for i, s in enumerate(self.truck_dir):
            g.addWidget(s, 0, 1 + i)
        g.addWidget(QLabel('距离 (m):'), 1, 0)
        self.truck_dist = self._spin(-5, 5, 0.2)
        g.addWidget(self.truck_dist, 1, 1)
        self.truck_policy = QComboBox()
        self.truck_policy.addItems(FOLLOW_POLICIES)
        g.addWidget(self.truck_policy, 1, 2, 1, 2)
        g.setColumnStretch(4, 1)
        return w

    def _meta_page_arc(self) -> QWidget:
        w = QWidget()
        g = QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(4)
        g.addWidget(QLabel('环绕目标点 (m):'), 0, 0)
        self.arc_target = [self._spin(-5, 5, 0.0) for _ in range(3)]
        for i, s in enumerate(self.arc_target):
            g.addWidget(s, 0, 1 + i)
        g.addWidget(QLabel('半径 (m):'), 0, 4)
        self.arc_radius = self._spin(0, 5, 0.3)
        g.addWidget(self.arc_radius, 0, 5)
        g.addWidget(QLabel('方位角 起→止 (deg):'), 1, 0)
        self.arc_start = self._spin(-360, 360, 0.0, 5, 1)
        self.arc_end = self._spin(-360, 360, 90.0, 5, 1)
        g.addWidget(self.arc_start, 1, 1)
        g.addWidget(self.arc_end, 1, 2)
        self.arc_dir = QComboBox()
        self.arc_dir.addItems(['CW（俯视顺时针）', 'CCW（俯视逆时针）'])
        g.addWidget(self.arc_dir, 1, 3, 1, 3)
        g.setColumnStretch(6, 1)
        return w

    def _meta_page_crane(self) -> QWidget:
        w = QWidget()
        g = QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(4)
        g.addWidget(QLabel('末端高度变化 (m，带符号):'), 0, 0)
        self.crane_delta = self._spin(-2, 2, 0.15)
        g.addWidget(self.crane_delta, 0, 1)
        self.crane_dir = QComboBox()
        self.crane_dir.addItems(['UP（上升）', 'DOWN（下降）'])
        g.addWidget(self.crane_dir, 0, 2)
        g.setColumnStretch(3, 1)
        return w

    def _build_play_group(self) -> QGroupBox:
        box = QGroupBox('⑤ 回放（九道前置闸，任一不过就不下发任何指令）')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        v.setSpacing(6)

        row = QHBoxLayout()
        row.addWidget(QLabel('轨迹:'))
        self.ed_play_name = QLineEdit()
        self.ed_play_name.setPlaceholderText('留空 = 用内存缓冲区里的那条（刚录完未落盘的）')
        row.addWidget(self.ed_play_name, 1)
        btn_use_buffer = self._button('用缓冲区', '#5D6D7E', self.ed_play_name.clear)
        btn_use_buffer.setToolTip(
            'PlayTrajectory.name 留空即回放内存缓冲区那条。'
            'stop_teach 不自动落盘，所以刚录完的轨迹只在缓冲区里')
        btn_use_buffer.setFixedWidth(88)
        row.addWidget(btn_use_buffer)
        row.addWidget(QLabel('倍率:'))
        self.sp_speed = self._spin(SPEED_MIN, SPEED_MAX, 0.5, 0.05, 2)
        self.sp_speed.setToolTip(
            f'playback.min/max_speed_scale = [{SPEED_MIN}, {SPEED_MAX}]。'
            '上限 1.0 = 只允许降速')
        row.addWidget(self.sp_speed)
        row.addWidget(QLabel('循环:'))
        self.sp_loop = QSpinBox()
        self.sp_loop.setRange(1, 99)
        self.sp_loop.setFixedWidth(56)
        row.addWidget(self.sp_loop)
        v.addLayout(row)

        btns = QHBoxLayout()
        self.btn_dry = self._button('干跑校验', '#8E44AD', lambda: self._play(dry_run=True))
        self.btn_dry.setToolTip('只跑闸①~⑦并返回结论，不切模式、不下发 —— 上板前自检用')
        self.btn_play = self._button('开始回放', '#27AE60', lambda: self._play(dry_run=False))
        self.btn_pause_play = self._button('暂停', '#E67E22',
                                           lambda: self._trigger('pause_playback'))
        self.btn_resume_play = self._button('继续', '#2980B9',
                                            lambda: self._trigger('resume_playback'))
        self.btn_stop_play = self._button('停止', '#C0392B',
                                          lambda: self._trigger('stop_playback'))
        for b in (self.btn_dry, self.btn_play, self.btn_pause_play,
                  self.btn_resume_play, self.btn_stop_play):
            btns.addWidget(b)
        v.addLayout(btns)

        speed_row = QHBoxLayout()
        self.btn_set_speed = self._button('回放中改倍率', '#16A085', self._set_speed)
        self.btn_set_speed.setToolTip(
            '下一分段生效（约 chunk_horizon_sec = 1.0s）。要立刻生效就先暂停再改再继续')
        speed_row.addWidget(self.btn_set_speed)
        speed_row.addStretch(1)
        v.addLayout(speed_row)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat('%p%  —  未在回放')
        v.addWidget(self.progress)

        self.lbl_play_info = QLabel('—')
        self.lbl_play_info.setStyleSheet(RO_STYLE)
        v.addWidget(self.lbl_play_info)
        return box

    def _build_log_group(self) -> QGroupBox:
        box = QGroupBox('日志（服务应答 / exit_reason）')
        box.setStyleSheet(BOX_STYLE)
        v = QVBoxLayout(box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(190)
        self.log_view.setStyleSheet('font-family:monospace; font-size:11px;')
        v.addWidget(self.log_view)
        return box

    def _button(self, text: str, color: str, slot) -> QPushButton:
        b = QPushButton(text)
        b.setFixedHeight(30)
        b.setStyleSheet(
            f'QPushButton{{background:{color}; color:white; font-weight:bold; '
            f'border:none; padding:4px 10px;}} '
            f'QPushButton:disabled{{background:#D5D8DC; color:#909497;}}')
        b.clicked.connect(slot)
        return b

    def _await_services(self):
        """等 /robot_arm/teach/list_trajectories 出现，就绪后拉一次列表。

        只探这一个服务：13 个服务都由同一个节点在同一次 create_service 里注册，
        看见其中一个就意味着节点在跑。最多等 10s，超时给出能照着排查的话。
        """
        self._boot_tries += 1
        if self.node.clients_map['list'].service_is_ready():
            self._boot_timer.stop()
            self._log('已连上 arm_teach_node，13 个服务就绪', 'ok')
            self._refresh_list()
        elif self._boot_tries >= 25:
            self._boot_timer.stop()
            self._log('等了 10s 仍没发现 /robot_arm/teach/* 服务。'
                      'arm_teach_node 起了吗？（ros2 node list | grep arm_teach）', 'err')

    # ══════════════════════════════════════════════════════════════════════
    # 状态回调
    # ══════════════════════════════════════════════════════════════════════
    def _on_teach_state(self, msg):
        self.state = msg.state
        self.jog_enabled = msg.jog_relay_enabled
        idx = msg.state if msg.state < len(STATE_NAMES) else 0

        self.lbl_state.setText(f'{STATE_CN[idx]}\n{STATE_NAMES[idx]}')
        self.lbl_state.setStyleSheet(
            f'color:white; background:{STATE_COLORS[idx]}; padding:4px;')

        mt = msg.motion_type.value
        cm = msg.underlying_control_mode.mode
        self.status_fields['轨迹名'].setText(msg.trajectory_name or '—')
        self.status_fields['运镜'].setText(
            MOTION_TYPES[mt] if mt < len(MOTION_TYPES) else str(mt))
        self.status_fields['点数'].setText(str(msg.point_count))
        self.status_fields['时长'].setText(f'{msg.elapsed_sec:.2f} s')
        self.status_fields['底层模式'].setText(
            CONTROL_MODES[cm] if cm < len(CONTROL_MODES) else str(cm))
        self.status_fields['示教方式'].setText(
            'JOG' if msg.teach_mode == 0 else 'DRAG')
        gate = self.status_fields['点动闸']
        gate.setText('放行' if msg.jog_relay_enabled else '关闭')
        gate.setStyleSheet(RO_STYLE + ('color:#1E8449; font-weight:bold;'
                                       if msg.jog_relay_enabled else 'color:#909497;'))
        self.status_fields['说明'].setText(msg.message or '—')

        if msg.state in (1, 2):
            self.lbl_buffer.setText(
                f'录制中：{msg.trajectory_name}  已录 {msg.point_count} 点 / {msg.elapsed_sec:.1f} s')
        self._sync_enabled()

    def _on_playback_state(self, msg):
        idx = msg.status if msg.status < len(PLAYBACK_STATUS) else 0
        self.progress.setValue(int(msg.progress_percent))
        self.progress.setFormat(f'%p%  —  {PLAYBACK_CN[idx]}')
        self.lbl_play_info.setText(
            f'{msg.trajectory_name}   点 {msg.point_index}/{msg.point_total}   '
            f'{msg.elapsed_sec:.1f}/{msg.total_sec:.1f} s（原始时间轴）   '
            f'倍率 {msg.speed_scale:.2f}   {msg.message}')

    def _on_joint_state(self, positions):
        for i, pos in enumerate(positions):
            self.joint_labels[i].setText(f'{pos:.4f}')

    def _on_motion_changed(self, idx: int):
        self.lbl_motion_hint.setText(MOTION_HINTS[idx])
        self.meta_stack.setCurrentIndex(idx)

    def _on_row_selected(self):
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        if not rows:
            return
        name = self.table.item(rows[0].row(), 0).text()
        self.ed_play_name.setText(name)

    def _update_jog_labels(self):
        for i, s in enumerate(self.jog_sliders):
            self.jog_values[i].setText(f'{s.value() / JOG_SLIDER_SCALE:+.3f}')

    def _sync_enabled(self):
        """按状态机开关按钮 —— 节点侧本来就会拒绝非法操作，这里是为了让界面先说清楚。"""
        idle = self.state == 0
        recording = self.state == 1
        rec_paused = self.state == 2
        playing = self.state == 3
        play_paused = self.state == 4

        self.btn_start.setEnabled(idle)
        self.btn_pause_rec.setEnabled(recording)
        self.btn_resume_rec.setEnabled(rec_paused)
        self.btn_stop_rec.setEnabled(recording or rec_paused)

        jog_ok = self.jog_enabled and recording
        for s in self.jog_sliders:
            s.setEnabled(jog_ok)
        self.btn_jog_zero.setEnabled(jog_ok)
        if jog_ok:
            if not self._jog_timer.isActive():
                self._jog_timer.start()
        else:
            if self._jog_timer.isActive():
                self._jog_timer.stop()
            self._jog_zero(publish=False)

        self.btn_dry.setEnabled(idle)
        self.btn_play.setEnabled(idle)
        self.btn_pause_play.setEnabled(playing)
        self.btn_resume_play.setEnabled(play_paused)
        self.btn_stop_play.setEnabled(playing or play_paused)
        self.btn_set_speed.setEnabled(playing or play_paused)
        self.btn_save.setEnabled(idle)
        self.btn_load.setEnabled(idle)
        self.btn_delete.setEnabled(idle)

    # ══════════════════════════════════════════════════════════════════════
    # 动作
    # ══════════════════════════════════════════════════════════════════════
    def _publish_jog(self):
        self.node.publish_jog([s.value() / JOG_SLIDER_SCALE for s in self.jog_sliders])

    def _jog_zero(self, publish=True):
        for s in self.jog_sliders:
            s.blockSignals(True)
            s.setValue(0)
            s.blockSignals(False)
        self._update_jog_labels()
        if publish and self.jog_enabled:
            self._publish_jog()

    def _start_teach(self):
        req = StartTeach.Request()
        req.name = self.ed_name.text().strip()
        req.motion_type = MotionType(value=self.cb_motion.currentIndex())
        req.teach_mode = self.cb_teach_mode.currentIndex()
        req.allow_jog_fallback = self.chk_fallback.isChecked()
        req.description = self.ed_desc.text().strip()
        self._log(f'→ start_teach  name={req.name or "(自动)"}  '
                  f'motion={MOTION_TYPES[req.motion_type.value]}  '
                  f'teach_mode={req.teach_mode}', 'info')
        self.node.call('start_teach', req)

    def _stop_teach(self):
        self._log('→ stop_teach', 'info')
        self.node.call('stop_teach', StopTeach.Request())

    def _trigger(self, tag: str):
        self._log(f'→ {tag}', 'info')
        self.node.call(tag, Trigger.Request())

    def _build_meta(self) -> MotionMeta:
        meta = MotionMeta()
        meta.subject_label = self.ed_subject.text().strip()
        idx = self.cb_motion.currentIndex()
        if idx == 1:
            meta.dolly_direction = [s.value() for s in self.dolly_dir]
            meta.dolly_distance_m = self.dolly_dist.value()
            meta.dolly_keep_camera_facing = self.dolly_facing.isChecked()
        elif idx == 2:
            meta.truck_direction = [s.value() for s in self.truck_dir]
            meta.truck_distance_m = self.truck_dist.value()
            meta.truck_follow_policy = self.truck_policy.currentIndex()
        elif idx == 3:
            meta.arc_target_point = [s.value() for s in self.arc_target]
            meta.arc_radius_m = self.arc_radius.value()
            meta.arc_start_angle_deg = self.arc_start.value()
            meta.arc_end_angle_deg = self.arc_end.value()
            meta.arc_rotation_direction = self.arc_dir.currentIndex()
        elif idx == 4:
            meta.crane_height_delta_m = self.crane_delta.value()
            meta.crane_direction = self.crane_dir.currentIndex()
        return meta

    def _save(self):
        req = SaveTrajectory.Request()
        req.name = self.ed_save_name.text().strip()
        req.overwrite = self.chk_overwrite.isChecked()
        req.meta_override = self.chk_meta.isChecked()
        if req.meta_override:
            req.motion_meta = self._build_meta()
        self._log(f'→ save_trajectory  name={req.name or "(缓冲区名)"}  '
                  f'overwrite={req.overwrite}  meta_override={req.meta_override}', 'info')
        self.node.call('save', req)

    def _play(self, dry_run: bool):
        req = PlayTrajectory.Request()
        req.name = self.ed_play_name.text().strip()
        req.speed_scale = self.sp_speed.value()
        req.loop_count = self.sp_loop.value()
        req.dry_run = dry_run
        self._log(f'→ play_trajectory  name={req.name or "(缓冲区)"}  '
                  f'speed={req.speed_scale:.2f}  loop={req.loop_count}  '
                  f'dry_run={dry_run}', 'info')
        self.node.call('play', req)

    def _set_speed(self):
        req = SetPlaybackSpeed.Request()
        req.speed_scale = self.sp_speed.value()
        self._log(f'→ set_playback_speed  {req.speed_scale:.2f}', 'info')
        self.node.call('set_speed', req)

    def _refresh_list(self):
        req = ListTrajectories.Request()
        req.filter_by_motion_type = self.chk_filter.isChecked()
        req.motion_type_filter = self.cb_filter.currentIndex()
        self.node.call('list', req)

    def _selected_name(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            self._log('先在列表里选一行', 'warn')
            return None
        return self.table.item(rows[0].row(), 0).text()

    def _load_selected(self):
        name = self._selected_name()
        if not name:
            return
        req = LoadTrajectory.Request()
        req.name = name
        self._log(f'→ load_trajectory  {name}', 'info')
        self.node.call('load', req)

    def _delete_selected(self):
        name = self._selected_name()
        if not name:
            return
        if QMessageBox.question(
                self, '确认删除', f'删除磁盘上的轨迹 "{name}"？此操作不可撤销。',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        req = DeleteTrajectory.Request()
        req.name = name
        self._log(f'→ delete_trajectory  {name}', 'info')
        self.node.call('delete', req)

    # ══════════════════════════════════════════════════════════════════════
    # 服务应答
    # ══════════════════════════════════════════════════════════════════════
    def _on_service_done(self, tag: str, resp):
        level = 'ok' if getattr(resp, 'success', False) else 'err'
        msg = getattr(resp, 'message', '')

        if tag == 'list':
            self._fill_table(resp)
            return

        if tag == 'start_teach':
            self._log(f'← start_teach  success={resp.success}  '
                      f'name={resp.trajectory_name}  teach_mode={resp.teach_mode}  {msg}', level)
            if resp.success:
                # 只填「保存为」，**不填回放框** —— 回放框留空才表示"用内存缓冲区"。
                # 填上名字会让回放去磁盘找这条还没落盘的轨迹，直接 exit_reason=not_found，
                # 正好挡住「录完先回放看看，满意再存」这条主流程。
                self.ed_save_name.setText(resp.trajectory_name)
                self.ed_play_name.clear()
                self._buffer_ready = False
        elif tag == 'stop_teach':
            self._log(f'← stop_teach  success={resp.success}  '
                      f'{resp.point_count} 点（原始 {resp.raw_point_count}）  '
                      f'{resp.duration_sec:.2f} s  {msg}', level)
            if resp.success:
                self._buffer_ready = True
                self.ed_play_name.clear()      # 留空 = 回放缓冲区里刚录的这条
                self.lbl_buffer.setText(
                    f'缓冲区：{resp.trajectory_name}  {resp.point_count} 点'
                    f'（原始 {resp.raw_point_count}，压缩掉 '
                    f'{resp.raw_point_count - resp.point_count}）  '
                    f'{resp.duration_sec:.2f} s  —— 未落盘')
                self._log('提示：回放框已留空 → 现在点「开始回放」放的就是缓冲区里这条'
                          '（还没落盘）。满意再点「保存到磁盘」。', 'ok')
        elif tag == 'save':
            self._log(f'← save_trajectory  success={resp.success}  '
                      f'{resp.point_count} 点  path={resp.path}  {msg}', level)
            if resp.success:
                self._refresh_list()
        elif tag == 'load':
            traj = resp.trajectory
            self._log(f'← load_trajectory  success={resp.success}  '
                      f'{len(traj.points)} 点  {traj.duration_sec:.2f} s  {msg}', level)
            if traj.name:
                self._buffer_ready = True
                self.ed_play_name.setText(traj.name)
                self.lbl_buffer.setText(
                    f'缓冲区：{traj.name}  {len(traj.points)} 点  '
                    f'{traj.duration_sec:.2f} s  （从磁盘载入）')
        elif tag == 'play':
            # exit_reason 是九道闸里哪一道没过的唯一线索，必须原样呈现
            self._log(f'← play_trajectory  success={resp.success}  '
                      f'exit_reason={resp.exit_reason}  {resp.point_count} 点  '
                      f'预计 {resp.duration_sec:.2f} s  {msg}', level)
            if not resp.success:
                self.progress.setFormat(f'%p%  —  被拒：{resp.exit_reason}')
                if resp.exit_reason == 'not_found' and self._buffer_ready:
                    self._log('↑ 这条名字在磁盘上没有。刚录的那条还在内存缓冲区 —— '
                              '点「用缓冲区」清空轨迹名再回放，或先「保存到磁盘」。', 'warn')
        elif tag == 'delete':
            self._log(f'← delete_trajectory  success={resp.success}  {msg}', level)
            if resp.success:
                self._refresh_list()
        elif tag == 'set_speed':
            self._log(f'← set_playback_speed  success={resp.success}  '
                      f'生效倍率={resp.active_speed_scale:.2f}  {msg}', level)
        else:
            self._log(f'← {tag}  success={resp.success}  {msg}', level)

    def _fill_table(self, resp):
        self.table.setRowCount(0)
        if not resp.success:
            self._log(f'← list_trajectories 失败：{resp.message}', 'err')
            return
        for i, name in enumerate(resp.names):
            self.table.insertRow(i)
            mt = resp.motion_types[i]
            cells = [name,
                     MOTION_TYPES[mt] if mt < len(MOTION_TYPES) else str(mt),
                     str(resp.point_counts[i]),
                     f'{resp.durations_sec[i]:.2f}',
                     resp.created_at[i]]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col in (2, 3):
                    item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(i, col, item)
        # 解析失败的文件值得显式报出来 —— 多半是手改坏的 YAML 或版本不支持
        if resp.invalid_names:
            self._log(f'⚠ 有 {len(resp.invalid_names)} 个文件解析失败（版本不支持/格式坏）：'
                      f'{", ".join(resp.invalid_names)}', 'warn')
        self._log(f'← list_trajectories  {len(resp.names)} 条', 'ok')

    def _log(self, text: str, level: str = 'info'):
        color = {'info': '#2C3E50', 'ok': '#1E8449',
                 'warn': '#B9770E', 'err': '#C0392B'}.get(level, '#2C3E50')
        self.log_view.append(f'<span style="color:{color};">{text}</span>')
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum())

    def closeEvent(self, event):
        # 关窗前把点动停掉：GUI 一走没人再喂指令，虽然 0.3s 后看门狗也会停，
        # 但主动补一帧全 0 更干净。
        if self._jog_timer.isActive():
            self._jog_timer.stop()
            self.node.publish_jog([0.0, 0.0, 0.0])
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    signals = RosSignals()
    node = TeachGuiNode(signals)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    win = MainWindow(node, signals)
    win.show()
    code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
