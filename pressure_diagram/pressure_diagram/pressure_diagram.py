#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import signal
import sys
import time

# 系统同时装有 PyQt5 和 PyQt6 时，pyqtgraph 会自动挑中 PyQt6，其控件便无法放进
# 本文件使用的 PyQt5 布局(addWidget 报 unexpected type)。导入前先钉死绑定。
os.environ.setdefault('PYQTGRAPH_QT_LIB', 'PyQt5')

import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
import rclpy  # noqa: E402
from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

pg.setConfigOption('imageAxisOrder', 'row-major')
pg.setConfigOption('useOpenGL', False)

# ---------------------------------------------------------------------------
# 显示器缩放适配
# 界面所有尺寸/字号均以 BASE_SCREEN 为设计基准，运行时按显示器可用区域等比缩放。
# 注意：若系统已启用 Qt 的 HiDPI 缩放(如 200%)，availableGeometry 返回的是
# 逻辑像素，系数会自然回到基准附近，由 Qt 负责放大，不会与此处重复叠加。
# ---------------------------------------------------------------------------
# 设计基准分辨率：在此分辨率下 ui_scale 恰好为 1.00，界面按标称尺寸显示
BASE_SCREEN_W = 1280.0
BASE_SCREEN_H = 800.0
# 热力图 grow 系数的参考窗口尺寸。刻意保持 1600x900 不跟随 BASE_SCREEN，
# 否则热力图会在基准屏上被放大一圈(详见 _matrix_grow_factor)
BASE_WINDOW_W = 1600.0
BASE_WINDOW_H = 900.0

# 缩放力度调节旋钮：嫌界面大就把 *_MAX 调小，嫌小就调大
UI_SCALE_MIN = 0.75      # 显示器缩放系数下限(小屏)
UI_SCALE_MAX = 1.15      # 显示器缩放系数上限(大屏/高分屏)
MATRIX_GROW_MIN = 0.6    # 热力图随窗口收缩的下限
MATRIX_GROW_MAX = 1.4    # 热力图随窗口放大的上限

_UI_SCALE = None

# ---------------------------------------------------------------------------
# 话题数据格式(由 linker_hand_o30.py publish_touch 发布)
#   合力: {"stamp","hand_type","sensors":{"1":{"name","unit","force":{"FnS"..},
#                                             "cell_sum","cell_max"}, ...}}
#   点阵: {"stamp","hand_type","sensors":{"1":{"name","rows","cols","matrix"}, ...}}
# sensors 的 key 是协议传感器编号；名称随固件可能是英文或中文，故以编号为准，
# 编号不认识时再退回名称匹配。
# ---------------------------------------------------------------------------
SENSOR_ID_TO_KEY = {1: 'thumb', 2: 'index', 3: 'middle', 4: 'ring',
                    5: 'pinky', 6: 'palm'}
SENSOR_NAME_TO_KEY = {
    'thumb': 'thumb', 'index': 'index', 'middle': 'middle', 'ring': 'ring',
    'pinky': 'pinky', 'little': 'pinky', 'palm': 'palm',
    '大拇指': 'thumb', '拇指': 'thumb', '食指': 'index', '中指': 'middle',
    '无名指': 'ring', '小指': 'pinky', '手掌': 'palm', '掌心': 'palm',
}
# 合力优先取协议声明的法向力合力；描述里没声明时用点阵求和近似(见 SDK 文档)
FORCE_TAG = 'FnS'
# 点阵行列数一律取自话题数据(sensors[i].rows/cols)，代码里不假设任何尺寸。
# 下面这个 1×1 只是"还没收到任何一帧"时的占位形状，收到数据即被真实形状取代。
PLACEHOLDER_MATRIX_SHAPE = (1, 1)
# 启动时为拿到真实行列数最多等待的秒数(拿不到就先用占位形状，之后随数据自适应)
SHAPE_PREFETCH_TIMEOUT = 2.0


def sensor_key(sid, entry):
    """把 sensors 的一项映射成内部键名(thumb/index/.../palm)，认不出返回 None。"""
    try:
        key = SENSOR_ID_TO_KEY.get(int(sid))
    except (TypeError, ValueError):
        key = None
    if key is None:
        name = str((entry or {}).get('name', '')).strip().lower()
        key = SENSOR_NAME_TO_KEY.get(name)
    return key


def sensor_mass(entry):
    """取一个传感器的合力值：优先 force[FnS]，否则退回 cell_sum。"""
    force = entry.get('force') or {}
    val = force.get(FORCE_TAG)
    if not isinstance(val, (int, float)):
        val = entry.get('cell_sum')
    if not isinstance(val, (int, float)):
        val = entry.get('cell_max')
    return float(val) if isinstance(val, (int, float)) else None


def ui_scale(refresh: bool = False) -> float:
    """显示器缩放系数(相对 1920x1080)，按上下限限幅并缓存。"""
    global _UI_SCALE
    if _UI_SCALE is not None and not refresh:
        return _UI_SCALE
    scale = 1.0
    app = QtWidgets.QApplication.instance()
    if app is not None:
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            scale = min(avail.width() / BASE_SCREEN_W,
                        avail.height() / BASE_SCREEN_H)
            scale = max(UI_SCALE_MIN, min(scale, UI_SCALE_MAX))
    _UI_SCALE = scale
    return _UI_SCALE


def sp(px) -> int:
    """按显示器缩放像素值(用于控件尺寸、间距、边距)。"""
    return max(1, int(round(px * ui_scale())))


def sf(pt, factor: float = 1.0) -> int:
    """按显示器缩放字号，下限 7 保证小屏可读。"""
    return max(7, int(round(pt * ui_scale() * factor)))


def clamp_to_screen(w: int, h: int):
    """把尺寸限制在显示器可用区域内(留 6% 余量给任务栏/标题栏)。"""
    screen = QtWidgets.QApplication.primaryScreen()
    if screen is None:
        return w, h
    avail = screen.availableGeometry()
    return (min(w, int(avail.width() * 0.94)),
            min(h, int(avail.height() * 0.94)))


class PressureDiagram(Node, QtWidgets.QMainWindow):
    def __init__(self):
        # 先调用 Node 的 __init__，避免 super() 歧义
        Node.__init__(self, 'pressure_diagram')
        QtWidgets.QMainWindow.__init__(self)

        # 键名与协议传感器一致(1拇指~5小指，6手掌另算)
        self.fingers_matrix = ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'pinky_matrix']
        self.fingers_mass = ['thumb_mass', 'index_mass', 'middle_mass', 'ring_mass', 'pinky_mass']

        # 掌心(palm)为可选通道：数据不存在，或持续无效(-1)时不显示
        self.palm_mass_key = 'palm_mass'
        self.palm_matrix_key = 'palm_matrix'
        self.palm_hide_frames = 5  # 连续多少帧无效则隐藏

        self.finger_colors = [
            (255, 75, 75),    # Thumb - Red
            (50, 205, 50),    # Index - Green
            (65, 105, 255),   # Middle - Blue
            (255, 215, 0),    # Ring - Gold
            (218, 112, 214)   # Pinky - Purple
        ]
        self.palm_color = (34, 211, 238)  # Palm - Cyan

        # 波形图数据（含 palm）
        self.wave_data = {f: np.zeros(100) for f in self.fingers_mass}
        self.wave_data[self.palm_mass_key] = np.zeros(100)
        self.current_wave_values = {f: 0.0 for f in self.fingers_mass}
        self.current_wave_values[self.palm_mass_key] = 0.0

        # palm 波形显示状态
        self.palm_wave_present = False      # 最近一帧是否包含 palm_mass
        self.palm_wave_neg_streak = 0       # palm_mass 连续为 -1 的帧数
        self.palm_wave_plot = None          # palm 波形子图（按需创建）

        # 热力图数据 - 行列数完全由话题数据决定，这里只放占位形状
        self.matrix_data = {f: np.zeros(PLACEHOLDER_MATRIX_SHAPE) for f in self.fingers_matrix}
        self.matrix_shapes = {f: PLACEHOLDER_MATRIX_SHAPE for f in self.fingers_matrix}  # (rows, cols)
        self.data_received = {f: False for f in self.fingers_matrix}

        # palm 热力图数据与显示状态（尺寸同样来自数据）
        self.matrix_data[self.palm_matrix_key] = np.zeros(PLACEHOLDER_MATRIX_SHAPE)
        self.matrix_shapes[self.palm_matrix_key] = PLACEHOLDER_MATRIX_SHAPE
        self.data_received[self.palm_matrix_key] = False
        self.palm_matrix_present = False    # 最近一帧是否包含 palm_matrix
        self.palm_matrix_neg_streak = 0     # palm_matrix 全为 -1 的连续帧数
        self.palm_heat_container = None     # palm 热力图容器（预创建，按需显隐）

        self.wave_sub = None
        self.matrix_sub = None
        self.wave_frames = 0        # 已收到的合力帧数(用于左右手自动探测)
        self.current_side = 'left'

        # hand_type: left/right 指定初始订阅哪只手；auto 按已有话题自动选
        self.declare_parameter('hand_type', 'auto')

        # ROS2 节流计时器
        self.last_matrix_log_time = {f: None for f in self.fingers_matrix}
        self.last_matrix_log_time[self.palm_matrix_key] = None

        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.qos_profile = qos_profile

        # 顺序很关键：先订阅并抓一帧点阵，拿到真实 rows×cols 后再建界面，
        # 这样热力图视图的纵横比/图像尺寸一开始就是设备的真实形状。
        self.setup_ros()
        self.prefetch_matrix_shapes()
        self.init_ui()

    def prefetch_matrix_shapes(self):
        """建界面前先 spin 一小会儿，抓一帧点阵数据拿到真实行列数。

        订阅到已在运行的发布者需要经过 DDS 发现，所以这里最多等
        SHAPE_PREFETCH_TIMEOUT 秒；自动模式下这只手没数据就换另一只再等一轮。
        全都超时(设备未启动/未开点阵)就先用占位形状，之后每帧刷新仍会按数据里的
        rows×cols 自适应，不会卡住界面。
        """
        got = self._spin_for_shapes(SHAPE_PREFETCH_TIMEOUT)
        if not got and self.auto_side:
            other = 'right' if self.current_side == 'left' else 'left'
            self.get_logger().info(
                f"{self.current_side} 手无点阵数据，改试 {other} 手")
            self._subscribe_side(other)
            got = self._spin_for_shapes(SHAPE_PREFETCH_TIMEOUT)

        shapes = {f: self.matrix_shapes[f] for f in self.fingers_matrix
                  if self.data_received[f]}
        if shapes:
            self.get_logger().info(
                "点阵形状(来自话题): " +
                ", ".join(f"{f.replace('_matrix', '')}={r}×{c}"
                          for f, (r, c) in shapes.items()))
        else:
            self.get_logger().warn(
                "未收到点阵数据，先用占位形状，收到后自动按真实行列数刷新")

    def _spin_for_shapes(self, timeout):
        """spin 到收齐 5 个手指的点阵或超时；返回是否拿到过数据。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.data_received[f] for f in self.fingers_matrix):
                break
        return any(self.data_received[f] for f in self.fingers_matrix)

    def init_ui(self):
        self.setWindowTitle("Robotic Hand Sensor Fusion Interface")
        self.setStyleSheet("background-color: #0f172a;")

        # 热力图视图的设计基准尺寸，窗口缩放时以此为基础重算
        self._matrix_base_sizes = {}
        self._matrix_labels = {}
        self._matrix_label_colors = {}

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QHBoxLayout(central_widget)
        main_layout.setSpacing(sp(15))
        main_layout.setContentsMargins(sp(10), sp(10), sp(10), sp(10))

        # ============== 左侧：波形图 ==============
        left_widget = self.create_waveform_panel()
        main_layout.addWidget(left_widget, stretch=1)

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setStyleSheet(
            f"QFrame {{ background-color: #334155; max-width: {sp(2)}px; }}")
        main_layout.addWidget(line)

        # ============== 右侧：热力图 ==============
        right_widget = self.create_heatmap_panel()
        main_layout.addWidget(right_widget, stretch=1)

        # 下拉框对齐到实际订阅的话题(可能被 hand_type/自动探测改过)
        self._sync_combos()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)

        # 默认以布局允许的最小尺寸打开窗口
        self._resize_to_minimum()

    def _resize_to_minimum(self):
        """把窗口收缩到布局允许的最小尺寸。

        用设计基准(grow=1.0)下的视图最小尺寸来求窗口最小值：若直接用当前
        grow，下限 MATRIX_GROW_MIN 会反过来压缩视图最小尺寸，窗口会一路
        塌缩到热力图无法辨识的程度。
        """
        for finger in self._matrix_base_sizes:
            self._apply_single_matrix_view_scale(finger, 1.0)
        central = self.centralWidget()
        if central is not None and central.layout() is not None:
            central.layout().activate()
        hint = self.minimumSizeHint()
        self.resize(*clamp_to_screen(hint.width(), hint.height()))
        # 定稿尺寸后再按实际窗口大小刷新一次，之后允许用户手动拉得更小
        self._last_grow = -1.0
        self._apply_matrix_view_scale()

    def create_waveform_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setSpacing(sp(8))

        wave_ctrl = QtWidgets.QFrame()
        wave_ctrl.setStyleSheet(
            "QFrame { background-color: #1e293b; border: %dpx solid #3b82f6;"
            " border-radius: %dpx; }" % (max(1, sp(2)), sp(4)))
        wave_layout = QtWidgets.QHBoxLayout(wave_ctrl)
        wave_layout.setContentsMargins(sp(12), sp(6), sp(12), sp(6))

        title = QtWidgets.QLabel("WAVEFORM MONITOR (30Hz)")
        title.setStyleSheet(
            f"color: #60a5fa; font-weight: bold; font-size: {sf(13)}px;")

        self.wave_combo = QtWidgets.QComboBox()
        self.wave_combo.addItems([
            "/cb_left_hand_matrix_touch_mass",
            "/cb_right_hand_matrix_touch_mass"
        ])
        self.wave_combo.setStyleSheet(f"""
            QComboBox {{ color: white; background-color: #334155; border: 1px solid #475569; border-radius: {sp(4)}px; padding: {sp(4)}px {sp(8)}px; min-width: {sp(260)}px; font-size: {sf(12)}px; }}
            QComboBox QAbstractItemView {{ background-color: #1e293b; color: white; selection-background-color: #3b82f6; font-size: {sf(12)}px; }}
        """)
        self.wave_combo.currentTextChanged.connect(self.switch_wave_topic)
        
        wave_layout.addWidget(title)
        wave_layout.addWidget(self.wave_combo)
        wave_layout.addStretch()
        layout.addWidget(wave_ctrl)
        
        self.wave_widget = pg.GraphicsLayoutWidget()
        self.wave_widget.setBackground('#0f172a')
        layout.addWidget(self.wave_widget, stretch=1)
        
        self.wave_plots = {}
        self.wave_curves = {}

        for i, finger in enumerate(self.fingers_mass):
            # 默认最后一个指头显示 Time 轴（若 palm 出现，会自动移交给 palm）
            self._add_wave_plot(i, finger, self.finger_colors[i],
                                show_time=(i == len(self.fingers_mass) - 1))

        return panel

    def _set_wave_time_axis(self, plot, show_time):
        """控制某个波形子图底部 Time 轴的显示（仅最底部子图显示）"""
        axis = plot.getAxis('bottom')
        if show_time:
            axis.setStyle(showValues=True)
            axis.setLabel('Time', color='#64748b')
            axis.showLabel(True)
        else:
            axis.setStyle(showValues=False)
            axis.showLabel(False)

    def _add_wave_plot(self, row, finger, color, show_time):
        """创建一个波形子图并登记曲线/数值文本，供手指与 palm 复用"""
        p = self.wave_widget.addPlot(row=row, col=0)
        p.setMenuEnabled(False)
        p.setMouseEnabled(x=False, y=False)
        p.setYRange(0, 4500)
        p.setXRange(0, 100)
        p.showGrid(x=True, y=True, alpha=0.3)
        p.getAxis('left').setTextPen('#64748b')
        p.getAxis('bottom').setTextPen('#64748b')

        # 坐标轴刻度字体与预留宽度随显示器缩放，否则高分屏上刻度数字过小
        tick_font = QtGui.QFont("Arial", sf(8))
        for side in ('left', 'bottom'):
            axis = p.getAxis(side)
            axis.setStyle(tickFont=tick_font, tickLength=-sp(5))
        p.getAxis('left').setWidth(sp(42))

        qcolor = QtGui.QColor(*color)
        display_name = finger.replace('_mass', '').upper()
        p.setTitle(f"[ {display_name} ]", color=qcolor, size=f"{sf(11)}pt")
        self._set_wave_time_axis(p, show_time)

        pen = pg.mkPen(color=color, width=2.5 * ui_scale())
        curve = p.plot(pen=pen)
        self.wave_curves[finger] = curve

        text = pg.TextItem(text="0", color=(255, 255, 255), anchor=(1, 0.5))
        text.setFont(QtGui.QFont("Arial", sf(10), QtGui.QFont.Bold))
        p.addItem(text)
        self.wave_curves[finger + '_text'] = text

        self.wave_plots[finger] = p
        return p

    def _create_palm_wave_plot(self):
        """按需创建 palm 波形子图（置于最底部，并接管 Time 轴）"""
        if self.palm_wave_plot is not None:
            return
        last_finger = self.fingers_mass[-1]
        if last_finger in self.wave_plots:
            self._set_wave_time_axis(self.wave_plots[last_finger], False)
        row = len(self.fingers_mass)
        self.palm_wave_plot = self._add_wave_plot(
            row, self.palm_mass_key, self.palm_color, show_time=True)

    def _remove_palm_wave_plot(self):
        """移除 palm 波形子图并把 Time 轴交还给最后一个指头"""
        if self.palm_wave_plot is None:
            return
        self.wave_widget.removeItem(self.palm_wave_plot)
        self.palm_wave_plot = None
        self.wave_curves.pop(self.palm_mass_key, None)
        self.wave_curves.pop(self.palm_mass_key + '_text', None)
        self.wave_plots.pop(self.palm_mass_key, None)
        last_finger = self.fingers_mass[-1]
        if last_finger in self.wave_plots:
            self._set_wave_time_axis(self.wave_plots[last_finger], True)

    def create_heatmap_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setSpacing(sp(8))

        heat_ctrl = QtWidgets.QFrame()
        heat_ctrl.setStyleSheet(
            "QFrame { background-color: #1e293b; border: %dpx solid #f43f5e;"
            " border-radius: %dpx; }" % (max(1, sp(2)), sp(4)))
        heat_layout = QtWidgets.QHBoxLayout(heat_ctrl)
        heat_layout.setContentsMargins(sp(12), sp(6), sp(12), sp(6))

        title = QtWidgets.QLabel(self._matrix_title_text())
        title.setStyleSheet(
            f"color: #f43f5e; font-weight: bold; font-size: {sf(13)}px;")
        self.matrix_title = title

        # 单位标签
        unit_label = QtWidgets.QLabel("Max Value (g)")
        unit_label.setStyleSheet(
            f"color: #94a3b8; font-size: {sf(11)}px; border: none; background: transparent;")
        heat_layout.addWidget(unit_label)
        heat_layout.addSpacing(sp(10))

        self.matrix_combo = QtWidgets.QComboBox()
        self.matrix_combo.addItems([
            "/cb_left_hand_matrix_touch",
            "/cb_right_hand_matrix_touch"
        ])
        self.matrix_combo.setStyleSheet(f"""
            QComboBox {{ color: white; background-color: #334155; border: 1px solid #475569; border-radius: {sp(4)}px; padding: {sp(4)}px {sp(8)}px; min-width: {sp(260)}px; font-size: {sf(12)}px; }}
            QComboBox QAbstractItemView {{ background-color: #1e293b; color: white; selection-background-color: #f43f5e; font-size: {sf(12)}px; }}
        """)
        self.matrix_combo.currentTextChanged.connect(self.switch_matrix_topic)

        heat_layout.addWidget(title)
        heat_layout.addWidget(self.matrix_combo)
        heat_layout.addStretch()
        layout.addWidget(heat_ctrl)

        heat_container = QtWidgets.QWidget()
        heat_grid = QtWidgets.QGridLayout(heat_container)
        heat_grid.setSpacing(sp(10))
        heat_grid.setContentsMargins(sp(5), sp(5), sp(5), sp(5))

        self.matrix_plots = {}
        self.matrix_images = {}
        self.matrix_peaks = {}

        # 网格位置与 fingers_matrix 一一对应(3 + 2，第 3 格留给颜色条)
        grid_pos = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1)]
        positions = list(zip(self.fingers_matrix, grid_pos))

        self.colormap = pg.colormap.get('plasma')

        for idx, (finger, (row, col)) in enumerate(positions):
            container = self._create_matrix_view(
                finger, self.finger_colors[idx],
                finger.replace('_matrix', '').upper(),
                sizes=(240, 280, 120, 160)
            )
            heat_grid.addWidget(container, row, col)

        # 颜色条
        legend_widget = QtWidgets.QWidget()
        legend_layout = QtWidgets.QVBoxLayout(legend_widget)
        legend_layout.setAlignment(QtCore.Qt.AlignCenter)

        lbl_max = QtWidgets.QLabel("MAX")
        lbl_max.setStyleSheet(f"color: #fbbf24; font-size: {sf(9)}px;")
        lbl_max.setAlignment(QtCore.Qt.AlignCenter)

        gradient = pg.GradientWidget(orientation='right')
        gradient.setMaximumWidth(sp(25))
        gradient.setMaximumHeight(sp(180))
        gradient.setColorMap(self.colormap)

        lbl_min = QtWidgets.QLabel("0")
        lbl_min.setStyleSheet(f"color: #3b82f6; font-size: {sf(9)}px;")
        lbl_min.setAlignment(QtCore.Qt.AlignCenter)

        legend_layout.addWidget(lbl_max)
        legend_layout.addWidget(gradient, stretch=1)
        legend_layout.addWidget(lbl_min)

        heat_grid.addWidget(legend_widget, 1, 2)
        self.gradient_widget = gradient

        # palm 热力图（尺寸不固定），单独占一行并横跨三列；默认隐藏，
        # 只有收到有效 palm_matrix 数据时才显示
        self.palm_heat_container = self._create_matrix_view(
            self.palm_matrix_key, self.palm_color, 'PALM',
            sizes=(520, 340, 200, 160)
        )
        self.palm_heat_container.setVisible(False)
        heat_grid.addWidget(self.palm_heat_container, 2, 0, 1, 3)

        self.heat_grid = heat_grid

        layout.addWidget(heat_container, stretch=1)
        return panel

    def _matrix_title_text(self):
        """标题里的行列数取自实际收到的点阵，不同手指尺寸不一致时逐个列出。"""
        shapes = [self.matrix_shapes[f] for f in self.fingers_matrix
                  if self.data_received[f]]
        uniq = set(shapes)
        if not uniq:
            return "PRESSURE MATRIX (waiting for data)"
        if len(uniq) == 1:
            rows, cols = uniq.pop()
            return f"PRESSURE MATRIX ({rows}×{cols})"
        return "PRESSURE MATRIX (" + " / ".join(
            f"{r}×{c}" for r, c in sorted(uniq)) + ")"

    def _create_matrix_view(self, finger, color, name, sizes):
        """创建一个热力图子视图并登记，供手指与 palm 复用。

        sizes = (max_w, max_h, min_w, min_h)，为 1920x1080 下的设计基准值，
        实际尺寸由 _apply_matrix_view_scale 按显示器与窗口大小换算。
        """
        container = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(container)
        vbox.setSpacing(sp(2))
        vbox.setContentsMargins(0, 0, 0, 0)

        name_label = QtWidgets.QLabel(name)
        name_label.setAlignment(QtCore.Qt.AlignCenter)
        color_hex = '#{:02x}{:02x}{:02x}'.format(*color)
        name_label.setStyleSheet(
            f"color: {color_hex}; font-weight: bold; font-size: {sf(12)}px;")
        vbox.addWidget(name_label)

        view = pg.PlotWidget()
        view.setMenuEnabled(False)
        view.setMouseEnabled(x=False, y=False)
        view.setBackground('#0f172a')
        view.hideAxis('left')
        view.hideAxis('bottom')

        # 初始形状取自已收到的话题数据(prefetch)，没数据时才是占位形状；
        # 之后每帧刷新都会按 matrix_shapes 重新锁定纵横比
        rows, cols = self.matrix_shapes.get(finger, PLACEHOLDER_MATRIX_SHAPE)
        view.setAspectLocked(True, ratio=cols / rows)

        img = pg.ImageItem()
        img.setLookupTable(self.colormap.getLookupTable())
        img.setImage(np.zeros((rows, cols)), levels=[0, 100])
        view.addItem(img)

        peak_text = pg.TextItem(text="", color=(255, 255, 255), anchor=(0.5, 0.5))
        peak_text.setFont(QtGui.QFont("Arial", sf(10), QtGui.QFont.Bold))
        view.addItem(peak_text)

        vbox.addWidget(view, stretch=1)

        self.matrix_plots[finger] = view
        self.matrix_images[finger] = img
        self.matrix_peaks[finger] = peak_text
        self._matrix_base_sizes[finger] = sizes
        self._matrix_labels[finger] = name_label
        self._matrix_label_colors[finger] = color_hex
        # 立即按当前显示器缩放一次基准尺寸
        self._apply_single_matrix_view_scale(finger, 1.0)
        return container

    # ---- 热力图显示区域缩放适配 --------------------------------------
    def _matrix_grow_factor(self) -> float:
        """当前窗口相对设计基准窗口(1600x900 缩放后)的尺寸系数。

        窗口最大化或用户拉大时热力图跟着变大，缩小时同步收缩，
        不再被固定的 setMaximumSize 卡死。上限由 MATRIX_GROW_MAX 控制。
        """
        base_w = max(1, sp(BASE_WINDOW_W))
        base_h = max(1, sp(BASE_WINDOW_H))
        g = min(self.width() / base_w, self.height() / base_h)
        return max(MATRIX_GROW_MIN, min(g, MATRIX_GROW_MAX))

    def _apply_single_matrix_view_scale(self, finger, grow: float):
        max_w, max_h, min_w, min_h = self._matrix_base_sizes[finger]
        view = self.matrix_plots[finger]
        view.setMaximumSize(sp(max_w * grow), sp(max_h * grow))
        # 最小尺寸只跟随收缩，避免窗口变大时最小尺寸把布局顶开
        shrink = min(1.0, grow)
        view.setMinimumSize(sp(min_w * shrink), sp(min_h * shrink))

        # 文字按 sqrt(grow) 次线性增长：视图变大时字号只温和跟随，
        # 避免峰值数字和标签盖住热力图本身
        font_grow = grow ** 0.5

        label = self._matrix_labels.get(finger)
        if label is not None:
            color_hex = self._matrix_label_colors[finger]
            label.setStyleSheet(
                f"color: {color_hex}; font-weight: bold; font-size: {sf(12, font_grow)}px;")

        peak = self.matrix_peaks.get(finger)
        if peak is not None:
            peak.setFont(QtGui.QFont("Arial", sf(10, font_grow), QtGui.QFont.Bold))

    def _apply_matrix_view_scale(self):
        """窗口尺寸变化时重算所有热力图视图与其文字的大小。"""
        if not getattr(self, '_matrix_base_sizes', None):
            return
        grow = self._matrix_grow_factor()
        if abs(grow - getattr(self, '_last_grow', -1.0)) < 0.02:
            return  # 变化太小则跳过，避免 resize 过程中反复重排
        self._last_grow = grow
        for finger in self._matrix_base_sizes:
            self._apply_single_matrix_view_scale(finger, grow)
        if getattr(self, 'gradient_widget', None) is not None:
            self.gradient_widget.setMaximumWidth(sp(25 * grow))
            self.gradient_widget.setMaximumHeight(sp(180 * grow))

    def resizeEvent(self, event):
        QtWidgets.QMainWindow.resizeEvent(self, event)
        self._apply_matrix_view_scale()

    def showEvent(self, event):
        QtWidgets.QMainWindow.showEvent(self, event)
        handle = self.windowHandle()
        if handle is not None and not getattr(self, '_screen_hooked', False):
            handle.screenChanged.connect(self._on_screen_changed)
            self._screen_hooked = True
        self._apply_matrix_view_scale()

    def _on_screen_changed(self, *_):
        """窗口被拖到另一台分辨率不同的显示器时重新适配。"""
        ui_scale(refresh=True)
        self._last_grow = -1.0
        self._apply_matrix_view_scale()

    def setup_ros(self):
        """只建订阅，不碰界面(此时下拉框还没创建)。"""
        self.get_logger().info("Setting up ROS...")
        side = str(self.get_parameter('hand_type').value).lower()
        self.auto_side = side not in ('left', 'right')
        if self.auto_side:
            side = self._detect_hand_side() or 'left'
        self._subscribe_side(side)

    def _detect_hand_side(self):
        """看当前有哪只手在发合力话题，两只都有时优先左手。"""
        try:
            names = {n for n, _ in self.get_topic_names_and_types()}
        except Exception:
            return None
        for side in ('left', 'right'):
            if f"/cb_{side}_hand_matrix_touch_mass" in names:
                return side
        return None

    def _subscribe_side(self, side):
        self.current_side = side
        self.subscribe_waveform(f"/cb_{side}_hand_matrix_touch_mass")
        self.subscribe_matrix(f"/cb_{side}_hand_matrix_touch")

    def _sync_combos(self):
        """界面建好后把下拉框对齐到实际订阅的话题(屏蔽信号，避免重复订阅)。"""
        for combo, topic in (
                (self.wave_combo, f"/cb_{self.current_side}_hand_matrix_touch_mass"),
                (self.matrix_combo, f"/cb_{self.current_side}_hand_matrix_touch")):
            combo.blockSignals(True)
            combo.setCurrentText(topic)
            combo.blockSignals(False)

    def subscribe_waveform(self, topic):
        if self.wave_sub:
            self.destroy_subscription(self.wave_sub)
        self.wave_sub = self.create_subscription(String, topic, self.wave_callback, self.qos_profile)
        self.get_logger().info(f"WAVEFORM: {topic}")

    def subscribe_matrix(self, topic):
        if self.matrix_sub:
            self.destroy_subscription(self.matrix_sub)
        self.matrix_sub = self.create_subscription(String, topic, self.matrix_callback, self.qos_profile)
        self.get_logger().info(f"MATRIX: {topic}")

    def switch_wave_topic(self, topic):
        self.subscribe_waveform(topic)
        self.wave_frames = 0
        self.current_side = 'right' if '_right_' in topic else 'left'
        for f in self.fingers_mass:
            self.wave_data[f] = np.zeros(100)
            self.current_wave_values[f] = 0.0
        # 重置 palm 波形状态并隐藏
        self.wave_data[self.palm_mass_key] = np.zeros(100)
        self.current_wave_values[self.palm_mass_key] = 0.0
        self.palm_wave_present = False
        self.palm_wave_neg_streak = 0
        self._remove_palm_wave_plot()

    def switch_matrix_topic(self, topic):
        self.subscribe_matrix(topic)
        self.current_side = 'right' if '_right_' in topic else 'left'
        # 形状保留上一帧的(等新话题第一帧来了再按其 rows×cols 更新)，
        # 只把数值清零，避免切换瞬间残留上一只手的图像
        for f in list(self.fingers_matrix) + [self.palm_matrix_key]:
            self.matrix_data[f] = np.zeros(self.matrix_shapes[f])
            self.data_received[f] = False
            img = getattr(self, 'matrix_images', {}).get(f)
            if img is not None:
                img.setImage(self.matrix_data[f], autoLevels=False, levels=[0, 1])
            peak = getattr(self, 'matrix_peaks', {}).get(f)
            if peak is not None:
                peak.setText("")
        # 重置 palm 显示状态并隐藏
        self.palm_matrix_present = False
        self.palm_matrix_neg_streak = 0
        if self.palm_heat_container is not None:
            self.palm_heat_container.setVisible(False)

    def wave_callback(self, msg):
        """解析合力话题：sensors[编号] -> force[FnS] 或 cell_sum。"""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        sensors = data.get('sensors')
        if not isinstance(sensors, dict):
            return
        self.wave_frames += 1

        palm_val = None
        for sid, entry in sensors.items():
            if not isinstance(entry, dict):
                continue
            key = sensor_key(sid, entry)
            if key is None:
                continue
            val = sensor_mass(entry)
            if val is None:
                continue
            if key == 'palm':
                palm_val = val
            elif key + '_mass' in self.current_wave_values:
                self.current_wave_values[key + '_mass'] = val

        # palm 可选：数据里没有则不显示；值为 -1 视为无效帧
        if palm_val is None:
            self.palm_wave_present = False
        else:
            self.palm_wave_present = True
            self.current_wave_values[self.palm_mass_key] = palm_val
            if palm_val == -1:
                self.palm_wave_neg_streak += 1
            else:
                self.palm_wave_neg_streak = 0

    def matrix_callback(self, msg):
        """解析点阵话题：sensors[编号] -> {rows, cols, matrix}，形状随数据自适应。"""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        sensors = data.get('sensors')
        if not isinstance(sensors, dict):
            return

        palm_arr = None
        for sid, entry in sensors.items():
            if not isinstance(entry, dict):
                continue
            key = sensor_key(sid, entry)
            if key is None:
                continue
            arr = self._to_matrix(entry)
            if arr is None:
                continue
            if key == 'palm':
                palm_arr = arr
                continue
            finger = key + '_matrix'
            if finger not in self.matrix_data:
                continue
            self.matrix_data[finger] = arr
            self.matrix_shapes[finger] = arr.shape
            self.data_received[finger] = True

            # ROS2 节流日志
            now = self.get_clock().now()
            last = self.last_matrix_log_time[finger]
            if last is None or (now - last).nanoseconds > 3e9:
                self.last_matrix_log_time[finger] = now
                self.get_logger().info(
                    f"{finger}: {arr.shape[0]}×{arr.shape[1]}, max={arr.max():.1f}")

        # palm 可选：数据里没有则不显示；全为 -1 视为无效帧
        pk = self.palm_matrix_key
        if palm_arr is None:
            self.palm_matrix_present = False
        else:
            self.palm_matrix_present = True
            self.matrix_data[pk] = palm_arr
            self.matrix_shapes[pk] = palm_arr.shape
            self.data_received[pk] = True
            if np.all(palm_arr == -1):
                self.palm_matrix_neg_streak += 1
            else:
                self.palm_matrix_neg_streak = 0

    def _to_matrix(self, entry):
        """把一项 sensors 数据取成二维数组；一维 cells 按 rows/cols 折行。"""
        raw = entry.get('matrix')
        if raw is None:
            raw = entry.get('cells')
        if raw is None:
            return None
        try:
            arr = np.array(raw, dtype=np.float32)
        except (ValueError, TypeError):
            return None
        if arr.ndim == 2 and arr.size:
            return arr
        if arr.ndim == 1 and arr.size:
            rows, cols = entry.get('rows'), entry.get('cols')
            if isinstance(rows, int) and isinstance(cols, int) \
                    and rows > 0 and cols > 0 and rows * cols == arr.size:
                return arr.reshape(rows, cols)
        return None

    def _update_single_heatmap(self, finger):
        """刷新单个热力图（供手指与 palm 复用）"""
        if not self.data_received[finger]:
            return

        data = self.matrix_data[finger]
        rows, cols = self.matrix_shapes[finger]
        img = self.matrix_images[finger]
        view = self.matrix_plots[finger]
        peak_text = self.matrix_peaks[finger]

        max_val = data.max()

        if max_val > 0:
            display_data = data / max_val if max_val > 1 else data
        else:
            display_data = data

        # **关键修复**: 垂直翻转，使第0行(ROS数据第1行)显示在顶部
        display_data_flipped = np.flipud(display_data)
        img.setImage(display_data_flipped, autoLevels=False, levels=[0, 1])

        # palm 尺寸不固定，按当前实际形状锁定纵横比并设置视图范围
        view.setAspectLocked(True, ratio=cols / rows)
        view.setRange(xRange=(-0.5, cols - 0.5), yRange=(-0.5, rows - 0.5))

        # 显示数值：矩阵中的最大压力值，单位g(克)
        if max_val > 0:
            max_idx = np.unravel_index(np.argmax(data), data.shape)
            peak_text.setText(f"{max_val:.0f}")
            # 坐标转换：因为图像翻转了，y坐标也要翻转
            flipped_row = (rows - 1) - max_idx[0]
            peak_text.setPos(max_idx[1], flipped_row)
        else:
            peak_text.setText("0")

    def update_display(self):
        # ===== 波形图 =====
        # 根据最新数据决定 palm 波形是否显示（不存在或连续 palm_hide_frames 帧为 -1 则隐藏）
        show_palm_wave = self.palm_wave_present and \
            self.palm_wave_neg_streak < self.palm_hide_frames
        if show_palm_wave and self.palm_wave_plot is None:
            self._create_palm_wave_plot()
        elif not show_palm_wave and self.palm_wave_plot is not None:
            self._remove_palm_wave_plot()

        wave_fingers = list(self.fingers_mass)
        if self.palm_wave_plot is not None:
            wave_fingers.append(self.palm_mass_key)

        for finger in wave_fingers:
            self.wave_data[finger] = np.roll(self.wave_data[finger], -1)
            self.wave_data[finger][-1] = self.current_wave_values[finger]
            self.wave_curves[finger].setData(self.wave_data[finger])

            val = self.current_wave_values[finger]
            self.wave_curves[finger + '_text'].setText(f"{val:.0f}")
            self.wave_curves[finger + '_text'].setPos(99, val)

        # ===== 热力图 =====
        for finger in self.fingers_matrix:
            self._update_single_heatmap(finger)

        # 标题里的行列数跟随实际数据
        title_text = self._matrix_title_text()
        if self.matrix_title.text() != title_text:
            self.matrix_title.setText(title_text)

        # palm 热力图显隐同步（不存在或连续 palm_hide_frames 帧全为 -1 则隐藏）
        show_palm_matrix = self.palm_matrix_present and \
            self.palm_matrix_neg_streak < self.palm_hide_frames
        if self.palm_heat_container is not None:
            self.palm_heat_container.setVisible(show_palm_matrix)
        if show_palm_matrix:
            self._update_single_heatmap(self.palm_matrix_key)

def signal_handler(sig, frame):
    QtWidgets.QApplication.quit()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    # 启用高分辨率(HiDPI)缩放适配，必须在创建 QApplication 之前设置
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    # 支持 125%/150% 等非整数缩放(Qt >= 5.14)，避免向下取整导致界面偏小
    if hasattr(QtCore.Qt, 'HighDpiScaleFactorRoundingPolicy'):
        QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
            QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    # 按显示器缩放全局默认字体
    scale = ui_scale(refresh=True)
    font = app.font()
    pt = font.pointSizeF()
    if pt <= 0:
        pt = 9.0
    font.setPointSizeF(round(pt * scale, 1))
    app.setFont(font)

    signal.signal(signal.SIGINT, signal_handler)

    gui = PressureDiagram()
    gui.get_logger().info(f"UI scale factor: {scale:.2f}")
    gui.show()

    # ROS2 spin 在 Qt 定时器中处理
    def ros_spin():
        rclpy.spin_once(gui, timeout_sec=0)

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(ros_spin)
    ros_timer.start(1)  # ~1000Hz

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
