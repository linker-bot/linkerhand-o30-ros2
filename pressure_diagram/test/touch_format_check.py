#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""触觉话题解析/渲染自检（不需要真实设备，也不需要显示器）。

用法：
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/linkerhand-o30-ros2/pressure_diagram/test/touch_format_check.py

用合成数据喂 wave_callback/matrix_callback，验证界面完全按话题里的
sensors[编号].rows/cols 自适应，不依赖任何写死的行列数。
"""
import os, json
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import numpy as np, rclpy
from PyQt5 import QtWidgets
from std_msgs.msg import String
from pressure_diagram.pressure_diagram import PressureDiagram

NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']
fails = []
def ck(c, label):
    print(('  OK  ' if c else '  FAIL ') + label)
    if not c: fails.append(label)

rclpy.init()
app = QtWidgets.QApplication([])
gui = PressureDiagram()

# --- 1) 任意形状的 2D matrix（4×3，与设备的 10×7 无关）------------------
rows, cols = 4, 3
m = [[0] * cols for _ in range(rows)]
m[1][2] = 200          # 第2行(索引1)、第3列
sensors = {str(i): {"name": n, "rows": rows, "cols": cols, "matrix": m}
           for i, n in enumerate(NAMES, 1)}
# palm(编号6) 用另一个形状，验证 palm 独立自适应
palm = [[7, 8, 9, 10]]
sensors['6'] = {"name": "palm", "rows": 1, "cols": 4, "matrix": palm}
gui.matrix_callback(String(data=json.dumps({"stamp": 1, "hand_type": "right",
                                            "sensors": sensors})))
mass = {str(i): {"name": n, "unit": "g", "force": {"FnS": 100 + i},
                 "cell_sum": 5, "cell_max": 3} for i, n in enumerate(NAMES, 1)}
mass['6'] = {"name": "palm", "unit": "g", "force": {}, "cell_sum": 42}  # 无 FnS -> 退回 cell_sum
gui.wave_callback(String(data=json.dumps({"stamp": 1, "hand_type": "right",
                                          "sensors": mass})))
gui.update_display()

ck(all(gui.matrix_shapes[f] == (rows, cols) for f in gui.fingers_matrix),
   f"5指形状跟随数据 -> {gui.matrix_shapes['thumb_matrix']}")
ck(gui.matrix_shapes['palm_matrix'] == (1, 4),
   f"palm 形状独立跟随数据 -> {gui.matrix_shapes['palm_matrix']}")
img = gui.matrix_images['thumb_matrix'].image
ck(img.shape == (rows, cols), f"ImageItem 尺寸 = {img.shape}")
ck(gui.matrix_peaks['thumb_matrix'].toPlainText() == "200",
   f"峰值文本 = {gui.matrix_peaks['thumb_matrix'].toPlainText()!r}")
ck(np.argmax(img) == np.argmax(np.flipud(np.array(m))), "峰值位置随翻转对齐")
ck(gui.matrix_title.text() == f"PRESSURE MATRIX ({rows}×{cols})",
   f"标题 = {gui.matrix_title.text()!r}")
ck([gui.current_wave_values[f] for f in gui.fingers_mass] == [101, 102, 103, 104, 105],
   "波形取 force[FnS]")
ck(gui.current_wave_values['palm_mass'] == 42, "force 为空时退回 cell_sum")
ck(gui.palm_wave_plot is not None, "palm 波形子图已创建")
# 窗口没 show()，isVisible() 恒为假，故用 isHidden() 判显式隐藏
ck(not gui.palm_heat_container.isHidden(), "palm 热力图未被隐藏")
ck(all(gui.wave_curves[f].getData()[1][-1] == gui.current_wave_values[f]
       for f in gui.fingers_mass), "曲线末点=当前值")

# --- 2) 只有一维 cells + rows/cols 的情况 -------------------------------
sensors2 = {str(i): {"name": n, "rows": 2, "cols": 3, "cells": [1, 2, 3, 4, 5, 6]}
            for i, n in enumerate(NAMES, 1)}
gui.matrix_callback(String(data=json.dumps({"sensors": sensors2})))
gui.update_display()
ck(gui.matrix_shapes['thumb_matrix'] == (2, 3), "一维 cells 按 rows×cols 折行")
ck(gui.matrix_title.text() == "PRESSURE MATRIX (2×3)", "标题随新形状更新")

# --- 3) palm 消失后应自动隐藏 ------------------------------------------
for _ in range(gui.palm_hide_frames + 1):
    gui.matrix_callback(String(data=json.dumps({"sensors": sensors})))
    gui.wave_callback(String(data=json.dumps({"sensors": mass})))
del sensors['6'], mass['6']
gui.matrix_callback(String(data=json.dumps({"sensors": sensors})))
gui.wave_callback(String(data=json.dumps({"sensors": mass})))
gui.update_display()
ck(gui.palm_heat_container.isHidden() and gui.palm_wave_plot is None,
   "palm 数据消失后隐藏")

# --- 4) 中文名 + 未知编号，靠 name 兜底 --------------------------------
zh = {"9": {"name": "大拇指", "rows": 2, "cols": 2, "matrix": [[1, 2], [3, 4]]}}
gui.matrix_callback(String(data=json.dumps({"sensors": zh})))
ck(gui.matrix_shapes['thumb_matrix'] == (2, 2), "编号不认识时按名称匹配")

# --- 5) 坏数据不应抛异常 ----------------------------------------------
for bad in ['not json', '{}', '{"sensors": null}', '{"sensors": {"1": 5}}',
            '{"sensors": {"1": {"name": "thumb"}}}']:
    gui.matrix_callback(String(data=bad)); gui.wave_callback(String(data=bad))
ck(True, "畸形数据不抛异常")

print("\n结果:", "全部通过" if not fails else f"{len(fails)} 项失败: {fails}")
raise SystemExit(1 if fails else 0)
