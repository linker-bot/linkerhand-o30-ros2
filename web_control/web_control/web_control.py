#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
web_control: 灵巧手 Web 控制台服务器 (通用, 适配多种机械手)。

通用适配约定 (不内置任何型号专属的关节名 / 映射 / 关节数):
  - 关节名称与数量: 全部来自 /cb_<hand>_hand_state 的 name / position 长度
  - 状态名→URDF 关节名映射: 从 URDF 关节名自动推导 (见 derive_state_to_urdf),
    可用 URDF 同目录 <urdf_stem>.map.json 或 map.json 显式覆盖
  - URDF 型号子目录: 取 /cb_<hand>_hand_info 的 model (模糊匹配), auto 且唯一时自动选择
  - 弧度限位: 从生效 URDF 的 <joint><limit> 读取并缓存

功能:
  - 订阅 /cb_<hand>_hand_state (sensor_msgs/JointState, position 0~255)
  - 订阅 /cb_<hand>_hand_control_cmd (sensor_msgs/JointState) — 控制命令回显 ("关闭控制"模式)
  - 发布 /cb_<hand>_hand_control_cmd (sensor_msgs/JointState, SDK 只读 position)
  - 内置 HTTP 服务器 (标准库 ThreadingHTTPServer, 零第三方依赖):
      GET  /           → static/index.html 控制台页面
      GET  /api/status → 节点状态 (手部类型 / 是否收到状态 / 关节名与位置)
      GET  /api/stream → SSE 实时状态流 (限频推送 + 15s 心跳保活, 浏览器自动重连)
      GET  /api/touch  → 最新一帧触觉矩阵 + 合力统计 (前端 ~30Hz 轮询; 热力图用 data, 波形图用 mass 的 force[FnS])
      GET  /api/cmd_state → 控制命令话题 /cb_<hand>_hand_control_cmd 最新一帧 (前端"关闭控制"30Hz 回显)
      POST /api/cmd    → {"positions":[...]} 发布一条控制命令 (0~255, 关节数须等于状态消息)

用法:
  ros2 launch web_control web_control.launch.py
  ros2 launch web_control web_control.launch.py hand:=right
  ros2 launch web_control web_control.launch.py model:=O30
  ros2 launch web_control web_control.launch.py urdf_path:=/path/to/urdf   # 更换 URDF 根目录
  端口无需指定: 从 8090 起自动探测, 被占用则自动顺延 (8091, 8092, ...)
'''
import argparse
import json
import re
import socket
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

U8_MIN, U8_MAX = 0, 255

# ---------------------------------------------------------------------------
# 通用关节名适配 (不内置任何型号专属名称 / 映射 / 关节数):
#   关节名     ← /cb_<hand>_hand_state 的 name (顺序即下标)
#   命令关节数 ← 状态消息 position 的长度 (HandBridge.publish_command)
#   状态名→URDF 关节名映射: 从 URDF 关节名自动推导 (derive_state_to_urdf),
#                  可用 URDF 同目录 <urdf_stem>.map.json 或 map.json 显式覆盖
#   弧度限位   ← 生效 URDF 的 <joint><limit> (读取后按路径缓存)
# 线性映射约定: joint 最小值 → 滑动条 0, joint 最大值 → 滑动条 255
# ---------------------------------------------------------------------------
# 手指同义词 (状态话题与 URDF 对同一手指可能采用不同命名, 统一归一到规范名)
_FINGER_SYNONYMS = {'little': 'pinky'}


def _joint_tokens(name):
    '''关节名 → 小写 token 列表 (如 thumb_roll → [thumb, roll])。'''
    return [t for t in str(name).lower().split('_') if t]


def _norm_token(tok):
    return _FINGER_SYNONYMS.get(tok, tok)


def _finger_of(name):
    '''取关节名首 token 作为「手指」(同义词归一), 无 token 返回 None。'''
    toks = _joint_tokens(name)
    return _norm_token(toks[0]) if toks else None


def _tokens_subseq(sub, full):
    '''sub 的 token 是否按顺序出现在 full 的 token 中 (同义词视为相同 token)。'''
    it = iter(full)
    for s in sub:
        sn = _norm_token(s)
        for f in it:
            if _norm_token(f) == sn:
                break
        else:
            return False
    return True


def derive_state_to_urdf(state_names, urdf_joint_names):
    '''推导 状态话题关节名 → URDF 关节名 映射 (通用启发式, 不内置任何名称)。

    urdf_joint_names 须保持 URDF 文档顺序 (第 3 层按序配对依赖它)。
    三层规则 (按优先级):
      1. 精确同名: 状态名与 URDF 关节名一致 → 直接映射;
      2. token 子序列: 状态名 token (thumb_roll → [thumb, roll]) 按顺序出现在
         某 URDF 关节名 token 中 (thumb_cmc_roll) → 映射; 候选多于 1 个时取
         token 数最接近的, 仍并列则放弃留给第 3 层;
      3. 同指按序: 未映射的状态关节与 URDF 关节按「手指」(首 token) 分组,
         同组数量相等 → 按各自顺序配对 (如 thumb_root1→thumb_mcp,
         thumb_tip→thumb_ip, little_*→pinky_*)。
    返回 {状态名: URDF 关节名}; 无法映射的名称不出现在结果中 (由调用方报缺失)。
    '''
    mapping = {}
    urdf_names = list(urdf_joint_names)
    urdf_set = set(urdf_names)
    for s in state_names:                            # 1. 精确同名
        if s in urdf_set:
            mapping[s] = s
    used = set(mapping.values())
    for s in state_names:                            # 2. token 子序列
        if s in mapping:
            continue
        cands = [u for u in urdf_names
                 if u not in used and _tokens_subseq(_joint_tokens(s),
                                                     _joint_tokens(u))]
        if not cands:
            continue
        best_len = min(len(_joint_tokens(u)) for u in cands)
        best = [u for u in cands if len(_joint_tokens(u)) == best_len]
        if len(best) == 1:
            mapping[s] = best[0]
            used.add(best[0])

    def _group(names):
        groups = {}
        for n in names:
            f = _finger_of(n)
            if f:
                groups.setdefault(f, []).append(n)
        return groups

    s_groups = _group([s for s in state_names if s not in mapping])
    u_groups = _group([u for u in urdf_names if u not in used])
    for f, s_list in s_groups.items():               # 3. 同指按序
        u_list = u_groups.get(f)
        if u_list and len(s_list) == len(u_list):
            for s, u in zip(s_list, u_list):
                mapping[s] = u
                used.add(u)
    return mapping


class HandBridge(Node):
    '''ROS2 桥接节点: 订阅状态 + 发布控制指令, 并缓存最新状态供 Web 层读取。

    线程模型与 gui_control 一致: rclpy 在守护线程里 spin_once,
    HTTP 请求线程直接调用 publish (已验证的线程安全模式)。'''

    def __init__(self, hand_type, model='auto', urdf_base=None):
        super().__init__('web_control_node')
        self.hand_type = hand_type
        self.model_arg = model   # URDF 型号子目录 (auto = 从 info 匹配 / 唯一时自动)
        self.state_topic = f'/cb_{hand_type}_hand_state'
        self.cmd_topic = f'/cb_{hand_type}_hand_control_cmd'
        self.touch_topic = f'/cb_{hand_type}_hand_matrix_touch'
        self.mass_topic = f'/cb_{hand_type}_hand_matrix_touch_mass'

        self._lock = threading.Lock()
        self._latest = None      # {"seq":int, "names":[..], "positions":[..], "t":float}
        self._seq = 0
        self.cmd_count = 0
        self._hand_info = None   # 解析自 /cb_<hand>_hand_info: {hand_type, model, ...}
        self._temperature = []   # 各关节实时温度 °C（顺序同状态话题 / 滑块顺序）
        self._over_temp = []     # 达到 temp_warn 阈值的关节（服务端告警，前端按 >80 单独着色）
        self._touch = None       # 最新一帧矩阵触觉 {"seq":int, "t":float, "data":dict}
        self._touch_seq = 0
        self._mass = None        # 最新一帧合力统计 {"seq":int, "t":float, "data":dict}
        self._mass_seq = 0
        # URDF 根目录: 优先用启动参数 (web_control.launch.py 的 urdf_path, 可为任意目录);
        # 留空则回退到包内置 urdf 目录 (即"现在的目录")
        self.urdf_base = Path(urdf_base).expanduser() if urdf_base \
            else Path(__file__).parent / 'urdf'
        # URDF 弧度限位缓存: {urdf 绝对路径: {joint_name: (min_rad, max_rad)}}
        # 加载 URDF 时读取一次后缓存 (不同 URDF 文件限位不同, 以路径为 key)
        self._joint_limits_cache = {}
        # 状态名→URDF 关节名映射缓存: {(urdf 路径, 关节名元组): mapping}
        self._mapping_cache = {}

        self.cmd_pub = self.create_publisher(JointState, self.cmd_topic, 10)
        self.create_subscription(JointState, self.state_topic, self._state_cb, 10)
        # 控制命令回显订阅 ("关闭控制"模式: 前端 30Hz 轮询, 用该话题最新 position 刷新滑动条)
        self._cmd_latest = None    # {"seq":int, "names":[..], "positions":[..], "t":float}
        self._cmd_seq = 0
        self.cmd_sub = self.create_subscription(JointState, self.cmd_topic,
                                                self._cmd_cb, 10)
        self.create_subscription(String, f'/cb_{hand_type}_hand_info',
                                 self._info_cb, 10)
        # 触觉矩阵点阵 (std_msgs/String JSON): 1~5 手指, 6 手掌 (无该传感器的机型不出现)
        self.create_subscription(String, self.touch_topic, self._touch_cb, 10)
        # 合力话题: 各传感器 force.FnS 等, 前端波形图数据源, 条数=该消息中的传感器数
        self.create_subscription(String, self.mass_topic, self._mass_cb, 10)
        self.get_logger().info(
            f'web_control 启动: 订阅 {self.state_topic}/{self.cmd_topic}, 发布 {self.cmd_topic}')

    def _state_cb(self, msg):
        '''状态回调: 只缓存最新一帧, 不做任何耗时操作。'''
        names = list(msg.name)
        if not names or len(names) != len(msg.position):
            return
        with self._lock:
            self._seq += 1
            self._latest = {
                'seq': self._seq,
                'names': names,
                'positions': [float(p) for p in msg.position],
                't': time.time(),
            }

    def _cmd_cb(self, msg):
        '''控制命令回调: 只缓存最新一帧指令位置(0~255), 供 /api/cmd_state 回显。'''
        names = list(msg.name)
        if not names or len(names) != len(msg.position):
            return
        with self._lock:
            self._cmd_seq += 1
            self._cmd_latest = {
                'seq': self._cmd_seq,
                'names': names,
                'positions': [float(p) for p in msg.position],
                't': time.time(),
            }

    def get_latest(self):
        with self._lock:
            return dict(self._latest) if self._latest else None

    def state_names(self):
        '''状态消息的关节名列表 (顺序即下标); 尚未收到状态消息时返回 None。

        通用适配: 关节名称与数量完全由 /cb_<hand>_hand_state 决定, 不内置默认值。'''
        with self._lock:
            return list(self._latest['names']) if self._latest else None

    def joint_count(self):
        '''状态消息的关节数 (= position 长度); 未收到状态消息时为 0。'''
        names = self.state_names()
        return len(names) if names else 0

    def cmd_latest(self):
        '''控制命令话题 /cb_<hand>_hand_control_cmd 最新一帧 {seq,names,positions,t}; 尚未收到返回 None。'''
        with self._lock:
            return dict(self._cmd_latest) if self._cmd_latest else None

    def _info_cb(self, msg):
        '''/cb_<hand>_hand_info 是 std_msgs/String(JSON), 取 hand_type / model。'''
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict) or not data.get('hand_type'):
            return
        with self._lock:
            self._hand_info = {k: data[k] for k in ('hand_type', 'model',
                                                    'side', 'uid')
                               if data.get(k) is not None}
            # 温度向量顺序与状态话题一致(即滑块顺序), 每帧缓存
            self._temperature = list(data.get('temperature') or [])
            self._over_temp = list(data.get('over_temp') or [])

    def hand_info(self):
        with self._lock:
            return dict(self._hand_info) if self._hand_info else None

    def temperature_info(self):
        '''返回最新缓存的温度(°C 数组, 顺序同滑块)与 over_temp 告警。'''
        with self._lock:
            return {'temperature': list(self._temperature),
                    'over_temp': list(self._over_temp)}

    def _touch_cb(self, msg):
        '''触觉矩阵回调: 解析 JSON 只缓存最新一帧, 不做任何耗时操作。

        消息格式 (std_msgs/String): {"stamp", "hand_type",
        "sensors": {"1"~"5": {name, rows, cols, matrix}, "6"(手掌, 可选): ...}}'''
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict) or not isinstance(data.get('sensors'), dict):
            return
        with self._lock:
            self._touch_seq += 1
            self._touch = {'seq': self._touch_seq, 't': time.time(),
                           'data': data}

    def touch_latest(self):
        '''最新一帧触觉矩阵 {"seq","t","data"}; 尚未收到时返回 None。'''
        with self._lock:
            return dict(self._touch) if self._touch else None

    def _mass_cb(self, msg):
        '''合力话题回调: 解析 JSON 只缓存最新一帧, 不做任何耗时操作。

        消息格式 (std_msgs/String): {"stamp","hand_type","sensors":
        {"1"~"5": {name, unit, force: {FnS}, cell_sum, cell_max}, ...}};
        force 字段取决于协议数据分布描述, 未声明的字段不会存在。'''
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict) or not isinstance(data.get('sensors'), dict):
            return
        with self._lock:
            self._mass_seq += 1
            self._mass = {'seq': self._mass_seq, 't': time.time(),
                          'data': data}

    def mass_latest(self):
        '''最新一帧合力统计 {"seq","t","data"}; 尚未收到时返回 None。'''
        with self._lock:
            return dict(self._mass) if self._mass else None

    def _parse_urdf_limits(self, urdf_path):
        '''解析 URDF 的 <joint><limit> 弧度限位 (按 URDF 路径缓存, 保持文档顺序)。

        返回 {joint_name: (min_rad, max_rad)}, 仅含 revolute/prismatic 关节。
        读取失败 / XML 解析失败时抛异常 —— 上层返回错误, 不退回任何内置值。'''
        key = str(urdf_path)
        cached = self._joint_limits_cache.get(key)
        if cached is not None:
            return cached
        text = urdf_path.read_text(encoding='utf-8')
        try:
            root = ET.fromstring(text)
        except ET.ParseError as e:
            raise RuntimeError(f'URDF XML 解析失败 ({urdf_path}): {e}')
        limits = {}
        for j in root.iter('joint'):
            name = j.get('name')
            if not name:
                continue
            if j.get('type', 'revolute') not in ('revolute', 'prismatic'):
                continue
            lim = j.find('limit')
            if lim is None:
                continue
            try:
                lo = float(lim.get('lower', 0.0))
                hi = float(lim.get('upper', 0.0))
            except (TypeError, ValueError):
                continue
            if lo > hi:
                lo, hi = hi, lo
            limits[name] = (lo, hi)
        self._joint_limits_cache[key] = limits
        return limits

    def state_to_urdf(self, names, urdf_path):
        '''状态话题关节名 → URDF 关节名 映射 (通用, 不内置任何型号名称)。

        推导自 URDF 关节名 (见 derive_state_to_urdf); URDF 同目录的
        <urdf_stem>.map.json 或 map.json ({"状态名": "URDF关节名"}) 可显式覆盖。
        结果按 (urdf 路径, 关节名) 缓存; 无法映射的名称不在返回 dict 中。'''
        key = (str(urdf_path), tuple(names))
        cached = self._mapping_cache.get(key)
        if cached is not None:
            return cached
        limits = self._parse_urdf_limits(urdf_path)
        mapping = dict(derive_state_to_urdf(names, list(limits.keys())))
        for src in (urdf_path.with_name(urdf_path.stem + '.map.json'),
                    urdf_path.with_name('map.json')):
            if not src.is_file():
                continue
            try:
                data = json.loads(src.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if isinstance(data, dict):
                for s, u in data.items():
                    if isinstance(s, str) and isinstance(u, str):
                        mapping[s] = u
        self._mapping_cache[key] = mapping
        return mapping

    def joints_meta(self):
        '''状态话题关节名 → URDF 关节名 + 弧度限位 (供 3D 显示换算用)。

        - 关节名: 取自 /cb_<hand>_hand_state 的 name (通用; 未收到时 ok=False 且 retryable);
        - 映射:   从 URDF 关节名自动推导 (见 state_to_urdf);
        - 限位:   当前生效 URDF 的 <joint><limit> (加载时缓存);
        - 线性映射: joint 最小值 → 滑动条 0, joint 最大值 → 滑动条 255。
        部分关节无法映射时仍 ok=True 并带 partial/missing (3D 视图显示其余关节)。'''
        names = self.state_names()
        if not names:
            return {'ok': False, 'retryable': True, 'joints': [],
                    'error': '等待状态消息 (/cb_*_hand_state) ...'}
        try:
            hand, model, urdf_path = self.resolve_urdf()
            limits = self._parse_urdf_limits(urdf_path)
        except (FileNotFoundError, OSError, RuntimeError) as e:
            return {'ok': False, 'retryable': False, 'joints': [],
                    'error': f'URDF 不可用: {e}'}
        mapping = self.state_to_urdf(names, urdf_path)
        joints = []
        missing = []
        for n in names:
            u = mapping.get(n)
            if u is None or u not in limits:
                missing.append(n)
                continue
            lo, hi = limits[u]
            joints.append({'state': n, 'urdf': u, 'min_rad': lo, 'max_rad': hi})
        if not joints:
            return {'ok': False, 'retryable': False, 'joints': [],
                    'error': (f'未能推导任何 状态→URDF 关节映射 '
                              f'(缺失: {", ".join(missing)})')}
        return {
            'ok': True,
            'partial': bool(missing),
            'missing': missing,
            'hand': hand,
            'model': model,
            'joints': joints,
        }

    @staticmethod
    def _match_model(model_raw, model_dirs):
        '''将 model 字符串 (设备上报 / 命令行) 匹配到 urdf_base 下的型号子目录。

        未指定 (空 / auto): 仅一个候选目录 → 用它; 多个 → None (歧义, 需 --model 指定)。
        已指定: 先精确 (忽略大小写), 再双向子串匹配 (设备上报的型号名可能带附加字符)。'''
        if not model_dirs:
            return None
        m = (model_raw or '').strip().lower()
        if not m or m == 'auto':
            return model_dirs[0] if len(model_dirs) == 1 else None
        for d in model_dirs:
            if d.lower() == m:
                return d
        hits = [d for d in model_dirs if m in d.lower() or d.lower() in m]
        return hits[0] if len(hits) == 1 else None

    def resolve_urdf(self):
        '''选择 URDF 文件, 返回 (hand, model, path) (hand/model 为子目录名)。

        通用选择规则:
          hand : 优先 info 话题的 hand_type, 回退启动参数 --hand;
          model: info 话题的 model 或 --model (见 _match_model),
                 auto 且型号子目录唯一时自动选用。
        无 URDF 时抛 FileNotFoundError。'''
        if not self.urdf_base.is_dir():
            raise FileNotFoundError(f'URDF 目录不存在: {self.urdf_base}')
        model_dirs = sorted(d.name for d in self.urdf_base.iterdir()
                            if d.is_dir())
        if not model_dirs:
            raise FileNotFoundError(f'URDF 目录为空 (无型号子目录): {self.urdf_base}')
        info = self.hand_info() or {}
        model_raw = str(info.get('model') or '').strip() or self.model_arg
        for hand in (str(info.get('hand_type') or '').lower(), self.hand_type):
            if not hand:
                continue
            model = self._match_model(model_raw, model_dirs)
            if model is None:
                continue
            base = self.urdf_base / model / hand
            if base.is_dir():
                files = sorted(base.glob('*.urdf'))
                if files:
                    return hand, model, files[0]
        raise FileNotFoundError(
            f'未找到 URDF (urdf_base={self.urdf_base}, hand={self.hand_type}, '
            f'model={model_raw or "auto"})')

    def serve_urdf(self):
        '''读取 URDF 并把 mesh 引用改写为 /api/mesh/... 绝对路径。'''
        hand, model, urdf_path = self.resolve_urdf()
        text = urdf_path.read_text(encoding='utf-8')
        prefix = f'/api/mesh/{model}/{hand}/'
        text = re.sub(r'filename="([^"]+)"',
                      lambda m: f'filename="{prefix}{m.group(1).lstrip("/")}"',
                      text)
        return text, hand, model

    def serve_mesh(self, hand, model, rel):
        '''按相对路径返回 mesh 文件字节 (防目录穿越), 不存在返回 None。'''
        base = (self.urdf_base / model / hand).resolve()
        if not base.is_dir():
            return None
        target = (base / rel).resolve()
        try:
            target.relative_to(base)
        except ValueError:
            return None
        if not target.is_file() or target.suffix.lower() not in ('.stl', '.dae', '.obj'):
            return None
        return target.read_bytes()

    def publish_command(self, positions):
        '''校验并发布一条位置命令 (0~255)。返回 (ok, message)。

        通用适配: 关节数必须等于状态消息 position 的长度 (不内置 MIN/MAX_JOINTS),
        尚未收到状态消息时拒绝下发 (无法确认关节数, 避免发给硬件错误长度)。'''
        try:
            values = [float(p) for p in positions]
        except (TypeError, ValueError):
            return False, 'positions 必须是数字列表'
        expected = self.joint_count()
        if expected == 0:
            return False, (f'尚未收到状态消息 {self.state_topic}, 无法确认关节数')
        if len(values) != expected:
            return False, (f'关节数 {len(values)} 与状态消息关节数 {expected} '
                           f'(position 长度) 不符')
        if any(v != v or v in (float('inf'), float('-inf')) for v in values):
            return False, 'positions 含 NaN/Inf'
        clamped = [int(round(min(U8_MAX, max(U8_MIN, v)))) for v in values]

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(p) for p in clamped]
        # velocity / effort 不填: SDK 控制话题只读 position, 空列表表示「不控制」
        self.cmd_pub.publish(msg)
        with self._lock:
            self.cmd_count += 1
        return True, f'已发布 {len(clamped)} 个关节'

    def start_spin(self):
        threading.Thread(target=self._spin_loop, daemon=True).start()

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def shutdown(self):
        try:
            self.destroy_node()
        except Exception:
            pass


def make_handler(bridge, html_path, stream_hz):
    '''生成 HTTP 请求处理器类 (闭包绑定 bridge / 页面 / 推送频率)。'''

    class Handler(BaseHTTPRequestHandler):
        server_version = 'HandWebControl/1.0'

        def log_message(self, fmt, *args):
            '''SSE 长连接会持续产生访问日志, 默认打到 stderr 很吵, 这里静默。'''
            pass

        def _send_json(self, obj, code=200):
            data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        # ---------- GET ----------
        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._serve_html()
            elif self.path == '/api/status':
                self._api_status()
            elif self.path == '/api/info':
                self._api_info()
            elif self.path == '/api/cmd_state':
                self._api_cmd_state()
            elif self.path == '/api/touch':
                self._api_touch()
            elif self.path == '/api/stream':
                self._sse_stream(stream_hz)
            elif self.path == '/api/urdf':
                self._api_urdf()
            elif self.path == '/api/joints':
                self._api_joints()
            elif self.path.startswith('/api/mesh/'):
                self._api_mesh()
            elif re.match(r'^/(lib|js)/[\w./-]+$', self.path):
                self._serve_static()
            else:
                self._send_json({'ok': False, 'error': 'not found'}, 404)

        def do_POST(self):
            if self.path == '/api/cmd':
                self._api_cmd()
            else:
                self._send_json({'ok': False, 'error': 'not found'}, 404)

        def _serve_html(self):
            try:
                data = html_path.read_bytes()
            except OSError as e:
                self._send_json({'ok': False, 'error': f'页面加载失败: {e}'}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _api_status(self):
            latest = bridge.get_latest()
            self._send_json({
                'ok': True,
                'hand': bridge.hand_type,
                'hand_info': bridge.hand_info(),
                'state_topic': bridge.state_topic,
                'cmd_topic': bridge.cmd_topic,
                'connected': latest is not None,
                'joints': len(latest['names']) if latest else 0,
                'names': latest['names'] if latest else [],
                'positions': latest['positions'] if latest else [],
                'cmd_count': bridge.cmd_count,
            })

        def _api_info(self):
            '''温度诊断端点(前端 1Hz 轮询): 返回各关节温度 °C 数组(顺序同滑块)。'''
            info = bridge.temperature_info()
            self._send_json({
                'ok': True,
                'hand': bridge.hand_type,
                'temperature': info['temperature'],
                'over_temp': info['over_temp'],
            })

        def _api_cmd_state(self):
            '''控制命令回显端点 (前端"关闭控制" ~30Hz 轮询):
            返回 /cb_<hand>_hand_control_cmd 最新一帧 {names, positions};
            尚未收到命令时 connected=False, 前端保持上次显示。'''
            latest = bridge.cmd_latest()
            self._send_json({
                'ok': True,
                'hand': bridge.hand_type,
                'cmd_topic': bridge.cmd_topic,
                'connected': latest is not None,
                'names': latest['names'] if latest else [],
                'positions': latest['positions'] if latest else [],
                'cmd_count': bridge.cmd_count,
            })

        def _api_touch(self):
            '''触觉数据端点 (前端 ~30Hz 轮询): 最新帧矩阵 (data) + 合力 (mass)。

            data.sensors: {"1"~"5": {name, rows, cols, matrix}, ...};
            mass.sensors: {"1"~"5": {name, unit, force: {FnS}, cell_sum,
            cell_max}, ...}; 对应话题未收到数据时相应字段为 null。'''
            latest = bridge.touch_latest()
            mass = bridge.mass_latest()
            self._send_json({
                'ok': True,
                'connected': latest is not None,
                'topic': bridge.touch_topic,
                'data': latest['data'] if latest else None,
                't': latest['t'] if latest else None,
                'mass': mass['data'] if mass else None,
                'mass_topic': bridge.mass_topic,
            })

        # ---------- POST ----------
        def _api_cmd(self):
            try:
                length = int(self.headers.get('Content-Length', 0))
            except ValueError:
                length = 0
            raw = self.rfile.read(length) if length > 0 else b''
            try:
                payload = json.loads(raw.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                self._send_json({'ok': False, 'error': f'JSON 解析失败: {e}'}, 400)
                return
            positions = payload.get('positions') if isinstance(payload, dict) else payload
            if not isinstance(positions, (list, tuple)):
                self._send_json({'ok': False, 'error': '缺少 positions 数组'}, 400)
                return
            ok, info = bridge.publish_command(list(positions))
            self._send_json({'ok': ok, 'message': info}, 200 if ok else 400)

        # ---------- URDF / 3D ----------
        def _api_urdf(self):
            try:
                text, hand, model = bridge.serve_urdf()
            except FileNotFoundError as e:
                self._send_json({'ok': False, 'error': str(e)}, 404)
                return
            except OSError as e:
                self._send_json({'ok': False, 'error': f'URDF 读取失败: {e}'},
                                500)
                return
            data = text.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/xml; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('X-URDF-Hand', hand)
            self.send_header('X-URDF-Model', model)
            self.end_headers()
            self.wfile.write(data)

        def _api_joints(self):
            self._send_json(bridge.joints_meta())

        def _api_mesh(self):
            m = re.match(r'^/api/mesh/([^/]+)/([^/]+)/(.+)$', self.path)
            if not m:
                self._send_json({'ok': False, 'error': 'mesh 路径格式错误'}, 400)
                return
            model, hand, rel = m.group(1), m.group(2), m.group(3)
            data = bridge.serve_mesh(hand, model, rel)
            if data is None:
                self._send_json({'ok': False, 'error': 'mesh 不存在'}, 404)
                return
            self.send_response(200)
            self.send_header('Content-Type',
                             'model/stl' if rel.lower().endswith('.stl')
                             else 'application/octet-stream')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        MIME_TYPES = {
            '.js': 'application/javascript; charset=utf-8',
            '.mjs': 'application/javascript; charset=utf-8',
            '.html': 'text/html; charset=utf-8',
            '.json': 'application/json; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.stl': 'model/stl',
        }

        def _serve_static(self):
            '''只允许 static/ 目录下的 /lib/... 与 /js/... 文件 (防目录穿越)。'''
            static_dir = html_path.parent.resolve()
            target = (static_dir / self.path.lstrip('/')).resolve()
            try:
                target.relative_to(static_dir)
            except ValueError:
                self._send_json({'ok': False, 'error': 'forbidden'}, 403)
                return
            if not target.is_file():
                self._send_json({'ok': False, 'error': 'not found'}, 404)
                return
            data = target.read_bytes()
            ctype = self.MIME_TYPES.get(target.suffix.lower(),
                                        'application/octet-stream')
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)

        # ---------- SSE ----------
        def _sse_stream(self, hz):
            '''每客户端一条线程: 有更新就推一帧, 无更新则按 15s 发心跳保活。'''
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()

            interval = 1.0 / max(1.0, float(hz))
            last_seq = -1
            last_sent = time.monotonic()
            while True:
                latest = bridge.get_latest()
                now = time.monotonic()
                if (latest is not None and latest['seq'] != last_seq
                        and now - last_sent >= interval):
                    payload = json.dumps({
                        'names': latest['names'],
                        'positions': latest['positions'],
                        't': latest['t'],
                    }, ensure_ascii=False)
                    try:
                        self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                        self.wfile.flush()
                        last_seq = latest['seq']
                        last_sent = now
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
                else:
                    if now - last_sent > 15.0:
                        try:
                            self.wfile.write(b': ping\n\n')
                            self.wfile.flush()
                            last_sent = now
                        except (BrokenPipeError, ConnectionResetError, OSError):
                            return
                    time.sleep(0.02)

    return Handler


def _local_lan_ip():
    '''取首个非环回 IPv4 地址 (用于提示局域网访问地址)。'''
    try:
        import subprocess
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True,
                             text=True, timeout=3).stdout
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[1] != 'lo':
                ip = parts[3].split('/')[0]
                if not ip.startswith('127.'):
                    return ip
    except (OSError, subprocess.SubprocessError):
        pass
    return None


BASE_PORT = 8090      # 端口自动探测起始端口
PORT_SPAN = 100       # 探测范围: 8090 ~ 8189


def find_free_port(host):
    '''从 BASE_PORT 起向后探测, 返回第一个可绑定的端口; 范围内全占用则报错。'''
    last_err = None
    for port in range(BASE_PORT, BASE_PORT + PORT_SPAN):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port))
                return port
            except OSError as e:
                last_err = e
    raise RuntimeError(f'端口 {BASE_PORT}~{BASE_PORT + PORT_SPAN - 1} 均被占用: {last_err}')


def main():
    parser = argparse.ArgumentParser(description='灵巧手 Web 控制台 (通用)')
    parser.add_argument('--hand', default='right', choices=['left', 'right'],
                        help='手部类型 (默认 right)')
    parser.add_argument('--model', default='auto',
                        help='URDF 型号子目录 (默认 auto: 从状态/信息匹配, 唯一时自动选用)')
    parser.add_argument('--urdf-path', default='',
                        help='URDF 根目录 (留空 = 包内置 urdf 目录; 由 web_control.launch.py 的 urdf_path 配置, 便于以后随时更换)')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址 (默认 0.0.0.0)')
    parser.add_argument('--stream-hz', type=float, default=15.0,
                        help='SSE 状态推送频率 (默认 15)')
    parser.add_argument('--html', default=None, help='自定义 index.html 路径')
    # ros2 launch (Node name=) 会注入 --ros-args -r __node:=web_control,
    # 由 rclpy 创建节点时消费并重命名节点; 普通 argparse 不识别, 用 parse_known_args 容忍
    args, _unknown = parser.parse_known_args()

    html_path = Path(args.html) if args.html else \
        Path(__file__).parent / 'static' / 'index.html'
    if not html_path.is_file():
        print(f'❌ 未找到页面文件: {html_path}')
        return 1

    if not rclpy.ok():
        rclpy.init()

    bridge = HandBridge(args.hand, args.model, args.urdf_path or None)
    bridge.start_spin()

    handler = make_handler(bridge, html_path, args.stream_hz)
    try:
        port = find_free_port(args.host)
        httpd = ThreadingHTTPServer((args.host, port), handler)
    except (OSError, RuntimeError) as e:
        print(f'❌ 没有可用端口: {e}')
        bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    httpd.daemon_threads = True

    print(f'✅ 灵巧手 Web 控制台已就绪  (hand={args.hand}, model={args.model})')
    print(f'   URDF 根目录: {bridge.urdf_base}')
    print(f'   本机访问:    http://127.0.0.1:{port}')
    lan_ip = _local_lan_ip()
    if lan_ip:
        print(f'   局域网访问:  http://{lan_ip}:{port}')
    print(f'   状态话题:  {bridge.state_topic}')
    print(f'   控制话题:  {bridge.cmd_topic}')
    print('   按 Ctrl+C 退出', flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n正在关闭...')
    finally:
        httpd.server_close()
        bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
