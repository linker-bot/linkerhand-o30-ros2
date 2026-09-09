#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install

O30 灵巧手 ROS2 驱动节点。

安全链路（按 o30-sdk-improve.md 的 P0 列表整理）：
  ① 上电校验先行：产品型号 / 左右手 / 协议名称 核对通过、关节无致命故障
     (执行器离线/异常)、使能写入成功之后，才允许发出任何运动指令
     (速度→位置→力矩)。任一致命项不过则只置 init_error，不建话题、不起
     工作线程，由 main() 统一收尾退出——不再「关闭后继续往下跑」。
     不需要上电自动摆位的场合把 auto_init_pose 置 False，节点只读不动。
  ② 设置命令按手隔离：/cb_hand_setting_cmd 是全局话题，双手节点都会收到，
     故回调必须比对 params.hand_type 与本节点 self.hand_type，不一致直接丢弃；
     另外额外提供 /cb_<hand>_hand_setting_cmd 单手话题。
  ③ 输入全部校验：长度(允许单元素广播)、NaN/Inf、限位截断后才下发；
     底层 _pack_u8 用 & 0xFF，256 会变 0、负数会回绕，所以越界必须在此拦住。
     控制话题只认 position：velocity/effort 只接收不下发（默认填 0 即「不控制」），
     速度与力矩是设定值，由 /cb_hand_setting_cmd 的 set_speed /
     set_max_torque_limits 各写一次即可，同值不重复占用总线。
  ④ deadman：cmd_timeout > 0 时，命令流中断超过该时间按 deadman_action
     进入 hold(保持) / open(张开五指) / disable(失能) 之一，并在 info 话题标记。
  ⑤ 关闭顺序：先置停止事件 → join 工作线程 → 关 CAN → destroy_node，
     rclpy.shutdown() 只在 main() 里做一次。工作线程循环条件同时看停止事件，
     不再只依赖 rclpy.ok()（那会导致 join 前线程无法退出）。
  ⑥ 状态/诊断真正发布：/cb_<hand>_hand_state(位置/速度/电流)、
     /cb_<hand>_hand_info(温度/电流/关节故障/通信错误/在线/deadman)、
     /cb_<hand>_hand_matrix_touch(点阵) 与 _matrix_touch_mass(合力/统计)。
'''
import math
import time
import json
import threading
import traceback

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from rclpy.clock import Clock
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from tabulate import tabulate
from wcwidth import wcswidth
from .core.canfd.linker_hand_o30_control import (
    LinkerHandO30Controller, NAMES_EN, NAMES_CN)
from .utils.color_msg import ColorMsg

# 上电初始姿态（20 个有效关节，顺序见 README 第 4 节），可用 init_pose 参数覆盖
INIT_POSE = [33, 23, 96, 176, 212, 162, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
# 速度设置即便为0也不会停止，0映射电机速度8000,255映射电机速度12000
INIT_VEL = [200] * 20
INIT_TORQUE = [200] * 20

U8_MIN, U8_MAX = 0, 255
# 致命关节故障：执行器离线/异常属于不可恢复，带着它运动没有意义且危险
FATAL_JOINT_FAULTS = ("执行器离线", "执行器异常")
DEADMAN_ACTIONS = ("hold", "open", "disable")


class LinkerHand(Node):
    def __init__(self, name):
        super().__init__(name)
        # 初始化失败原因；非 None 时 main() 直接收尾退出，不 spin
        self.init_error = None
        self.linker_hand = None
        self.run_thread = None
        self.touch_thread = None
        self._closed = False
        self._stop_event = threading.Event()
        self._warn_stamps = {}
        # 实测频率计：{name: [窗口内计数, 窗口起点, 最近一次算出的 Hz]}
        self._rate_meter = {}

        # ros时间获取
        self.stamp_clock = Clock()
        self.declare_params()
        self.read_params()
        try:
            self.init_hand()          # 初始化并校验 Linker Hand SDK（含上电摆位）
        except KeyboardInterrupt:
            self.init_error = "用户中断初始化"
            return
        except Exception as e:
            traceback.print_exc()
            self.init_error = f"初始化异常: {e}"
            return
        if self.init_error:
            return

        self.init_topics()            # 初始化ROS2话题
        self.run_thread = threading.Thread(target=self.run, daemon=True)
        self.run_thread.start()
        # 触觉单独一条线程：总线本身是串行的(控制器 _txn_lock)，分线程不会增加总线
        # 吞吐，但能让「读 10~15 帧触觉」不再顶在状态发布/命令下发前面——两者的
        # 事务在总线上交替进行，控制延迟不再被触觉读取整段拖住。
        if self.is_touch and self.touch_fingers:
            self.touch_thread = threading.Thread(target=self.run_touch, daemon=True)
            self.touch_thread.start()

    # ------------------------------------------------------------------ #
    # 参数
    # ------------------------------------------------------------------ #
    def declare_params(self):
        '''声明参数（带默认值）'''
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'O30')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('canfd_device', 0)
        # 通信后端：'libcanbus'(默认, 厂商私有库) 或 'socketcan'(内核原生 can0 + python-can,
        # 用于透明塑封 USB-CANFD 设备)。后四个参数仅 socketcan 生效。
        self.declare_parameter('comm_type', 'libcanbus')
        self.declare_parameter('channel', 'can0')        # socketcan 接口名
        self.declare_parameter('bitrate', 1000000)       # 仲裁段波特率
        self.declare_parameter('dbitrate', 5000000)      # 数据段波特率
        self.declare_parameter('auto_setup', True)       # 自动 ip link 拉起接口

        # ---- 上电安全 ---- #
        # True: 校验全部通过后自动摆到 init_pose；False: 只连接/校验/使能，不动
        self.declare_parameter('auto_init_pose', True)
        self.declare_parameter('init_pose', INIT_POSE)   # 上电目标姿态(20 个)
        self.declare_parameter('init_velocity', INIT_VEL[0])   # 上电速度(0~255)
        self.declare_parameter('init_torque', INIT_TORQUE[0])  # 上电力矩(0~255)
        self.declare_parameter('self_test_delay', 5.0)   # 上电自检等待秒数
        # True: 产品型号/左右手/协议名称 与配置不符时拒绝启动（避免控错设备）
        self.declare_parameter('strict_device_check', True)
        self.declare_parameter('supported_protocol_versions', ['0.0.2', '0.0.3', '0.0.4'])
        # True: 上电检出执行器离线/异常仍继续（仅告警），默认拒绝启动
        self.declare_parameter('ignore_joint_faults', False)

        # ---- 控制输入限位 ---- #
        self.declare_parameter('joint_limit_min', [U8_MIN] * len(INIT_POSE))
        self.declare_parameter('joint_limit_max', [U8_MAX] * len(INIT_POSE))

        # ---- deadman（命令流看门狗） ---- #
        # 秒；<=0 关闭。收到过第一条命令后才开始计时，超时按 deadman_action 处置
        self.declare_parameter('cmd_timeout', 0.0)
        self.declare_parameter('deadman_action', 'hold')  # hold | open | disable

        # ---- 循环频率与诊断 ---- #
        self.declare_parameter('state_rate', 30.0)        # 状态发布/命令下发频率 Hz
        # 触觉发布频率 Hz。触觉在独立线程里跑，不会挤占状态循环；但总线是串行的，
        # 每个手指每周期要花 1 次选择写 + ceil(数据长度/61) 次带应答读，
        # 五指约 15 个事务/周期 —— 30Hz 能否达成看实测 touch_hz（见 info 话题）。
        self.declare_parameter('touch_rate', 30.0)
        # 要发布的传感器编号(1拇指~5小指, 6手掌)。只读需要的手指是提高触觉频率
        # 最有效的手段：读 2 个手指的事务量只有五指的 40%。
        self.declare_parameter('touch_fingers', [1, 2, 3, 4, 5])
        # False 时只发合力/统计话题(_matrix_touch_mass)，省掉点阵 JSON 序列化
        self.declare_parameter('touch_publish_matrix', True)
        self.declare_parameter('info_rate', 1.0)          # 诊断信息发布频率 Hz
        self.declare_parameter('publish_velocity', True)  # 状态里带实时速度(多一次总线读)
        self.declare_parameter('publish_effort', True)    # 状态里带实时电流(多一次总线读)
        self.declare_parameter('temp_warn', 90)           # 过温告警阈值 °C(设备自报 90)

    def read_params(self):
        '''获取参数值'''
        self.hand_type = str(self.get_parameter('hand_type').value).lower()
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = bool(self.get_parameter('is_touch').value)
        self.canfd_device = self.get_parameter('canfd_device').value
        self.comm_type = self.get_parameter('comm_type').value
        self.channel = self.get_parameter('channel').value
        self.bitrate = self.get_parameter('bitrate').value
        self.dbitrate = self.get_parameter('dbitrate').value
        self.auto_setup = self.get_parameter('auto_setup').value

        self.auto_init_pose = bool(self.get_parameter('auto_init_pose').value)
        self.init_pose_param = list(self.get_parameter('init_pose').value)
        self.init_velocity = int(self.get_parameter('init_velocity').value)
        self.init_torque = int(self.get_parameter('init_torque').value)
        self.self_test_delay = float(self.get_parameter('self_test_delay').value)
        self.strict_device_check = bool(self.get_parameter('strict_device_check').value)
        self.supported_protocol_versions = [
            str(v) for v in self.get_parameter('supported_protocol_versions').value]
        self.ignore_joint_faults = bool(self.get_parameter('ignore_joint_faults').value)

        self.joint_limit_min = list(self.get_parameter('joint_limit_min').value)
        self.joint_limit_max = list(self.get_parameter('joint_limit_max').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.deadman_action = str(self.get_parameter('deadman_action').value).lower()
        if self.deadman_action not in DEADMAN_ACTIONS:
            ColorMsg(msg=f"⚠️ deadman_action={self.deadman_action} 无效，"
                         f"可选 {DEADMAN_ACTIONS}，按 hold 处理", color="yellow")
            self.deadman_action = "hold"

        self.state_rate = max(1.0, float(self.get_parameter('state_rate').value))
        self.touch_rate = max(0.1, float(self.get_parameter('touch_rate').value))
        self.touch_fingers = [int(f) for f in self.get_parameter('touch_fingers').value]
        self.touch_publish_matrix = bool(
            self.get_parameter('touch_publish_matrix').value)
        self.info_rate = max(0.1, float(self.get_parameter('info_rate').value))
        self.publish_velocity = bool(self.get_parameter('publish_velocity').value)
        self.publish_effort = bool(self.get_parameter('publish_effort').value)
        self.temp_warn = int(self.get_parameter('temp_warn').value)

        if self.hand_type not in ("left", "right"):
            ColorMsg(msg=f"⚠️ hand_type={self.hand_type} 非 left/right，"
                         f"话题名与左右手核对可能不符预期", color="yellow")

    # ------------------------------------------------------------------ #
    # 初始化：连接 → 校验 → 使能 → （可选）摆位
    # ------------------------------------------------------------------ #
    def init_hand(self):
        '''初始化Linker Hand SDK。任一致命项不通过时置 init_error 并直接返回，
        绝不发送运动指令。'''
        try:
            self.linker_hand = LinkerHandO30Controller(
                hand_type=self.hand_type, canfd_device=self.canfd_device,
                comm_type=self.comm_type, channel=self.channel,
                bitrate=self.bitrate, dbitrate=self.dbitrate,
                auto_setup=self.auto_setup)
        except Exception as e:
            self.init_error = f"创建控制器失败: {e}"
            return
        if not self.linker_hand.is_connected:
            self.init_error = "CANFD 通信未建立（检查设备、权限、波特率与 comm_type）"
            return

        # 有效关节数以控制器实测结果为准（self.linker_hand.num_joints）
        self.num_joints = self.linker_hand.num_joints
        self.joint_names = (NAMES_EN if len(NAMES_EN) == self.num_joints
                            else list(self.linker_hand.joint_names))
        self.joint_names_cn = (NAMES_CN if len(NAMES_CN) == self.num_joints
                               else list(self.joint_names))
        self._resolve_limits()
        self._reset_cmd_state()

        # ---- 设备信息 ---- #
        try:
            hand_info = self.linker_hand.get_device_info()
        except Exception as e:
            self.init_error = f"获取设备信息失败: {e}"
            return
        self.hand_info = hand_info or {}
        print(tabulate([[k, v] for k, v in self.hand_info.items()],
                       headers=[], tablefmt='plain', stralign='left'), flush=True)

        model = (self.hand_info.get("产品型号") or "").strip()
        side = (self.hand_info.get("左右手") or "").strip()
        uid = (self.hand_info.get("设备唯一标识") or "").strip()
        ColorMsg(msg=f"正在校验 {model or '未知型号'}-{side or '未知手别'} 设备，请稍候...",
                 color="yellow")

        if not uid:
            self.init_error = ("设备唯一标识为空，设备未应答 —— 检查硬件连接、"
                               "hand_type 与 comm_type 是否与实际设备一致")
            return
        if not self.check_device_identity(model, side):
            return          # check_device_identity 内部已写 init_error
        if not self.check_joint_health():
            return

        # ---- 使能：位置模式 + 使能全部有效关节，写后回读确认 ---- #
        # if not self.linker_hand.setup():
        #     self.init_error = "关节使能/控制模式写入失败（0x35 / 0x0D 无应答）"
        #     return
        # time.sleep(0.05)
        # masks = self.linker_hand.get_joint_enable()
        # if masks is None:
        #     ColorMsg(msg="⚠️ 无法回读关节使能状态(0x0D)，按已使能继续", color="yellow")
        # else:
        #     not_enabled = [n for n, m in zip(self.joint_names, masks) if not (m & 0x04)]
        #     if not_enabled:
        #         self.init_error = f"以下关节未使能: {not_enabled}"
        #         return
        #     ColorMsg(msg=f"✅ {len(masks)} 个关节已使能", color="green")

        ColorMsg(msg=f"✅ {model}-{side} 设备校验通过", color="green")

        # ---- 校验全部通过后才允许运动 ---- #
        if not self.auto_init_pose:
            ColorMsg(msg="ℹ️ auto_init_pose=False，跳过上电摆位（不发送任何运动指令）",
                     color="yellow")
            return
        self.goto_init_pose()

    def check_device_identity(self, model, side):
        '''核对产品型号 / 左右手 / 协议，防止把命令发到别的设备或不兼容固件上。
        strict_device_check=True 时不符即拒绝启动；False 时仅告警。'''
        problems = []
        want_model = str(self.hand_joint or "").strip()
        if want_model and want_model.upper() not in model.upper():
            problems.append(f"产品型号不符：设备={model or '空'}，配置 hand_joint={want_model}")
        want_side = {"left": "LEFT", "right": "RIGHT"}.get(self.hand_type)
        if want_side and side and want_side not in side.upper():
            problems.append(f"左右手不符：设备={side}，配置 hand_type={self.hand_type}")
        if want_side and not side:
            problems.append("无法读取设备左右手(0x42/0x49 与产品信息均无应答)")

        proto_name = (self.hand_info.get("协议名称") or "").strip()
        if proto_name and "HOP" not in proto_name.upper():
            problems.append(f"协议名称不符：设备={proto_name}，本 SDK 只实现 HOP")

        for p in problems:
            ColorMsg(msg=f"❌ {p}", color="red")
        if problems and self.strict_device_check:
            self.init_error = "；".join(problems) + "（strict_device_check=False 可降级为告警）"
            return False

        # 协议版本只告警：新固件版本号变化不必然不兼容，但要让用户看见
        proto_ver = (self.hand_info.get("协议版本") or "").strip()
        if proto_ver and proto_ver not in self.supported_protocol_versions:
            ColorMsg(msg=f"⚠️ 协议版本 {proto_ver} 不在已验证列表 "
                         f"{self.supported_protocol_versions} 中，字段偏移可能有变动",
                     color="yellow")
        return True

    def check_joint_health(self):
        '''上电关节故障检查：执行器离线/异常视为致命（可用 ignore_joint_faults 放行），
        堵转/过流/过温仅告警。'''
        try:
            faults = self.linker_hand.get_joint_faults()
        except Exception as e:
            ColorMsg(msg=f"⚠️ 关节故障读取异常: {e}，跳过该项检查", color="yellow")
            return True
        if faults is None:
            ColorMsg(msg="⚠️ 关节故障读取失败(0x0C 无应答)，跳过该项检查", color="yellow")
            return True
        if not faults:
            ColorMsg(msg="✅ 全部有效关节无故障", color="green")
            return True

        fatal = {n: f for n, f in faults.items()
                 if any(x in FATAL_JOINT_FAULTS for x in f)}
        for n, f in faults.items():
            ColorMsg(msg=f"{'❌' if n in fatal else '⚠️'} 关节 {n} 故障: {'、'.join(f)}",
                     color="red" if n in fatal else "yellow")
        if fatal and not self.ignore_joint_faults:
            self.init_error = (f"致命关节故障: { {n: '、'.join(f) for n, f in fatal.items()} }"
                               "（确认安全后可置 ignore_joint_faults=True 强行启动）")
            return False
        return True

    def goto_init_pose(self):
        '''上电摆位：等自检 → 设速度 → 摆到 init_pose → 设力矩。仅校验通过后调用。'''
        ok, init_pose = self._sanitize_vector(
            self.init_pose_param, "init_pose",
            self.joint_limit_min, self.joint_limit_max, throttle=False)
        if not ok:
            ColorMsg(msg="⚠️ init_pose 参数不合法，跳过上电摆位", color="yellow")
            return
        ColorMsg(msg=f"等待设备自检 {self.self_test_delay:.1f}s 后摆到初始姿态...",
                 color="yellow")
        if self._stop_event.wait(max(0.0, self.self_test_delay)):
            return
        vel = max(U8_MIN, min(U8_MAX, self.init_velocity))
        torque = max(U8_MIN, min(U8_MAX, self.init_torque))
        self.linker_hand.set_target_velocity([vel] * self.num_joints)
        time.sleep(0.1)  # 等待设备响应
        self.linker_hand.set_target_position(init_pose)
        time.sleep(0.1)  # 等待设备响应
        self.linker_hand.set_target_torque([torque] * self.num_joints)
        # 记下已写入的设定值，上层再发同样的速度/力矩就不必重复占用总线
        self._last_sent_vel = [vel] * self.num_joints
        self._last_sent_torque = [torque] * self.num_joints
        ColorMsg(msg=f"✅ 上电摆位完成（速度 {vel} / 力矩 {torque}）", color="green")

    def _resolve_limits(self):
        '''把 joint_limit_min/max 规整成长度 num_joints 的列表，并夹到 0~255。'''
        def fix(vals, name, default):
            out = [int(v) for v in vals] if vals else []
            if len(out) == 1:
                out = out * self.num_joints
            if len(out) != self.num_joints:
                ColorMsg(msg=f"⚠️ {name} 长度应为 {self.num_joints}(或 1)，"
                             f"收到 {len(out)}，改用默认 {default}", color="yellow")
                out = [default] * self.num_joints
            return [max(U8_MIN, min(U8_MAX, v)) for v in out]

        self.joint_limit_min = fix(self.joint_limit_min, "joint_limit_min", U8_MIN)
        self.joint_limit_max = fix(self.joint_limit_max, "joint_limit_max", U8_MAX)
        for i, (lo, hi) in enumerate(zip(self.joint_limit_min, self.joint_limit_max)):
            if lo > hi:
                ColorMsg(msg=f"⚠️ 关节 {self.joint_names[i]} 限位 min({lo})>max({hi})，"
                             f"已交换", color="yellow")
                self.joint_limit_min[i], self.joint_limit_max[i] = hi, lo

    def _reset_cmd_state(self):
        self._cmd_lock = threading.Lock()
        self._pending_pose = None
        self._pending_vel = None
        self._pending_torque = None
        # 已经写到设备里的速度/力矩：设备侧会锁存，值没变就不必再占一次总线事务
        self._last_sent_vel = None
        self._last_sent_torque = None
        self._last_cmd_time = None      # 最近一条合法命令的 monotonic 时间
        self._deadman_tripped = False
        self._rejected_cmds = 0
        self._read_fail = 0
        self._last_heartbeat = None
        self._online = True

    # ------------------------------------------------------------------ #
    # 话题
    # ------------------------------------------------------------------ #
    def init_topics(self):
        # 设置命令：全局话题(双手节点都会收到，回调内按 hand_type 过滤) + 单手话题
        self.hand_setting_sub = self.create_subscription(
            String, '/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self.hand_setting_own_sub = self.create_subscription(
            String, f'/cb_{self.hand_type}_hand_setting_cmd', self.hand_setting_cb, 10)
        # 控制命令订阅
        self.hand_cmd_sub = self.create_subscription(
            JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb, 10)
        # joint state状态发布
        self.hand_state_pub = self.create_publisher(
            JointState, f'/cb_{self.hand_type}_hand_state', 10)
        # 判断是否存在触觉数据，如果存在则创建触觉数据发布器
        if self.is_touch:
            self.matrix_touch_pub = self.create_publisher(
                String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
            self.matrix_touch_mass_pub = self.create_publisher(
                String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
            self._resolve_touch_fingers()
        else:
            self.touch_fingers = []
        # 其他信息发布
        self.hand_info_pub = self.create_publisher(
            String, f'/cb_{self.hand_type}_hand_info', 10)
        ColorMsg(msg=f"✅ Topics话题初始化完毕", color="green")

    def _resolve_touch_fingers(self):
        '''把 touch_fingers 参数与实机探测到的可用传感器求交集。

        读一个手指 = 1 次选择写 + ceil(数据长度/61) 次带应答读（70 字节的指尖
        传感器要 2 次），所以少读几个手指就能线性提高 touch_rate。这里顺手过滤掉
        本机没装的编号，免得每周期都去等一遍不存在传感器的超时。'''
        avail = self.linker_hand.get_available_sensors() or []
        if not avail:
            self.touch_fingers = []
            ColorMsg(msg="⚠️ is_touch=True 但未探测到可用传感器，触觉话题将无数据",
                     color="yellow")
            return
        wanted = [f for f in dict.fromkeys(self.touch_fingers)]   # 去重保序
        self.touch_fingers = [f for f in wanted if f in avail]
        skipped = [f for f in wanted if f not in avail]
        if skipped:
            ColorMsg(msg=f"⚠️ touch_fingers 中 {skipped} 不可用，已跳过（可用: {avail}）",
                     color="yellow")
        if not self.touch_fingers:
            ColorMsg(msg=f"⚠️ touch_fingers 与可用传感器无交集，触觉不发布（可用: {avail}）",
                     color="yellow")
            return
        ColorMsg(msg=f"✅ 触觉发布 {len(self.touch_fingers)} 指 {self.touch_fingers} "
                     f"@ {self.touch_rate}Hz"
                     f"（实测频率见 /cb_{self.hand_type}_hand_info 的 touch_hz）",
                 color="green")

    # ------------------------------------------------------------------ #
    # 输入校验
    # ------------------------------------------------------------------ #
    def _warn(self, key, msg, period=2.0):
        '''同类告警限流，避免高频命令刷屏（key 相同的消息 period 秒内只打一次）'''
        now = time.monotonic()
        if now - self._warn_stamps.get(key, 0.0) < period:
            return
        self._warn_stamps[key] = now
        ColorMsg(msg=msg, color="yellow")

    def _sanitize_vector(self, values, name, lo, hi, throttle=True):
        '''校验并规范化一个控制向量，返回 (ok, [int])。

        非数值、NaN/Inf、空列表、长度不符(且不是单元素广播) → 整条命令判否，不下发；
        越界值截断到 [lo, hi] 并告警——底层 _pack_u8 是 `& 0xFF`，256 会变成 0、
        负数会回绕成大值，越界不拦住会直接变成错误动作。
        lo/hi 可为标量或与关节数等长的列表。'''
        warn = self._warn if throttle else (
            lambda key, msg, period=0.0: ColorMsg(msg=msg, color="yellow"))
        vals = list(values) if values is not None else []
        if not vals:
            warn(f"{name}_len", f"⚠️ {name} 为空列表，已忽略该命令")
            return False, []
        if len(vals) == 1:
            vals = vals * self.num_joints           # 单元素广播到全部关节
        if len(vals) != self.num_joints:
            # 不按首元素广播：6/10 自由度上层的数据会把 20 个关节全推到同一个位置，
            # 那是能真出事的动作，宁可整条丢掉让上层暴露出来。
            warn(f"{name}_len", f"⚠️ {name} 长度应为 {self.num_joints}(或 1 广播)，"
                                f"收到 {len(vals)}，已忽略该命令")
            return False, []
        los = lo if isinstance(lo, (list, tuple)) else [lo] * self.num_joints
        his = hi if isinstance(hi, (list, tuple)) else [hi] * self.num_joints
        out, clamped = [], []
        for i, v in enumerate(vals):
            try:
                f = float(v)
            except (TypeError, ValueError):
                warn(f"{name}_type", f"⚠️ {name}[{i}]={v!r} 不是数值，已忽略该命令")
                return False, []
            if not math.isfinite(f):
                warn(f"{name}_nan", f"⚠️ {name}[{i}]={v} 为 NaN/Inf，已忽略该命令")
                return False, []
            iv = int(round(f))
            cv = max(los[i], min(his[i], iv))
            if cv != iv:
                clamped.append(f"{self.joint_names[i]}:{iv}->{cv}")
            out.append(cv)
        if clamped:
            warn(f"{name}_clamp", f"⚠️ {name} 越界已截断: {'、'.join(clamped[:5])}"
                                  f"{' ...' if len(clamped) > 5 else ''}")
        return True, out

    def hand_control_cb(self, msg):
        '''控制命令回调：只取 position，校验通过后交给工作线程（回调里不碰总线）。

        velocity / effort 只接收、不下发：速度与力矩是设备侧锁存的**设定值**，由
        /cb_hand_setting_cmd 的 set_speed / set_max_torque_limits 各写一次即可；
        跟着每条位置命令重复下发只会白占总线事务。默认这两个字段填 0 表示「不控制」。'''
        if len(msg.position) == 0:
            self._rejected_cmds += 1
            return
        ok, pose = self._sanitize_vector(
            msg.position, "position", self.joint_limit_min, self.joint_limit_max)
        if not ok:
            self._rejected_cmds += 1
            return
        if any(msg.velocity) or any(msg.effort):
            self._warn("vel_eff_off",
                       "ℹ️ 控制话题的 velocity/effort 不下发到设备，请用 "
                       "/cb_hand_setting_cmd 的 set_speed / set_max_torque_limits 设置",
                       period=30.0)
        with self._cmd_lock:
            self._pending_pose = pose
            self._last_cmd_time = time.monotonic()

    # ------------------------------------------------------------------ #
    # 工作线程
    # ------------------------------------------------------------------ #
    def run(self):
        '''控制线程：下发命令 + 发布关节状态 + 发布诊断（触觉见 run_touch）。'''
        period = 1.0 / self.state_rate
        next_info = 0.0
        while rclpy.ok() and not self._stop_event.is_set():
            start = time.monotonic()
            try:
                self.service_commands()
                self.publish_state()
                if start >= next_info:
                    next_info = start + 1.0 / self.info_rate
                    self.publish_info()
            except Exception as e:
                # 单次异常不能让控制线程永久退出——记录后退避重试
                self._warn("loop_err", f"⚠️ 控制循环异常: {e}")
                self._stop_event.wait(0.1)
            self._stop_event.wait(max(0.0, period - (time.monotonic() - start)))

    def run_touch(self):
        '''触觉线程：按 touch_rate 读取并发布 touch_fingers 指定的传感器。

        与控制线程共用总线（控制器内部 _txn_lock 串行化，安全）；跟不上设定频率时
        由 publish_info 里的 touch_hz 反映实测值，并给出降频建议。'''
        period = 1.0 / self.touch_rate
        while rclpy.ok() and not self._stop_event.is_set():
            start = time.monotonic()
            try:
                self.publish_touch()
            except Exception as e:
                self._warn("touch_err", f"⚠️ 触觉循环异常: {e}")
                self._stop_event.wait(0.1)
            self._stop_event.wait(max(0.0, period - (time.monotonic() - start)))

    def _tick(self, name):
        '''频率计打点：每满 1 秒结算一次实测 Hz。'''
        m = self._rate_meter.setdefault(name, [0, time.monotonic(), 0.0])
        m[0] += 1
        dt = time.monotonic() - m[1]
        if dt >= 1.0:
            m[2] = m[0] / dt
            m[0], m[1] = 0, time.monotonic()

    def _hz(self, name):
        m = self._rate_meter.get(name)
        return round(m[2], 1) if m else 0.0

    def service_commands(self):
        '''下发待处理目标值，并做 deadman 判定。

        位置来自控制话题，每次都下发；速度/力矩只来自 /cb_hand_setting_cmd，是设备侧
        锁存的设定值，写一次就一直有效，所以值没变就跳过（上层重复点「设置速度」不会
        白占总线）。使能状态被动过（deadman disable / enable_joints）之后缓存会清空，
        下一次设置重新写一遍。'''
        with self._cmd_lock:
            pose, self._pending_pose = self._pending_pose, None
            vel, self._pending_vel = self._pending_vel, None
            torque, self._pending_torque = self._pending_torque, None
            last_cmd = self._last_cmd_time

        if self._deadman_tripped and any(x is not None for x in (pose, vel, torque)):
            self.recover_deadman()
        if vel is not None and vel != self._last_sent_vel:
            if self.linker_hand.set_target_velocity(vel) is not False:
                self._last_sent_vel = list(vel)
            time.sleep(0.002)
        if torque is not None and torque != self._last_sent_torque:
            if self.linker_hand.set_target_torque(torque) is not False:
                self._last_sent_torque = list(torque)
            time.sleep(0.002)
        if pose is not None:
            self.linker_hand.set_target_position(pose)
            time.sleep(0.002)

        # deadman：收到过第一条命令后才开始计时（否则空闲待命就会误触发）
        if (self.cmd_timeout > 0 and last_cmd is not None
                and not self._deadman_tripped
                and time.monotonic() - last_cmd > self.cmd_timeout):
            self.trip_deadman()

    def trip_deadman(self):
        self._deadman_tripped = True
        ColorMsg(msg=f"❗ 控制命令中断超过 {self.cmd_timeout:.2f}s，"
                     f"执行 deadman 动作: {self.deadman_action}", color="red")
        if self.deadman_action == "open":
            # 只伸直 root1/tip，不动横滚/航向/指根2，避免未标定方向的意外姿态
            self.linker_hand.open_palm(0)
        elif self.deadman_action == "disable":
            # 清 ENABLED 位（保留 存在+可控+反馈），恢复时重新 enable_all_joints
            self.linker_hand.set_joint_enable([0x0B] * self.num_joints)
            self._invalidate_setpoint_cache()
        # hold: 目标值本就锁存在设备侧，不下发新指令即为保持

    def recover_deadman(self):
        ColorMsg(msg="✅ 控制命令恢复，退出 deadman 状态", color="green")
        if self.deadman_action == "disable":
            self.linker_hand.enable_all_joints()
            self._invalidate_setpoint_cache()
            time.sleep(0.01)
        self._deadman_tripped = False

    def _invalidate_setpoint_cache(self):
        '''使能状态被改动后，设备侧锁存的速度/力矩不再可信，下一条命令必须重写一遍。'''
        self._last_sent_vel = None
        self._last_sent_torque = None

    # ------------------------------------------------------------------ #
    # 发布
    # ------------------------------------------------------------------ #
    def publish_state(self):
        pose = self.linker_hand.get_current_position()
        if pose is None:
            self._read_fail += 1
            self._warn("pos_read", f"⚠️ 读取关节位置失败（累计 {self._read_fail} 次）")
            return
        self._read_fail = 0
        vel = self.linker_hand.get_current_velocity() if self.publish_velocity else None
        effort = self.linker_hand.get_motor_current() if self.publish_effort else None
        self.hand_state_pub.publish(self.joint_state_msg(pose, vel=vel, effort=effort))
        self._tick("state")

    def joint_state_msg(self, pose, vel=None, effort=None):
        joint_state = JointState()
        joint_state.header = Header()
        # 当前时间戳
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = [float(x) for x in pose]
        # 如果速度和力矩数据为空，则填充为0
        joint_state.velocity = ([float(x) for x in vel] if vel
                                else [0.0] * len(pose))
        joint_state.effort = ([float(x) for x in effort] if effort
                              else [0.0] * len(pose))
        return joint_state

    def publish_touch(self):
        '''一次读取同时得到点阵与合力（合力就在数据区开头），分别发到两个话题。

        只读 touch_fingers 指定的传感器——每个手指要花 1 次选择写 + ceil(长度/61)
        次带应答读，手指越少频率越高。'''
        matrix, mass = {}, {}
        for finger in self.touch_fingers:
            d = self.linker_hand.get_tactile_data(finger)
            if d is None:
                continue
            key = str(finger)
            if self.touch_publish_matrix:
                matrix[key] = {"name": d.get("name"), "rows": d.get("rows"),
                               "cols": d.get("cols"), "matrix": d.get("matrix")}
            mass[key] = {"name": d.get("name"), "unit": d.get("unit"),
                         "force": d.get("force"), "cell_sum": d.get("cell_sum"),
                         "cell_max": d.get("cell_max")}
        if not mass:
            self._warn("touch_read", "⚠️ 触觉读取失败（全部传感器无数据）")
            return
        stamp = self.get_clock().now().nanoseconds
        if self.touch_publish_matrix:
            self.matrix_touch_pub.publish(
                String(data=json.dumps({"stamp": stamp, "hand_type": self.hand_type,
                                        "sensors": matrix}, ensure_ascii=False)))
        self.matrix_touch_mass_pub.publish(
            String(data=json.dumps({"stamp": stamp, "hand_type": self.hand_type,
                                    "sensors": mass}, ensure_ascii=False)))
        self._tick("touch")

    def publish_info(self):
        '''诊断信息：温度/电流/关节故障/通信错误码/在线状态/deadman 状态。'''
        temps = self.linker_hand.get_temperature()
        currents = self.linker_hand.get_motor_current()
        faults = self.linker_hand.get_joint_faults()
        # 心跳递增即在线（比 is_online() 省一次事务与 0.1s 阻塞）
        hb = self.linker_hand.get_heartbeat()
        if hb is not None:
            self._online = self._last_heartbeat is None or hb != self._last_heartbeat
            self._last_heartbeat = hb
        else:
            self._online = False
        if not self._online:
            self._warn("offline", "⚠️ 心跳无变化/无应答，设备可能离线")

        over_temp = []
        if temps:
            over_temp = [f"{n}:{t}" for n, t in zip(self.joint_names, temps)
                         if t >= self.temp_warn]
            if over_temp:
                self._warn("over_temp",
                           f"⚠️ 关节温度达到阈值 {self.temp_warn}°C: {'、'.join(over_temp)}")
        err = self.linker_hand.last_error_code
        state_hz, touch_hz = self._hz("state"), self._hz("touch")
        self._check_rate("state", state_hz, self.state_rate,
                         "可关掉 publish_velocity/publish_effort 各省一次总线读")
        if self.touch_fingers:
            self._check_rate("touch", touch_hz, self.touch_rate,
                             "可减少 touch_fingers 的手指数，或关掉 touch_publish_matrix")
        info = {
            "stamp": self.get_clock().now().nanoseconds,
            "hand_type": self.hand_type,
            "model": self.hand_info.get("产品型号"),
            "side": self.hand_info.get("左右手"),
            "uid": self.hand_info.get("设备唯一标识"),
            "protocol": f"{self.hand_info.get('协议名称')} "
                        f"{self.hand_info.get('协议版本')}".strip(),
            "joint_names": list(self.joint_names),
            "online": self._online,
            "temperature": temps or [],
            "over_temp": over_temp,
            "current": currents or [],
            "joint_faults": faults or {},
            "comm_error": {"code": err,
                           "names": self.linker_hand.decode_error_code(err)},
            "deadman": {"enabled": self.cmd_timeout > 0,
                        "timeout": self.cmd_timeout,
                        "action": self.deadman_action,
                        "tripped": self._deadman_tripped},
            "rejected_cmds": self._rejected_cmds,
            "read_fail": self._read_fail,
            # 实测频率（不是设定值）：总线是串行的，两条线程共享带宽，
            # 达不到设定值时按 hint 降配置
            "state_hz": state_hz,
            "state_rate_target": self.state_rate,
            "touch_hz": touch_hz,
            "touch_rate_target": self.touch_rate if self.touch_fingers else 0.0,
            "touch_fingers": list(self.touch_fingers),
        }
        self.hand_info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

    def _check_rate(self, name, actual, target, hint):
        '''实测频率低于目标 80% 时限流告警（频率计要满 1 秒才有值，0 表示还没结算）'''
        if actual <= 0 or actual >= target * 0.8:
            return
        self._warn(f"rate_{name}", f"⚠️ {name} 实测 {actual}Hz 低于设定 {target}Hz：{hint}",
                   period=10.0)

    # ------------------------------------------------------------------ #
    # 设置命令
    # ------------------------------------------------------------------ #
    def hand_setting_cb(self, msg):
        '''设置命令回调。

        ⚠️ /cb_hand_setting_cmd 是全局话题，双手启动时两个节点都会收到同一条消息，
           因此必须比对 params.hand_type 与本节点的 self.hand_type：不一致直接丢弃，
           否则给右手设速度会把左手也一起改掉。缺省/"both"/"all" 视为广播。
        '''
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            ColorMsg(msg=f"⚠️ 设置命令不是合法 JSON: {e}", color="yellow")
            return
        if not isinstance(data, dict):
            ColorMsg(msg="⚠️ 设置命令应为 JSON 对象", color="yellow")
            return
        cmd = data.get("setting_cmd")
        params = data.get("params") if isinstance(data.get("params"), dict) else {}
        target = str(params.get("hand_type", "")).lower()
        if target and target not in (self.hand_type, "both", "all"):
            return          # 目标是另一只手，静默丢弃
        if not target:
            self._warn("setting_no_hand",
                       "ℹ️ 设置命令未指定 params.hand_type，按广播处理", period=10.0)
        print(f"Received setting command: {cmd}", flush=True)

        try:
            handler = {
                "set_speed":             self._cmd_set_speed,
                "set_max_torque_limits": self._cmd_set_torque,
                "get_faults":            self._cmd_get_faults,
                "get_info":              self._cmd_get_info,
                "enable_joints":         self._cmd_enable_joints,
                "disable_joints":        self._cmd_disable_joints,
            }.get(cmd)
            if handler is None:
                ColorMsg(msg=f"⚠️ 不支持的设置命令: {cmd}", color="yellow")
                return
            handler(params)
        except Exception as e:
            ColorMsg(msg=f"⚠️ 设置命令 {cmd} 执行失败: {e}", color="yellow")

    def _cmd_set_speed(self, params):
        speed = params.get("speed")
        if not isinstance(speed, list) or not speed:
            ColorMsg(msg="⚠️ speed 参数错误：应为非空列表（长度 20 或 1 广播）",
                     color="yellow")
            return
        ok, vals = self._sanitize_vector(speed, "speed", U8_MIN, U8_MAX, throttle=False)
        if not ok:
            return
        print(f"设置速度{vals}", flush=True)
        with self._cmd_lock:
            # 只排队，由控制线程统一写总线（回调线程不碰总线，也避免写两遍）
            self._pending_vel = vals
            self._last_cmd_time = time.monotonic()

    def _cmd_set_torque(self, params):
        torque = params.get("torque")
        if not isinstance(torque, list) or not torque:
            ColorMsg(msg="⚠️ torque 参数错误：应为非空列表（长度 20 或 1 广播）",
                     color="yellow")
            return
        ok, vals = self._sanitize_vector(torque, "torque", U8_MIN, U8_MAX, throttle=False)
        if not ok:
            return
        print(f"设置力矩{vals}", flush=True)
        with self._cmd_lock:
            self._pending_torque = vals
            self._last_cmd_time = time.monotonic()

    def _cmd_get_faults(self, params):
        self.linker_hand.print_joint_faults()

    def _cmd_get_info(self, params):
        # 重读一次产品信息并打印（同时刷新 info 话题里的缓存字段）
        self.hand_info = self.linker_hand.get_device_info() or self.hand_info
        print(tabulate([[k, v] for k, v in self.hand_info.items()],
                       headers=[], tablefmt='plain', stralign='left'), flush=True)

    def _cmd_enable_joints(self, params):
        ok = self.linker_hand.enable_all_joints()
        self._invalidate_setpoint_cache()
        ColorMsg(msg=f"{'✅' if ok else '⚠️'} 使能全部关节{'成功' if ok else '失败'}",
                 color="green" if ok else "yellow")

    def _cmd_disable_joints(self, params):
        ok = self.linker_hand.set_joint_enable([0x0B] * self.num_joints)
        self._invalidate_setpoint_cache()
        ColorMsg(msg=f"{'✅' if ok else '⚠️'} 失能全部关节{'成功' if ok else '失败'}",
                 color="green" if ok else "yellow")

    # ------------------------------------------------------------------ #
    # 关闭
    # ------------------------------------------------------------------ #
    def close(self):
        '''关闭顺序：置停止事件 → 等工作线程退出 → 关 CAN → 销毁节点。
        rclpy.shutdown() 交给 main()，避免线程仍以 rclpy.ok() 为条件时被提前拆掉。'''
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        # 两条线程都要等：触觉线程若还在总线上做分段读，先关 CAN 会让它拿到已关闭的
        # 句柄（libcanbus 下是野指针）。
        for th, name in ((self.run_thread, "控制线程"), (self.touch_thread, "触觉线程")):
            if th is not None and th.is_alive():
                th.join(timeout=2.0)
                if th.is_alive():
                    ColorMsg(msg=f"⚠️ {name} 2s 内未退出，继续关闭总线", color="yellow")
        if self.linker_hand is not None:
            try:
                self.linker_hand.close()
            except Exception as e:
                ColorMsg(msg=f"⚠️ 关闭控制器异常: {e}", color="yellow")
            self.linker_hand = None
        try:
            self.destroy_node()
        except Exception:
            pass


# 自定义中文对齐
def chinese_len(text):
    return wcswidth(str(text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LinkerHand("linker_hand_o30_ros2_sdk")
        if node.init_error:
            ColorMsg(msg=f"❌ 设备初始化失败: {node.init_error}", color="red")
        else:
            rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        ColorMsg(msg="收到 Ctrl+C，准备退出...", color="yellow")
    except Exception as e:
        ColorMsg(msg=f"❌ 发生错误: {e}", color="red")
        traceback.print_exc()
    finally:
        if node is not None:
            node.close()             # 停线程 → 关 CAN → 销毁节点
        if rclpy.ok():
            rclpy.shutdown()         # 关闭 ROS
        ColorMsg(msg="程序已退出", color="yellow")
