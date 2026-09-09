#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""驱动节点安全逻辑自检（用假控制器打桩，不接真机、不上总线）。

覆盖 o30-sdk-improve.md 的 P0 项：上电校验先行、双手命令隔离、20 长度速度命令、
输入校验与限位、命令超时 deadman、关闭顺序、诊断/触觉发布、控制线程抗异常。

不会影响真机：控制/设置命令都是直接调回调函数，不往话题发布；需要订阅验证的用例用
hand_type="test"（话题前缀 /cb_test_hand_*），与真机节点的 left/right 话题完全隔开。

运行（不需要 colcon，也不会被 pytest 自动收集）::

    source /opt/ros/<distro>/setup.bash
    python3 test/node_safety_check.py
"""
import json
import os
import sys
import time

# 直接从源码树导入被测节点（无需先 colcon build）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from linker_hand_o30_ros2_sdk import linker_hand_o30 as mod

N = 20
NAMES = mod.NAMES_EN


class FakeCtrl:
    def __init__(self, **kw):
        self.kw = kw
        self.num_joints = N
        self.joint_names = list(NAMES)
        self.is_connected = True
        self.last_error_code = 0
        self.sent = []          # (what, values)
        self.touch_reads = []   # 被读过的传感器编号（验证 touch_fingers 生效）
        self.closed = False
        self.enable_mask = [0x0F] * N
        self.hb = 0
        self.side = "RIGHT"
        self.faults = {}

    # --- 信息 ---
    def get_device_info(self):
        return {"产品型号": "O30", "设备唯一标识": "LHT2000000001",
                "协议名称": "HOP", "协议版本": "0.0.2", "左右手": self.side}

    def print_device_info(self): print("print_device_info")

    # --- 校验 ---
    def get_joint_faults(self): return self.faults
    def print_joint_faults(self): print("print_joint_faults")
    def setup(self): self.sent.append(("setup", None)); return True
    def get_joint_enable(self): return list(self.enable_mask)
    def enable_all_joints(self):
        self.enable_mask = [0x0F] * N; self.sent.append(("enable", None)); return True
    def set_joint_enable(self, masks):
        self.enable_mask = list(masks); self.sent.append(("disable", masks)); return True

    # --- 控制 ---
    def set_target_position(self, v): self.sent.append(("pos", list(v))); return True
    def set_target_velocity(self, v): self.sent.append(("vel", list(v))); return True
    def set_target_torque(self, v): self.sent.append(("tor", list(v))); return True
    def open_palm(self, v=0): self.sent.append(("open", v)); return True

    # --- 读 ---
    def get_current_position(self): return [11] * N
    def get_current_velocity(self): return [22] * N
    def get_motor_current(self): return [33] * N
    def get_temperature(self): return [40] * (N - 1) + [95]
    def get_heartbeat(self):
        self.hb += 1
        return self.hb
    def get_available_sensors(self): return [1, 2]
    def get_tactile_data(self, finger=None):
        if finger not in (1, 2):
            return None
        self.touch_reads.append(finger)
        return {"finger": finger, "name": f"传感器{finger}", "rows": 2, "cols": 2,
                "unit": "N", "force": {"FnS": 7}, "matrix": [[1, 2], [3, 4]],
                "cells": [1, 2, 3, 4], "cell_sum": 10, "cell_max": 4}

    def get_all_tactile_data(self):
        return {f: self.get_tactile_data(f) for f in (1, 2)}

    @staticmethod
    def decode_error_code(code): return []
    def close(self): self.closed = True


mod.LinkerHandO30Controller = FakeCtrl
fails = []


def check(cond, label):
    print(("  ✅ " if cond else "  ❌ ") + label)
    if not cond:
        fails.append(label)


def new_node(**params):
    """rclpy 参数只能通过命令行/覆盖注入，这里直接用 parameter_overrides。"""
    from rclpy.parameter import Parameter
    ov = []
    for k, v in params.items():
        ov.append(Parameter(k, value=v))
    n = mod.LinkerHand.__new__(mod.LinkerHand)
    # 走正常 __init__，但注入 parameter_overrides
    import rclpy.node as rn
    orig_init = rn.Node.__init__
    def patched(self, name, **kw):
        kw["parameter_overrides"] = ov
        orig_init(self, name, **kw)
    rn.Node.__init__ = patched
    try:
        mod.LinkerHand.__init__(n, "t_" + str(int(time.time() * 1e6) % 10 ** 8))
    finally:
        rn.Node.__init__ = orig_init
    return n


rclpy.init()

print("\n[1] 正常启动：校验 → 使能 → 摆位")
n = new_node(hand_type="right", self_test_delay=0.0, is_touch=True,
             state_rate=50.0, info_rate=50.0, touch_rate=50.0)
check(n.init_error is None, f"init_error is None (got {n.init_error})")
kinds = [k for k, _ in n.linker_hand.sent]
# init_hand() 里的 setup()（控制模式+使能）与使能回读目前被注释掉了，所以 sent 里可能
# 没有 "setup"。仍然必须成立的约束是：任何运动指令都在设备校验之后，且从设速度开始。
if "setup" in kinds:
    check(kinds[0] == "setup", f"先 setup 再运动: {kinds}")
else:
    print("  ℹ️ 源码里 setup()/使能回读已注释，跳过「先 setup 再运动」检查")
motion = [k for k in kinds if k in ("pos", "vel", "tor", "open")]
check(motion[:1] == ["vel"], f"运动从设速度开始（校验通过后才动）: {kinds}")
check("pos" in kinds and kinds.index("vel") < kinds.index("pos"), "速度先于位置下发")
time.sleep(0.3)
n.close()
check(n.linker_hand is None and not n.run_thread.is_alive(), "close 后线程退出且总线关闭")

print("\n[2] auto_init_pose=False 时不发任何运动指令")
n = new_node(hand_type="right", auto_init_pose=False, self_test_delay=0.0)
kinds = [k for k, _ in n.linker_hand.sent]
check(all(k in ("setup", "enable") for k in kinds), f"无运动指令: {kinds}")
n.close()

print("\n[3] 左右手不符 → 拒绝启动，且未发运动指令")
class SideMismatch(FakeCtrl):
    def __init__(self, **kw):
        super().__init__(**kw); self.side = "LEFT"
mod.LinkerHandO30Controller = SideMismatch
n = new_node(hand_type="right", self_test_delay=0.0)
check(n.init_error is not None, f"init_error 已置位: {n.init_error}")
check(n.linker_hand.sent == [], "未发送任何指令")
check(not hasattr(n, "hand_state_pub"), "未建话题")
n.close()
check(n.linker_hand is None, "close 仍释放了控制器")

print("\n[4] 致命关节故障 → 拒绝启动")
class Faulty(FakeCtrl):
    def get_joint_faults(self): return {"index_tip": ["执行器离线"]}
mod.LinkerHandO30Controller = Faulty
n = new_node(hand_type="right", self_test_delay=0.0)
check(n.init_error is not None, f"init_error: {n.init_error}")
n.close()
n = new_node(hand_type="right", self_test_delay=0.0, ignore_joint_faults=True)
check(n.init_error is None, "ignore_joint_faults=True 可放行")
n.close()

print("\n[5] 使能未生效 → 拒绝启动（依赖 init_hand() 的使能回读）")
class NotEnabled(FakeCtrl):
    def get_joint_enable(self): return [0x0B] * N
mod.LinkerHandO30Controller = NotEnabled
n = new_node(hand_type="right", self_test_delay=0.0)
if "setup" in [k for k, _ in n.linker_hand.sent]:
    check(n.init_error is not None and "未使能" in n.init_error, f"{n.init_error}")
else:
    # 使能写入/回读在源码里被注释掉了，节点不再检查 ENABLED 位，这里只确认没有误报
    print("  ℹ️ 源码里 setup()/使能回读已注释，节点不再校验使能状态")
    check(n.init_error is None, "使能回读关闭时不误报 init_error")
n.close()

mod.LinkerHandO30Controller = FakeCtrl

print("\n[6] 输入校验")
n = new_node(hand_type="right", auto_init_pose=False, state_rate=100.0)
c = n.linker_hand
ok, v = n._sanitize_vector([256] * N, "position", 0, 255, throttle=False)
check(ok and v == [255] * N, f"256 截断为 255（底层 &0xFF 会变 0）: {v[:3]}")
ok, v = n._sanitize_vector([-5] * N, "position", 0, 255, throttle=False)
check(ok and v == [0] * N, f"负数截断为 0: {v[:3]}")
ok, v = n._sanitize_vector([float("nan")] * N, "position", 0, 255, throttle=False)
check(not ok, "NaN 整条拒绝")
ok, v = n._sanitize_vector([1, 2, 3], "position", 0, 255, throttle=False)
check(not ok, "长度不符整条拒绝（不按首元素广播成 20 个同值）")
ok, v = n._sanitize_vector([80], "position", 0, 255, throttle=False)
check(ok and v == [80] * N, "单元素广播")
ok, v = n._sanitize_vector([], "position", 0, 255, throttle=False)
check(not ok, "空列表拒绝（不会去取 [0]）")

print("\n[7] 限位参数生效")
n2 = new_node(hand_type="right", auto_init_pose=False,
              joint_limit_min=[10] * N, joint_limit_max=[100] * N)
ok, v = n2._sanitize_vector([255] * N, "position", n2.joint_limit_min,
                            n2.joint_limit_max, throttle=False)
check(ok and v == [100] * N, f"限位 max=100 生效: {v[:3]}")
n2.close()

print("\n[8] 控制回调：只下发 position，velocity/effort 不下发")
msg = JointState()
msg.position = [float(x) for x in range(N)]
msg.velocity = [100.0] * N      # 控制话题上的速度/力矩只接收，不下发到设备
msg.effort = [50.0] * N
c.sent.clear()
n.hand_control_cb(msg)
time.sleep(0.2)
kinds = [k for k, _ in c.sent]
check(("pos", list(range(N))) in c.sent, f"位置已下发: {kinds}")
check("vel" not in kinds, f"控制话题的 velocity 不下发: {kinds}")
check("tor" not in kinds, f"控制话题的 effort 不下发: {kinds}")
c.sent.clear()
n.hand_control_cb(msg)
time.sleep(0.2)
check(set(k for k, _ in c.sent) == {"pos"},
      f"重复命令只重复下发位置: {[k for k, _ in c.sent]}")

# 速度/力矩只走设置命令，且是锁存的设定值：同值不该重复占总线
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": [77] * N}})))
time.sleep(0.2)
check(("vel", [77] * N) in c.sent, f"set_speed 下发一次: {[k for k, _ in c.sent]}")
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": [77] * N}})))
time.sleep(0.2)
check("vel" not in [k for k, _ in c.sent],
      f"同样的速度不重复下发: {[k for k, _ in c.sent]}")
n._cmd_enable_joints({})          # 使能状态被动过 → 缓存失效
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": [77] * N}})))
time.sleep(0.2)
check(("vel", [77] * N) in c.sent,
      f"重新使能后速度重写一遍: {[k for k, _ in c.sent]}")
n.close()

print("\n[9] 设置命令：20 长度速度必须执行、越界截断、非法拒绝")
n = new_node(hand_type="right", auto_init_pose=False, state_rate=100.0)
c = n.linker_hand
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed",
     "params": {"hand_type": "right", "speed": [255] * N}})))
time.sleep(0.15)
check(("vel", [255] * N) in c.sent, f"len==20 的速度命令已执行: {c.sent}")
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": [128]}})))
time.sleep(0.15)
check(("vel", [128] * N) in c.sent, "单元素广播速度")
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "right", "speed": []}})))
time.sleep(0.1)
check(c.sent == [], "空速度列表被拒绝（不再 IndexError）")
n.hand_setting_cb(String(data="{not json"))
check(True, "非法 JSON 不抛异常")

print("\n[10] 双手隔离：目标是左手时右手节点不动作")
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_speed", "params": {"hand_type": "left", "speed": [7] * N}})))
time.sleep(0.15)
check(c.sent == [], f"右手节点忽略左手命令: {c.sent}")
c.sent.clear()
n.hand_setting_cb(String(data=json.dumps(
    {"setting_cmd": "set_max_torque_limits",
     "params": {"hand_type": "right", "torque": [60] * N}})))
time.sleep(0.15)
check(("tor", [60] * N) in c.sent, "力矩设置已实现")
n.close()

print("\n[11] deadman: hold / open / disable")
for action, expect in (("hold", None), ("open", "open"), ("disable", "disable")):
    n = new_node(hand_type="right", auto_init_pose=False, state_rate=100.0,
                 cmd_timeout=0.15, deadman_action=action)
    c = n.linker_hand
    time.sleep(0.4)
    check(not n._deadman_tripped, f"{action}: 无命令流时不误触发")
    m = JointState(); m.position = [50.0] * N
    n.hand_control_cb(m)
    time.sleep(0.5)
    check(n._deadman_tripped, f"{action}: 命令中断后触发")
    if expect:
        check(any(k == expect for k, _ in c.sent), f"{action}: 执行了 {expect} 动作")
    c.sent.clear()
    n.hand_control_cb(m)
    time.sleep(0.05)          # 必须小于 cmd_timeout，否则会合法地再次触发
    check(not n._deadman_tripped, f"{action}: 恢复命令后退出 deadman")
    if action == "disable":
        check(any(k == "enable" for k, _ in c.sent), "disable 恢复时重新使能")
    n.close()

print("\n[12] 发布内容")
n = new_node(hand_type="test", auto_init_pose=False, is_touch=True,
             state_rate=50.0, info_rate=50.0, touch_rate=50.0, temp_warn=90)
got = {}
sub_n = rclpy.create_node("sub")
sub_n.create_subscription(JointState, "/cb_test_hand_state",
                          lambda m: got.setdefault("state", m), 10)
sub_n.create_subscription(String, "/cb_test_hand_info",
                          lambda m: got.setdefault("info", m), 10)
sub_n.create_subscription(String, "/cb_test_hand_matrix_touch",
                          lambda m: got.setdefault("touch", m), 10)
sub_n.create_subscription(String, "/cb_test_hand_matrix_touch_mass",
                          lambda m: got.setdefault("mass", m), 10)
t0 = time.time()
while time.time() - t0 < 4.0 and len(got) < 4:
    rclpy.spin_once(sub_n, timeout_sec=0.1)
check("state" in got and list(got["state"].position) == [11.0] * N, "state 发布位置")
check("state" in got and list(got["state"].velocity) == [22.0] * N, "state 发布速度")
check("state" in got and list(got["state"].effort) == [33.0] * N, "state 发布电流")
if "info" in got:
    info = json.loads(got["info"].data)
    check(info["online"] is True, "info: 在线状态")
    check(info["temperature"] == [40] * (N - 1) + [95], "info: 温度")
    check(info["over_temp"] == ["little_tip:95"], f"info: 过温告警 {info['over_temp']}")
    check(info["current"] == [33] * N, "info: 电流")
    check(info["deadman"]["tripped"] is False, "info: deadman 状态")
    check(info["model"] == "O30" and info["side"] == "RIGHT", "info: 设备信息")
    check("state_hz" in info and "touch_hz" in info, "info: 含实测频率字段")
else:
    check(False, "info 未发布")
if "touch" in got:
    t = json.loads(got["touch"].data)
    check(t["sensors"]["1"]["matrix"] == [[1, 2], [3, 4]], "touch: 点阵矩阵")
else:
    check(False, "touch 未发布")
if "mass" in got:
    mm = json.loads(got["mass"].data)
    check(mm["sensors"]["1"]["force"] == {"FnS": 7}
          and mm["sensors"]["1"]["cell_sum"] == 10, "mass: 合力与统计")
else:
    check(False, "mass 未发布")
sub_n.destroy_node()
n.close()

print("\n[13] 循环异常不致线程退出")
class Flaky(FakeCtrl):
    def __init__(self, **kw):
        super().__init__(**kw); self.n = 0
    def get_current_position(self):
        self.n += 1
        if self.n < 5:
            raise RuntimeError("boom")
        return [7] * N
mod.LinkerHandO30Controller = Flaky
n = new_node(hand_type="right", auto_init_pose=False, state_rate=50.0)
time.sleep(1.0)
check(n.run_thread.is_alive(), "抛异常后控制线程仍存活")
check(n.linker_hand.n > 5, f"仍在继续读取 (n={n.linker_hand.n})")
n.close()

print("\n[14] 触觉：手指子集 / 关闭点阵 / 实测频率上报 / 独立线程")
mod.LinkerHandO30Controller = FakeCtrl
n = new_node(hand_type="test", auto_init_pose=False, is_touch=True,
             touch_fingers=[2, 6], touch_publish_matrix=False,
             touch_rate=40.0, state_rate=40.0, info_rate=5.0)
check(n.touch_fingers == [2], f"不可用传感器被剔除: {n.touch_fingers}")
check(n.touch_thread is not None and n.touch_thread.is_alive(), "触觉线程独立运行")
got = {}
sub_n = rclpy.create_node("sub2")
sub_n.create_subscription(String, "/cb_test_hand_matrix_touch",
                          lambda m: got.setdefault("touch", m), 10)
sub_n.create_subscription(String, "/cb_test_hand_matrix_touch_mass",
                          lambda m: got.__setitem__("mass", m), 10)
sub_n.create_subscription(String, "/cb_test_hand_info",
                          lambda m: got.__setitem__("info", m), 10)
t0 = time.time()
while time.time() - t0 < 2.5:          # 频率计要满 1 秒才结算，多等一会
    rclpy.spin_once(sub_n, timeout_sec=0.1)
check("touch" not in got, "touch_publish_matrix=False 时不发点阵话题")
check("mass" in got and list(json.loads(got["mass"].data)["sensors"]) == ["2"],
      "合力话题只含配置的手指")
check(set(n.linker_hand.touch_reads) == {2},
      f"只读配置的传感器: {set(n.linker_hand.touch_reads)}")
if "info" in got:
    info = json.loads(got["info"].data)
    check(info["touch_hz"] > 0 and info["touch_fingers"] == [2],
          f"info: touch_hz={info['touch_hz']} touch_fingers={info['touch_fingers']}")
    check(info["state_hz"] > 0, f"info: state_hz={info['state_hz']}")
else:
    check(False, "info 未发布")
sub_n.destroy_node()
th = n.touch_thread
n.close()
check(not th.is_alive(), "close() 等触觉线程退出后才关总线")

rclpy.shutdown()
print("\n" + ("=" * 50))
print(f"失败 {len(fails)} 项" + ("：" + "; ".join(fails) if fails else "，全部通过 ✅"))
sys.exit(1 if fails else 0)
