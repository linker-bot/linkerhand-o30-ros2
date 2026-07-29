"""
Linker Hand O30 CANFD 控制类

寻址协议：HOP (Hand Object Protocol) v0.0.3
依据文档：HandProtocol_v1.0.pdf（同目录），与 1_协议帧格式0.0.3.xlsx 对照修订。

帧格式（CAN 数据域 = 3 字节帧头 + 载荷）：
    byte0  : bit7 = RTS(0=读实时状态值, 1=读设定值), bit6~0 = MI(主索引/功能码)
    byte1  : SI (子索引 / 对象内起始字节偏移)
    byte2  : EDL(有效数据长度；读请求时=期望返回字节数，写请求时=载荷有效字节数)
    byte3+ : LD (载荷数据，小端、UTF-8)

读写判定：无载荷=读；有载荷且 EDL<=实际载荷=写。
响应帧（标准帧，与协议文档及实测抓包一致）：
    - 返回帧 ID = 请求帧 ID | 0x400；
    - 数据区与请求同构：**回显 3 字节帧头(MI/SI/EDL)，其后才是有效载荷**。
    例：读 UID 发 ID=0x001 data=41 39 05；
        收 ID=0x401 data=41 39 05 | 4F 33 30 69 00 ("O30")。
    出错时回 MI=0x4F 错误码帧(SI=错误索引, EDL=01, 载荷[0]=错误码)。
CAN FD 单帧最大载荷 61 字节；本实现读请求均控制在单帧内，不做分片重组。
帧 ID：标准帧(11 位)请求=设备帧ID(右手 0x01/左手 0x02)，响应= 请求|0x400。
指令示例：
写右手标识：42 49 01 0F
写左手标识：42 49 01 F0
修改标准帧ID：35 00 02 02 00  将手标准帧ID改为02(02 00) 本控制器默认右手为01，左手为02
保存配置到 Flash：35 1E 01 01
断电重连后起效
查询有效关节: 0D 00 24 (从子索引0x00读到0x23, 共36字节; 每字节非0=该关节有效,
              0x0B=存在+可控+反馈; 见 get_valid_joints)
获取所有关节位置：01 00 19 (读子索引0x00~0x18 共25字节物理区, 内含20个有效电机,
              关节类型分组排列, 空洞位无电机; 见 JOINT_MAP)
"""

import os
import threading
import time
import struct
import subprocess
from typing import Optional, List, Dict, Tuple, Callable
from ctypes import *
from enum import Enum

STATUS_OK = 0

# ============================================================================
# 有效电机表（HOP 关节映射，MI=0x00/0x01/0x0C/0x0D 共用此子索引空间）
# ----------------------------------------------------------------------------
# 本表只列 O30 实际存在的 20 个有效电机；不存在的电机一律不收录——以本表为准。
# 物理子索引 si 仍按「关节类型分组」排布：每种类型在硬件内存里占 5 个连续子索引，
# 类型顺序 横滚(roll)→航向(yaw)→指根1(root1)→指根2(root2)→指尖(tip)，组内手指
# 顺序固定 拇指→食指→中指→无名指→小指：
#   0x00~0x04 横滚   0x05~0x09 航向   0x0A~0x0E 指根1
#   0x0F~0x13 指根2  0x14~0x18 指尖
# O30 只装了其中 20 个电机，故收录的 si 不连续(有空洞)：横滚仅拇指(0x00)，指根2
# 无拇指(缺 0x0F)，其余各类齐全。整组偏移写入仍按 JOINT_TYPE_GROUPS 覆盖完整 5 槽
# (空洞槽硬件忽略)，按指多帧见 JOINT_FINGER_SI。物理收发区间仍是 0x00~0x18
# (25 字节, 见 PHYS_SPAN)，内含这 20 个有效电机。
# 换型号时据 get_valid_joints()(发 0D 00 24, 字节非0=有效) 实测结果增删本表条目。
# ============================================================================
JOINT_MAP = {
    0:  {"name": "thumb_roll",    "si": 0x00, "description": "拇指横滚"},
    1:  {"name": "thumb_yaw",     "si": 0x05, "description": "拇指航向"},
    2:  {"name": "index_yaw",     "si": 0x06, "description": "食指航向"},
    3:  {"name": "middle_yaw",    "si": 0x07, "description": "中指航向"},
    4:  {"name": "ring_yaw",      "si": 0x08, "description": "无名指航向"},
    5:  {"name": "little_yaw",    "si": 0x09, "description": "小指航向"},
    6:  {"name": "thumb_root1",   "si": 0x0A, "description": "拇指指根1"},
    7:  {"name": "index_root1",   "si": 0x0B, "description": "食指指根1"},
    8:  {"name": "middle_root1",  "si": 0x0C, "description": "中指指根1"},
    9:  {"name": "ring_root1",    "si": 0x0D, "description": "无名指指根1"},
    10: {"name": "little_root1",  "si": 0x0E, "description": "小指指根1"},
    11: {"name": "index_root2",   "si": 0x10, "description": "食指指根2"},
    12: {"name": "middle_root2",  "si": 0x11, "description": "中指指根2"},
    13: {"name": "ring_root2",    "si": 0x12, "description": "无名指指根2"},
    14: {"name": "little_root2",  "si": 0x13, "description": "小指指根2"},
    15: {"name": "thumb_tip",     "si": 0x14, "description": "拇指指尖"},
    16: {"name": "index_tip",     "si": 0x15, "description": "食指指尖"},
    17: {"name": "middle_tip",    "si": 0x16, "description": "中指指尖"},
    18: {"name": "ring_tip",      "si": 0x17, "description": "无名指指尖"},
    19: {"name": "little_tip",    "si": 0x18, "description": "小指指尖"},
}
NAMES_EN = [info["name"] for info in JOINT_MAP.values()]
NAMES_CN = [info["description"] for info in JOINT_MAP.values()]
# ============================================================================
# 协议常量
# ============================================================================
class RTS:
    """byte0 bit7 —— 返回值切换位（仅读请求有效）"""
    REALTIME = 0   # 读状态：设备当前运行状态/反馈值
    SETTING  = 1   # 读设定值：最近一次成功写入的设定值


class MI:
    """主索引 / 功能码 (byte0 bit6~0)，取值 0x00~0x7F"""
    JOINT_MAP      = 0x00   # 关节逻辑映射表（对象 38 字节）
    POSITION       = 0x01   # 位置          (单字节向量, 36 字节, 0~255)
    VELOCITY       = 0x02   # 速度          (单字节向量)
    ACCEL          = 0x03   # 加速度        (单字节向量)
    CURRENT        = 0x04   # 电流          (单字节向量)
    VOLTAGE        = 0x05   # 电压          (单字节向量)
    TORQUE         = 0x06   # 转矩          (单字节向量)
    TEMPERATURE    = 0x07   # 温度          (单字节向量)
    MOVE_TIME      = 0x08   # 运动时间      (单字节向量, 单位 10ms/格)
    STALL_TIME     = 0x09   # 堵转判定时间
    STALL_THRESH   = 0x0A   # 堵转判定阈值
    STALL_CURRENT  = 0x0B   # 堵转后维持电流
    JOINT_ENABLE   = 0x0C   # 有效关节(可写使能) (单字节向量, 每关节 1 字节位掩码)
    JOINT_FAULT    = 0x0D   # 关节状态/能力位掩码(只读) —— 实测读 0D 00 24 返回每关节存在/
                            #   能力位(非0=有效, 0x0B=存在+可控+反馈, 见 JointBit)；
                            #   用于上电探测有效关节(get_valid_joints)。(spec 名"关节故障")
    HALF_POSITION  = 0x20   # 位置（半字）  (半字向量, 72 字节, 0~65535)
    POS_PID_P      = 0x21
    POS_PID_I      = 0x22
    POS_PID_D      = 0x23
    VEL_PID_P      = 0x24
    VEL_PID_I      = 0x25
    VEL_PID_D      = 0x26
    SPARSE_POS     = 0x30   # 稀疏关节位置（关节号+位置值 成对）
    SENSOR_CMD     = 0x31   # 传感器命令（对象 50 字节）
    SENSOR_DATA1   = 0x32   # 传感器数据通道1（第 0~254 字节，只读）
    SENSOR_DATA2   = 0x33   # 传感器数据通道2（第 255~510 字节）
    SENSOR_DATA3   = 0x34   # 传感器数据通道3（第 511 字节起）
    CONFIG         = 0x35   # 配置（结构体 31 字节）
    ACTION_1       = 0x36   # 预设动作 1（0x36~0x3A 为动作 1~5，各 7 字节）
    PRODUCT_INFO   = 0x41   # 产品信息（结构体 241 字节，只读）
    ENGINEERING    = 0x42   # 工程服务（结构体 222 字节）
    UNIT_RANGE     = 0x43   # 单位与量程（结构体 162 字节）
    ERROR_CODE     = 0x4F   # 通信错误码（结构体 17 字节，只读）


# MI 0x0C 有效关节 —— 每关节 1 字节位掩码
class JointBit:
    PRESENT      = 0x01   # bit0 关节存在
    CONTROLLABLE = 0x02   # bit1 允许控制
    ENABLED      = 0x04   # bit2 已使能
    FEEDBACK     = 0x08   # bit3 支持反馈
    CALIBRATABLE = 0x10   # bit4 支持校准
    ENABLE_DEFAULT = 0x0F  # 存在+可控+已使能+反馈（文档典型上电使能值）


class ProductInfoSI:
    """产品信息子索引 (MI=0x41)，全部 str / 只读 —— (SI, 字节长度)

    ⚠️ 实测本机固件为 HOP 0.0.2：设备唯一标识码 = 32 字节（非 PDF 0.0.3 的 48），
       因此自「协议名称」起的字段相对 PDF 整体左移 0x10。若换 0.0.3 固件，
       需把 DEVICE_UID 改回 48 并将 0x59 及之后字段各 +0x10（用 dump_product_info 核对）。
    """
    MODEL              = (0x00, 16)   # 产品型号全名 例:O30
    VOLTAGE_RANGE      = (0x10, 16)   # 供电电压范围  例:12V~24V
    MCU_UID            = (0x20, 25)   # MCU 唯一标识码
    DEVICE_UID         = (0x39, 32)   # 设备唯一标识码（0.0.2 为 32 字节）
    PROTOCOL_NAME      = (0x59, 8)    # 协议名称 例:HOP
    PROTOCOL_VERSION   = (0x61, 8)    # 协议版本 例:0.0.2
    HW_VER_INTERFACE   = (0x69, 8)    # 接口板硬件版本 例:V1.2.0
    HW_VER_ADAPTER     = (0x71, 8)    # 转接板硬件版本 例:V1.1.0
    HW_VER_CONTROL     = (0x79, 8)    # 控制板硬件版本
    BOOTLOADER_VERSION = (0x81, 8)    # bootloader 软件版本
    APP_VERSION        = (0x89, 8)    # app 程序版本
    MECH_VERSION       = (0x91, 8)    # 机械结构版本
    BUILD_TIME         = (0x99, 32)   # 程序编译时间 例:2026-05-xx xx:xx:50
    SUPPORTED_PROTO    = (0xB9, 32)   # 支持协议/接口类型 例:CAN,UART
    HAND_SIDE          = (0xD9, 8)    # 左右手标识 LEFT/RIGHT


class ConfigSI:
    """配置指令子索引 (MI=0x35)"""
    STD_FRAME_ID   = 0x00   # uint16  标准帧 id (0x0001~0x03FE)
    EXT_FRAME_ID   = 0x02   # uint32  扩展帧 id
    CAN_TYPE       = 0x06   # uint8   0x01=CAN2.0, 0x02=CAN FD
    BRS_ENABLE     = 0x07   # uint8   仅 CAN FD; 0x01=启用波特率切换
    ARB_BAUD       = 0x08   # uint8   仲裁域波特率
    DATA_BAUD      = 0x09   # uint8   数据域波特率(CAN FD)
    SILENT         = 0x0A   # uint8   0x01=不发送响应帧
    CTRL_MODE      = 0x18   # uint8   控制模式
    LED_ENABLE     = 0x19   # uint8   指示灯开关
    BUZZER_ENABLE  = 0x1B   # uint8   蜂鸣器开关
    RESTORE_CONFIG = 0x1C   # uint8   写 0x01 恢复映射+配置为默认并立即写 Flash
    RESTORE_ALL    = 0x1D   # uint8   写 0x01 恢复所有数据(须先写工程服务密码)
    SAVE_TO_FLASH  = 0x1E   # uint8   写 0x01 保存 RAM 待保存区到 Flash


class EngineeringSI:
    """工程服务子索引 (MI=0x42)"""
    PASSWORD        = 0x00   # str8   写入配置密码(默认 123456)
    HEARTBEAT       = 0x45   # uint32 连接指示心跳(只读, 设备周期自增)
    HAND_SIDE       = 0x49   # uint8  0xF0=左手, 0x0F=右手
    SENSOR_TYPE     = 0x4A   # uint8  传感器类型枚举
    DEVICE_UID      = 0xAB   # str48  设备唯一标识码
    UPDATE_APP      = 0xDB   # uint8  写 0x01 跳转 Bootloader
    PROTOCOL_SWITCH = 0xDD   # uint8  0x00=HOP, 0x01=旧版CAN协议(须保存重启)


class CtrlMode:
    """控制模式 (ConfigSI.CTRL_MODE 取值)，默认 0x01"""
    POSITION      = 0x01   # 位置模式
    VELOCITY      = 0x02   # 速度模式
    CURRENT       = 0x03   # 电流模式
    TORQUE        = 0x04   # 转矩模式
    POSITION_TIME = 0x05   # 位置时间模式
    OPEN_LOOP     = 0x06   # 开环模式
    COMPLIANT     = 0x07   # 柔顺控制
    TEACH         = 0x08   # 教学习模式
    DRAG          = 0x09   # 拖动模式


class ErrorCode:
    """通信错误码 (MI=0x4F)，位掩码可组合"""
    OK              = 0x00
    MI_NOT_EXIST    = 0x01   # 主索引不存在
    SI_NOT_EXIST    = 0x02   # 子索引不存在
    NOT_WRITABLE    = 0x04   # 寄存器不可写
    LEN_MISMATCH    = 0x08   # 数据长度不匹配
    NO_PERMISSION   = 0x10   # 权限不足
    DATA_PADDED     = 0x20   # 返回数据有补零
    VALUE_INVALID   = 0x40   # 数据值非法


class SensorFinger:
    """选择传感器 (MI 0x31 SI 0x2D)"""
    THUMB  = 1   # 大拇指
    INDEX  = 2   # 食指
    MIDDLE = 3   # 中指
    RING   = 4   # 无名指
    PINKY  = 5   # 小拇指
    PALM   = 6   # 手掌


class SensorType:
    """传感器类型字符串枚举 (MI 0x31 SI 0x00)"""
    NO_SENSOR     = "NO_SENSOR"
    TASHAN_GATHER = "TASHAN_GATHER"   # 他山(外部采集板)
    HUAWEIKE      = "HUAWEIKE"        # 华威科
    FULAI         = "FULAI"           # 福莱
    TSSP_GENGLE   = "TSSP_GENGLE"     # TSSP 福莱适配版
    TSSP_JZG      = "TSSP_JZG"        # TSSP 晶致感


# 工程服务 0x42/0x4A 传感器类型 uint8 枚举 → 名称（见 HandProtocol 传感器类型表）
SENSOR_TYPE_ENUM = {
    0x00: "无传感器",
    0x01: "TS(采集板)",
    0x02: "HWK",
    0x03: "FL",
    0x04: "TSSP_GENGLE",
    0x05: "TSSP_JZG",
}


class Gesture:
    """预设手势类型 (MI 0x36~0x3A SI 0x00)"""
    NONE         = 0x00
    NUMBER_1     = 0x01   # 0x01~0x0A 为数字 1~10
    NUMBER_10    = 0x0A
    HALF_GRIP    = 0x0B   # 半握
    CIRCLE       = 0x0C   # 画圆
    ELLIPSE      = 0x0D   # 画椭圆
    FINGER_DANCE = 0x0E   # 手指舞
    ROCK         = 0x0F   # 石头
    SCISSORS     = 0x10   # 剪刀
    PAPER        = 0x11   # 布
    GRASP        = 0x12   # 抓握
    EXTEND       = 0x13   # 伸展
    PACK         = 0x14   # 打包


# 有效关节的“逻辑顺序”—— O30 共 20 个有效电机（不存在的电机不收录，见 JOINT_MAP）。
# ⚠️ 0D 00 24 自动探测在本机返回错误，已停用；有效电机按下表静态固定，不再自动探测。
# 顺序按「关节类型」分组（与上位机 / GUI 滑块顺序一致）：
#   横滚×5指 → 航向×5指 → 指根1×5指 → 指根2×5指 → 指尖×5指；手指顺序 拇→食→中→无名→小。
# 本机物理子索引同样按类型分组(见 JOINT_MAP)，故此向量顺序与物理子索引 si 完全一致
# (向量下标 == si)。打包/解包仍按各值的 si 落到物理位置(见 _pack_u8 / _unpack_u8)。
_VALID_NAME2SI = {j["name"]: j["si"] for j in JOINT_MAP.values()}
_VALID_NAME2CN = {j["name"]: j["description"] for j in JOINT_MAP.values()}
_ACTIVE_FINGERS = ["thumb", "index", "middle", "ring", "little"]  # 拇→食→中→无名→小
_ACTIVE_TYPES   = ["roll", "yaw", "root1", "root2", "tip"]        # 横滚→航向→指根1→指根2→指尖
# 按「类型分组、组内拇→食→中→无名→小」展开，仅保留 JOINT_MAP 收录的有效电机
# (横滚缺食/中/无名/小，指根2 缺拇)，故需按成员过滤，跳过本机不存在的组合。
ACTIVE_JOINTS = [(f"{f}_{t}", _VALID_NAME2SI[f"{f}_{t}"])
                 for t in _ACTIVE_TYPES for f in _ACTIVE_FINGERS
                 if f"{f}_{t}" in _VALID_NAME2SI]
CN_NAMES = {si: _VALID_NAME2CN[n] for n, si in ACTIVE_JOINTS}  # 物理子索引 si -> 中文名称
JOINT_NAMES = [n for n, _ in ACTIVE_JOINTS]
NUM_JOINTS = len(ACTIVE_JOINTS)          # 20
# 名称 -> 物理子索引，便于按名寻址单个关节
JOINT_SI = {n: si for n, si in ACTIVE_JOINTS}
# 收发覆盖物理子索引 0x00~max(si) 这段连续区(本机 0x00~0x18, 25 字节)，含全部有效关节
PHYS_SPAN = max(si for _, si in ACTIVE_JOINTS) + 1   # 25
# 逻辑关节总数(含腕/掌/预留)，用于 get_valid_joints 全量探测
LOGICAL_JOINT_COUNT = 0x24               # 36
POS_MIN, POS_MAX = 0, 65535              # 半字位置取值范围
U8_MAX = 255                             # 单字节位置/向量取值上限（本机位置通道）

FINGER_ORDER = ["thumb", "index", "middle", "ring", "little"]   # 类型组内手指顺序 拇→食→中→无名→小
JOINT_TYPE_ORDER = ["roll", "yaw", "root1", "root2", "tip"]     # 关节类型顺序 横滚→航向→指根1→指根2→指尖

# 类型组 —— 同一关节类型的五指在内存中连续(roll/yaw/root1/root2/tip 各占 5 个连续子索引)，
# 可整组一次性偏移写入(单帧)。组内手指顺序见 FINGER_ORDER。
JOINT_TYPE_GROUPS: Dict[str, Tuple[int, int]] = {
    "roll":  (0x00, 5),
    "yaw":   (0x05, 5),
    "root1": (0x0A, 5),
    "root2": (0x0F, 5),
    "tip":   (0x14, 5),
}
# 同类关节跨五指的子索引（连续）。例 root1 跨五指 = [0x0A,0x0B,0x0C,0x0D,0x0E]
JOINT_TYPE_SI: Dict[str, List[int]] = {
    t: [base + i for i in range(cnt)]
    for t, (base, cnt) in JOINT_TYPE_GROUPS.items()
}
# 同一手指跨类型的子索引（非连续，按手指整体动作时用）。
# 例 拇指 = [0x00,0x05,0x0A,0x0F,0x14]
JOINT_FINGER_SI: Dict[str, List[int]] = {
    f: [JOINT_TYPE_GROUPS[t][0] + i for t in JOINT_TYPE_ORDER]
    for i, f in enumerate(FINGER_ORDER)
}


# ============================================================================
# 设备 / 驱动结构体
# ============================================================================
class DeviceID(Enum):
    RIGHT_HAND = 0x01
    LEFT_HAND = 0x02
    BOOTLOADER = 0x03
    BROADCAST = 0xFF


# 标准帧(11 位 ID)：响应 ID = 请求 ID | 0x400（HOP 协议文档 & 实测抓包一致）。
REPLY_ID_MASK = 0x400
STD_ID_MASK = 0x7FF
# 标准帧广播探测 ID：设备无视自身配置 ID 均响应，用于未知 ID 时建立通信
DISCOVERY_STD_ID = 0x7FF


class CanFD_Config(Structure):
    _fields_ = [
        ("NomBaud", c_uint),
        ("DatBaud", c_uint),
        ("NomPres", c_ushort),
        ("NomTseg1", c_char),
        ("NomTseg2", c_char),
        ("NomSJW", c_char),
        ("DatPres", c_char),
        ("DatTseg1", c_char),
        ("DatTseg2", c_char),
        ("DatSJW", c_char),
        ("Config", c_char),
        ("Model", c_char),
        ("Cantype", c_char)
    ]


class CanFD_Msg(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("FrameType", c_ubyte),
        ("DLC", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("BusSatus", c_ubyte),
        ("ErrSatus", c_ubyte),
        ("TECounter", c_ubyte),
        ("RECounter", c_ubyte),
        ("Data", c_ubyte * 64)
    ]


# ============================================================================
# 通信层（传输）—— 与协议/逻辑解耦
# ----------------------------------------------------------------------------
# 控制器只依赖通信对象的三件事：开关硬件、发一帧标准帧(send_frame)、把收到的原始帧
# 通过回调 on_frame(can_id, payload) 交回控制器。响应匹配/组帧/协议语义全在控制器。
# CANFDCommunication      : 厂商私有库 libcanbus.so（默认）。
# SocketCANCommunication  : 内核原生 can0 + python-can，用于「透明塑封 USB 转 CANFD 设备」，
#                           不依赖 libcanbus.so。帧格式与厂商设备一致(11 位标准帧)。
# ============================================================================
class CANFDCommunication:
    """基于厂商私有库 libcanbus.so 的 CANFD 传输层（11 位标准帧）。

    只负责：加载/扫描/打开设备、CANFD 初始化、发送单帧、后台接收线程把每帧
    解码为 (can_id, payload) 交给 on_frame 回调。不参与组帧/响应匹配。
    """

    DLC2LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

    def __init__(self, canfd_device: int = 0, channel: int = 0):
        self.canfd_device = canfd_device
        self.channel = channel
        self.canDLL = None
        self.is_connected = False
        self.running = False
        self._thread: Optional[threading.Thread] = None
        # 收到一帧原始数据时回调：on_frame(can_id: int, payload: bytes)
        self.on_frame: Optional[Callable[[int, bytes], None]] = None

    @staticmethod
    def _get_dlc(length: int) -> int:
        if length <= 8:    return length
        elif length <= 12: return 9
        elif length <= 16: return 10
        elif length <= 20: return 11
        elif length <= 24: return 12
        elif length <= 32: return 13
        elif length <= 48: return 14
        else:              return 15

    def initialize(self) -> bool:
        """初始化 CANFD 通信并启动接收线程"""
        try:
            CDLL("/usr/local/lib/libusb-1.0.so", RTLD_GLOBAL)
            time.sleep(0.1)
            self.canDLL = cdll.LoadLibrary("/usr/local/lib/libcanbus.so")

            print("=" * 50)
            print("开始扫描 CANFD 设备...")
            ret = self.canDLL.CAN_ScanDevice()
            print(f"找到 {ret} 个设备")
            if ret <= 0:
                print("❌ 未找到 CANFD 设备")
                return False

            ret = self.canDLL.CAN_OpenDevice(self.canfd_device, self.channel)
            if ret != STATUS_OK:
                print(f"❌ 打开设备失败: {ret}")
                return False
            print("✅ 设备打开成功")

            can_config = CanFD_Config(1000000, 5000000, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0x04, 0x0, 0x1)
            ret = self.canDLL.CANFD_Init(self.canfd_device, self.channel,
                                         byref(can_config))
            if ret != STATUS_OK:
                print(f"❌ CANFD 初始化失败: {ret}")
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                return False
            print("✅ CANFD 初始化成功")

            ret = self.canDLL.CAN_SetFilter(self.canfd_device, self.channel,
                                            0, 0, 0, 0, 1)
            if ret != STATUS_OK:
                print(f"❌ 设置过滤器失败: {ret}")
                self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                return False
            print("✅ 过滤器设置成功")

            self.is_connected = True
            self.running = True
            self._thread = threading.Thread(target=self._receive_loop, daemon=False)
            self._thread.start()
            time.sleep(0.05)

            if self._thread.is_alive():
                print("✅ 接收线程已启动")
            else:
                print("❌ 接收线程启动失败")
                return False
            return True

        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _receive_loop(self):
        while self.running and self.is_connected:
            try:
                msg_array = (CanFD_Msg * 100)()
                msg_ptr = cast(msg_array, POINTER(CanFD_Msg))
                ret = self.canDLL.CANFD_Receive(
                    self.canfd_device, self.channel, msg_ptr, 100, 10)
                if ret > 0:
                    for i in range(ret):
                        msg = msg_array[i]
                        dlc = msg.DLC
                        data_len = (self.DLC2LEN[dlc]
                                    if dlc < len(self.DLC2LEN) else 64)
                        payload = bytes(msg.Data[:data_len])
                        if self.on_frame is not None:
                            self.on_frame(msg.ID, payload)
            except Exception:
                pass
            time.sleep(0.002)

    def send_frame(self, can_id: int, data: bytes = b'') -> bool:
        """发送一个 CANFD 标准帧(11 位 ID)"""
        if not self.is_connected:
            return False
        try:
            dlc = self._get_dlc(len(data))
            data_array = (c_ubyte * 64)()
            for i, b in enumerate(data[:64]):
                data_array[i] = b
            # FrameType=4(CANFD), ExternFlag=0(标准帧 11 位 ID)
            msg = CanFD_Msg(can_id, 0, 4, dlc, 0, 0, 0, 0, 0, 0, data_array)
            ret = self.canDLL.CANFD_Transmit(self.canfd_device, self.channel,
                                             byref(msg), 1, 200)
            return ret >= 1
        except Exception:
            return False

    def close(self):
        """关闭通信（停接收线程、关设备）"""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
            print("✅ 接收线程已停止")
        if self.canDLL and self.is_connected:
            self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
            print("✅ CANFD 设备已关闭")
        self.is_connected = False


class SocketCANCommunication(CANFDCommunication):
    """基于内核原生 SocketCAN (python-can) 的 CANFD 传输层。

    用于「透明塑封 USB 转 CANFD 设备」：该设备在 Linux 下走标准 SocketCAN，
    无需厂商私有库(libcanbus.so)，通过内核 can0 接口 + python-can 收发。
    继承 CANFDCommunication，仅重写底层传输方法；帧格式与厂商设备完全一致
    （O30 为 11 位标准帧），故上层 Controller 无需任何改动。
    """

    def __init__(self, channel: str = "can0", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True):
        self.channel = channel      # 接口名，如 "can0"
        self.bitrate = bitrate      # 仲裁段波特率，默认 1Mbps（与 O30 libcanbus 一致）
        self.dbitrate = dbitrate    # 数据段波特率，默认 5Mbps
        self.auto_setup = auto_setup
        self.bus = None
        self.is_connected = False
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.on_frame: Optional[Callable[[int, bytes], None]] = None

    def _setup_interface(self):
        """自动配置 can0 接口（需要 sudo 权限）

        等价于手动执行：
            sudo ip link set can0 down
            sudo ip link set can0 type can bitrate <b> dbitrate <d> fd on restart-ms 100
            sudo ip link set can0 up
            sudo ip link set can0 txqueuelen 1000
        逐条容错，失败不中断（接口可能已由用户提前配置）。
        """
        cmds = [
            # (命令, 是否必需)。restart-ms 部分控制器不支持，单独下发且允许失败，
            # 避免它把 bitrate/dbitrate 这条关键配置一起带崩。
            (["sudo", "ip", "link", "set", self.channel, "down"], True),
            (["sudo", "ip", "link", "set", self.channel, "type", "can",
              "bitrate", str(self.bitrate), "dbitrate", str(self.dbitrate),
              "fd", "on"], True),
            (["sudo", "ip", "link", "set", self.channel, "type", "can",
              "restart-ms", "100"], False),  # 总线BUS-OFF自动恢复（部分设备不支持，可忽略）
            (["sudo", "ip", "link", "set", self.channel, "up"], True),
            (["sudo", "ip", "link", "set", self.channel, "txqueuelen", "1000"], True),
        ]
        for c, required in cmds:
            try:
                ret = subprocess.run(c, check=False, timeout=5,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if ret.returncode != 0:
                    err = ret.stderr.decode(errors='ignore').strip()
                    if required:
                        print(f"[SocketCAN] 接口配置命令失败: {' '.join(c)}  原因: {err}")
                    else:
                        print(f"[SocketCAN] 提示: 该设备不支持的可选项已跳过: {' '.join(c)} ({err})")
                time.sleep(0.1)  # 给内核时间完成状态切换（尤其 down 之后）
            except Exception as e:
                print(f"[SocketCAN] 接口配置命令异常: {' '.join(c)} -> {e}")

        # 回读实际生效的时序与状态，便于定位波特率不匹配 / BUS-OFF
        try:
            ret = subprocess.run(["ip", "-details", "link", "show", self.channel],
                                 check=False, timeout=5,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out = ret.stdout.decode(errors='ignore')
            timing = next((l.strip() for l in out.splitlines() if "bitrate" in l), "(未读到)")
            state = next((l.strip() for l in out.splitlines() if "state" in l), "")
            print(f"[SocketCAN] 期望 bitrate={self.bitrate} dbitrate={self.dbitrate}")
            print(f"            实际生效: {timing}")
            print(f"            状态行  : {state}")
            if "BUS-OFF" in state or "ERROR-PASSIVE" in state:
                print("[SocketCAN] ⚠️ 总线处于错误状态，通常是波特率与设备不一致或接线/终端电阻问题。")
        except Exception:
            pass

    def initialize(self) -> bool:
        """初始化 SocketCAN 通信并启动接收线程"""
        try:
            import can
        except ImportError:
            print("❌ 缺少 python-can 库，请执行: pip install python-can")
            return False

        try:
            print("=" * 50)
            print(f"开始初始化 SocketCAN 设备 (通道: {self.channel})...")

            # 自动拉起接口
            if self.auto_setup:
                print(f"正在自动配置接口 {self.channel} "
                      f"(bitrate={self.bitrate}, dbitrate={self.dbitrate}, fd on)...")
                self._setup_interface()
                time.sleep(0.2)

            # 打开总线前先确认接口存在，避免 python-can 抛出难懂异常
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                print(f"❌ 未找到网络接口 {self.channel}。")
                print("   该透明塑封设备需拨到 Linux 模式才会枚举为原生 CAN 接口；")
                print("   若 lsusb 显示为 'STM32 Virtual ComPort' 则说明仍是串口模式。")
                print("   请：1) 将 type-c 下方开关拨到 Linux 模式  2) 重新插拔USB")
                print(f"      3) 用 `ip -br link show type can` 确认 {self.channel} 出现")
                self.is_connected = False
                self.bus = None
                return False

            # 打开 CANFD 总线
            self.bus = can.interface.Bus(channel=self.channel,
                                         interface='socketcan', fd=True)
            self.is_connected = True

            self.running = True
            self._thread = threading.Thread(target=self._receive_loop, daemon=False)
            self._thread.start()
            time.sleep(0.05)
            if not self._thread.is_alive():
                print("❌ 接收线程启动失败")
                return False

            print(f"✅ SocketCAN 通道 {self.channel} 打开成功")
            print("✅ SocketCAN 通信初始化完成")
            print("=" * 50)
            return True

        except Exception as e:
            print(f"❌ SocketCAN 初始化失败: {e}")
            print("   请检查:")
            print(f"   1. 设备是否接入、type-c 下方开关是否拨到 Linux 模式")
            print(f"   2. 接口 {self.channel} 是否存在 (ip -br link show type can)")
            print(f"   3. 是否具备 sudo 权限以自动配置接口")
            self.is_connected = False
            self.bus = None
            return False

    def _receive_loop(self):
        while self.running and self.is_connected and self.bus is not None:
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg is not None and self.on_frame is not None:
                    self.on_frame(msg.arbitration_id, bytes(msg.data))
            except Exception:
                pass

    def send_frame(self, can_id: int, data: bytes = b'') -> bool:
        """发送一个 CANFD 标准帧(11 位 ID)"""
        if not self.is_connected or self.bus is None:
            return False
        try:
            import can
            data = bytes(data)
            data_len = min(len(data), 64)
            data = data[:data_len]
            # 零填充到合法 CANFD 帧长度（0-8,12,16,20,24,32,48,64）
            dlc = self._get_dlc(data_len)
            padded_len = self.DLC2LEN[dlc]
            payload = data + b'\x00' * (padded_len - data_len)

            msg = can.Message(
                arbitration_id=can_id,
                is_extended_id=False,   # O30 为 11 位标准帧
                is_fd=True,
                # 关闭 BRS：实测 BRS 开启 + 高速数据段会把总线打入 BUS-OFF，设备无法应答；
                # BRS 关闭时数据段保持仲裁波特率，稳定收发。
                bitrate_switch=False,
                data=payload,
            )
            self.bus.send(msg, timeout=0.2)
            return True
        except Exception as e:
            print(f"发送消息异常(SocketCAN): {e}")
            return False

    def close(self):
        """关闭 SocketCAN 连接（停接收线程、关总线）"""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
            print("✅ 接收线程已停止")
        if self.bus is not None:
            try:
                self.bus.shutdown()
                print("✅ SocketCAN 连接已关闭")
            except Exception as e:
                print(f"关闭 SocketCAN 连接失败: {e}")
        self.is_connected = False
        self.bus = None


# ============================================================================
# 控制类
# ============================================================================
class LinkerHandO30Controller:
    """Linker Hand O30 灵巧手 CANFD 控制类（HOP v0.0.3 协议）"""

    DLC2LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

    def __init__(self, hand_type: str = "right", canfd_device: int = 0,
                 frame_id: Optional[int] = None, comm_type: str = "libcanbus",
                 channel: str = "can0", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True):
        """
        hand_type   : "left"/"right"，仅作标签；HOP 协议左右手实际靠帧 ID 或读
                      产品信息(0x41/0xE9)区分，不内嵌于 CAN ID。
        frame_id    : 设备标准帧 ID(11 位)。不指定时按惯例 右手=0x01 / 左手=0x02
                      （同总线双手需各自配置不同 ID）。
        comm_type   : 通信后端 —— "libcanbus"(默认, 厂商私有库) 或
                      "socketcan"(内核原生 can0 + python-can, 透明塑封 USB-CANFD 设备)。
        canfd_device: 仅 libcanbus 用——设备序号。
        channel     : libcanbus 下为通道号(默认 0)；socketcan 下为接口名(默认 "can0")。
        bitrate/dbitrate/auto_setup : 仅 socketcan 用——仲裁/数据段波特率与是否自动拉起接口。
        """
        self.hand_type = hand_type
        if frame_id is None:
            frame_id = (DeviceID.LEFT_HAND.value if hand_type == "left"
                        else DeviceID.RIGHT_HAND.value)
        self.device_id = frame_id
        self.canfd_device = canfd_device
        self.comm_type = comm_type

        # 请求帧 / 返回帧的 CAN ID（标准帧）
        self._tx_id = self.device_id
        self._rx_id = self.device_id | REPLY_ID_MASK

        # 通信层（传输）：按 comm_type 选择后端，收到的原始帧经 on_frame 回调进入
        # _process_response。协议组帧/响应匹配仍全部在本控制器。
        if comm_type == "socketcan":
            self.comm: CANFDCommunication = SocketCANCommunication(
                channel=channel, bitrate=bitrate, dbitrate=dbitrate,
                auto_setup=auto_setup)
        else:
            ch = channel if isinstance(channel, int) else 0
            self.comm = CANFDCommunication(canfd_device=canfd_device, channel=ch)
        self.comm.on_frame = self._process_response

        # 缓存
        self.product_info: Dict[str, str] = {}
        self.last_error_code: int = ErrorCode.OK

        # 响应匹配状态保护锁
        self._lock = threading.Lock()

        # 事务串行锁：一次只允许一个请求在途，发完等到回复/超时才放行下一帧，
        # 防止上一帧响应未到就发新帧导致响应被覆盖/错认。
        self._txn_lock = threading.Lock()

        # 同步请求/应答：按当前在途请求的回显 MI 关联响应（杜绝写回显/陈旧帧污染）
        self._pending_event: Optional[threading.Event] = None
        self._pending_resp: Optional[bytes] = None
        self._pending_mi: Optional[int] = None
        self._last_tx: bytes = b''

        # 有效关节集合（向量型控制/解析的唯一依据）：先用静态 JOINT_MAP(valid=True)
        # 派生的默认值占位；initialize() 连接成功后会用 get_valid_joints() 实测结果
        # 覆盖（检测失败则保留此默认）。所有 set_target_position 等方法均按此集合
        # 接收/下发数据，做到“收有效电机的数据、按有效电机下发”。
        self._apply_active_joints(ACTIVE_JOINTS)
        self.cn_names: Dict[int, str] = dict(CN_NAMES)

        self.initialize()
        self.hand_info = self.get_device_info()

    @property
    def is_connected(self) -> bool:
        """通信是否已连接（代理到通信层）"""
        return self.comm.is_connected

    # ------------------------------------------------------------------ #
    # 初始化 / 关闭
    # ------------------------------------------------------------------ #
    def initialize(self) -> bool:
        """初始化通信层（委托给 self.comm）并完成上电准备"""
        if not self.comm.initialize():
            return False

        # 有效关节按 ACTIVE_JOINTS 静态固定（0D 00 24 自动探测在本机报错，已停用）。
        print(f"✅ 有效关节 {self.num_joints} 个")
        print("✅ 初始化完成")
        print("=" * 50)
        return True

    def close(self):
        """关闭控制器（委托通信层关闭硬件/总线与接收线程）"""
        print("正在关闭控制器...")
        self.comm.close()
        print("✅ 控制器已关闭")

    # ------------------------------------------------------------------ #
    # 接收 / 分发
    # ------------------------------------------------------------------ #
    def _process_response(self, can_id: int, payload: bytes):
        """通信层接收回调：处理返回帧。

        标准帧：响应 ID = 请求 ID | 0x400，数据区与请求同构——**前 3 字节回显
        MI/SI/EDL 帧头，其后才是有效载荷**。这里只按 ID 收下整帧裸数据(含帧头)，
        由 _request 校验帧头并剥离。读请求均控制在单帧内(<=61B)，不做分片重组。
        出错时设备返回 MI=0x4F 错误码帧，由 _request 通过回显 MI 不符识别。
        DLC→长度解码已在通信层完成，这里收到的 payload 即有效字节。
        """
        if (can_id & STD_ID_MASK) != self._rx_id:
            return

        with self._lock:
            ev = self._pending_event
            if ev is None or not payload:
                return
            # 仅当回显 MI 与在途请求匹配（或为 0x4F 错误帧）时才认作本次响应，
            # 否则忽略——防止写回显/陈旧帧覆盖正在等待的读响应。
            resp_mi = payload[0] & 0x7F
            if resp_mi == self._pending_mi or resp_mi == MI.ERROR_CODE:
                self._pending_resp = payload
                ev.set()

    # ------------------------------------------------------------------ #
    # 组帧 / 收发
    # ------------------------------------------------------------------ #
    @staticmethod
    def _frame(mi: int, si: int, edl: int, data: bytes = b'', rts: int = 0) -> bytes:
        """构造数据负载：byte0=RTS|MI, byte1=SI, byte2=EDL, byte3+=data"""
        return bytes([((rts & 1) << 7) | (mi & 0x7F), si & 0xFF, edl & 0xFF]) + data

    def _send_frame(self, can_id: int, data: bytes = b'') -> bool:
        """发送一个 CANFD 标准帧(11 位 ID)——记录 TX 后委托通信层发送。"""
        with self._lock:                 # 记录以便过滤可能的 TX 回显
            self._last_tx = bytes(data)
        return self.comm.send_frame(can_id, data)

    def _request(self, mi: int, si: int, length: int,
                 rts: int = RTS.REALTIME, timeout: float = 0.3) -> Optional[bytes]:
        """发送读请求并等待返回；校验回显帧头后返回剥离帧头、裁剪到 length 的载荷。
        超时 / 帧头不符 / 设备回错误码帧，均返回 None。

        事务串行：整个「发送→等回复→取走」过程持 _txn_lock，确保同一时刻只有
        一个请求在途，新请求会等上一请求拿到回复或超时后才发出，避免响应被覆盖。
        """
        with self._txn_lock:
            ev = threading.Event()
            with self._lock:
                self._pending_event = ev
                self._pending_resp = None
                self._pending_mi = mi & 0x7F

            self._send_frame(self._tx_id, self._frame(mi, si, length, b'', rts))
            got = ev.wait(timeout)

            with self._lock:
                body = self._pending_resp
                self._pending_event = None
                self._pending_resp = None
                self._pending_mi = None

        if not got or body is None or len(body) < 3:
            return None
        # 响应回显 3 字节帧头(MI/SI/EDL)；MI 不符 → 错误帧或串扰，丢弃
        resp_mi = body[0] & 0x7F
        if resp_mi != (mi & 0x7F):
            if resp_mi == MI.ERROR_CODE and len(body) >= 4:
                self.last_error_code = body[3]
                print(f"⚠️ 设备返回错误码 0x{body[3]:02X}（请求 MI=0x{mi:02X} SI=0x{si:02X}）")
            return None
        return body[3:3 + length]

    def _write(self, mi: int, si: int, data: bytes) -> bool:
        """通用写：EDL = 载荷长度。持 _txn_lock 发送，避免插入到某个读事务中间。"""
        with self._txn_lock:
            return self._send_frame(self._tx_id, self._frame(mi, si, len(data), data))

    # ------------------------------------------------------------------ #
    # 关节数据打包 / 解包（仅当前有效关节，集合由 _detect_active_joints 实测得到）
    # ------------------------------------------------------------------ #
    def _pack_u16(self, values: List[int]) -> bytes:
        buf = bytearray(self.phys_span * 2)
        for (_, si), v in zip(self.active_joints, values):
            struct.pack_into('<H', buf, si * 2, max(POS_MIN, min(POS_MAX, int(v))))
        return bytes(buf)

    def _unpack_u16(self, body: bytes) -> List[int]:
        out = []
        for _, si in self.active_joints:
            off = si * 2
            out.append(struct.unpack_from('<H', body, off)[0]
                       if off + 2 <= len(body) else 0)
        return out

    def _pack_u8(self, values: List[int]) -> bytes:
        buf = bytearray(self.phys_span)
        for (_, si), v in zip(self.active_joints, values):
            buf[si] = int(v) & 0xFF
        return bytes(buf)

    def _unpack_u8(self, body: bytes) -> List[int]:
        return [body[si] if si < len(body) else 0 for _, si in self.active_joints]

    # ================================================================== #
    # 关节使能 / 故障（上电流程必需）
    # ================================================================== #
    def enable_all_joints(self) -> bool:
        """使能全部有效关节（写 0x0F=存在+可控+已使能+反馈）。上电后写位置前必须执行。"""
        return self._write(MI.JOINT_ENABLE, 0x00,
                           self._pack_u8([JointBit.ENABLE_DEFAULT] * self.num_joints))

    def set_joint_enable(self, masks: List[int]) -> bool:
        """逐关节设置使能位掩码（见 JointBit）"""
        if len(masks) != self.num_joints:
            return False
        return self._write(MI.JOINT_ENABLE, 0x00, self._pack_u8(masks))

    def get_joint_enable(self) -> Optional[List[int]]:
        body = self._request(MI.JOINT_ENABLE, 0x00, self.phys_span)
        return self._unpack_u8(body) if body else None

    def get_joint_fault(self) -> Optional[List[int]]:
        """读取有效关节的状态/能力位掩码 (MI=0x0D, 见 JointBit)。
        ⚠️ spec 名为「关节故障」，但本固件返回的是存在/可控/使能/反馈等能力位
           (非故障码)；按 si 探测整机有效关节请用 get_valid_joints()。"""
        body = self._request(MI.JOINT_FAULT, 0x00, self.phys_span)
        return self._unpack_u8(body) if body else None

    def get_valid_joints(self) -> Optional[Dict[int, Dict]]:
        """探测本机有效关节（发 0D 00 24：从子索引 0x00 读到 0x23，共 36 字节）。

        上电流程第一步——返回每字节为该逻辑关节的存在/能力位掩码(见 JointBit)：
        非 0 即有效(实测 0x0B=存在+可控+反馈)，0 为无效(无对应电机)。结果按 si
        与 JOINT_MAP 比对，返回 {si: {name, description, mask}} 仅含有效关节。
        读取失败返回 None。可据此增删 JOINT_MAP 的条目(换型号时)。
        """
        body = self._request(MI.JOINT_FAULT, 0x00, LOGICAL_JOINT_COUNT)
        if not body:
            return None
        si2info = {j["si"]: j for j in JOINT_MAP.values()}
        valid: Dict[int, Dict] = {}
        for si in range(min(len(body), LOGICAL_JOINT_COUNT)):
            if body[si] != 0:
                info = si2info.get(si, {"name": f"si_0x{si:02X}", "description": "未知"})
                valid[si] = {"name": info["name"],
                             "description": info["description"],
                             "mask": body[si]}
        return valid

    def print_valid_joints(self):
        """打印 get_valid_joints() 结果，便于上电核对有效/可控关节。"""
        valid = self.get_valid_joints()
        if valid is None:
            print("❌ 无法读取有效关节（0D 00 24 超时/失败）")
            return
        print("=" * 50)
        print(f"有效关节（共 {len(valid)} 个，0D 00 24 实测）:")
        for si, v in valid.items():
            print(f"  0x{si:02X}  {v['name']:14s} {v['description']:8s} mask=0x{v['mask']:02X}")
        print("=" * 50)

    # ---- 有效关节集合的应用 / 实测刷新（向量型控制方法的唯一数据源） ---- #
    def _apply_active_joints(self, active: List[Tuple[str, int]]):
        """用 [(name, si), ...] 刷新当前有效关节集合及其派生量：
        num_joints / phys_span / joint_names / joint_si。
        set_target_position 等向量方法据此校验入参长度、按 si 打包下发。"""
        self.active_joints: List[Tuple[str, int]] = list(active)
        self.joint_names: List[str] = [n for n, _ in self.active_joints]
        self.joint_si: Dict[str, int] = {n: si for n, si in self.active_joints}
        self.num_joints: int = len(self.active_joints)
        self.phys_span: int = (max(si for _, si in self.active_joints) + 1
                               if self.active_joints else 0)

    def _apply_valid_dict(self, valid: Dict[int, Dict]):
        """用 get_valid_joints() 的 {si: {name, description, mask}} 刷新有效关节，
        集合按物理子索引 si 升序排列（与连续打包顺序一致）。"""
        active = [(info["name"], si) for si, info in sorted(valid.items())]
        self._apply_active_joints(active)
        self.cn_names = {si: info["description"] for si, info in valid.items()}

    def _detect_active_joints(self) -> bool:
        """实测有效关节(发 0D 00 24)并刷新当前有效关节集合。
        成功返回 True；读取失败返回 False 且保持原集合不变。"""
        valid = self.get_valid_joints()
        if not valid:
            return False
        self._apply_valid_dict(valid)
        return True

    # ================================================================== #
    # 位置控制（单字节 uint8 0~255, MI=0x01）—— 本机 0.0.2 固件的位置通道
    # ⚠️ 半字 0x20 在本固件读回全 0(疑未实现)，故位置默认走单字节；半字版见
    #    *_u16 变体，仅在未来固件支持时备用。
    # ================================================================== #
    def set_target_position(self, positions: List[int]) -> bool:
        """设置全部有效关节的目标位置（单字节 0~255, MI=0x01）。
        入参个数须等于当前实测有效关节数 self.num_joints，按各关节 si 连续打包下发。"""
        if len(positions) != self.num_joints:
            print(f"❌ 需要 {self.num_joints} 个关节位置，收到 {len(positions)}")
            return False
        return self._write(MI.POSITION, 0x00, self._pack_u8(positions))

    def get_current_position(self) -> Optional[List[int]]:
        """读取全部有效关节的实时位置（单字节 0~255, MI=0x01, RTS=0）。"""
        body = self._request(MI.POSITION, 0x00, self.phys_span, RTS.REALTIME)
        return self._unpack_u8(body) if body else None

    def get_target_position(self) -> Optional[List[int]]:
        """读取全部有效关节的设定位置（单字节 0~255, MI=0x01, RTS=1）。"""
        body = self._request(MI.POSITION, 0x00, self.phys_span, RTS.SETTING)
        return self._unpack_u8(body) if body else None

    # ---- 半字(uint16, MI=0x20)变体：本机 0.0.2 读回全 0，疑未实现，仅备用 ---- #
    def set_target_position_u16(self, positions: List[int]) -> bool:
        if len(positions) != self.num_joints:
            return False
        return self._write(MI.HALF_POSITION, 0x00, self._pack_u16(positions))

    def get_current_position_u16(self) -> Optional[List[int]]:
        body = self._request(MI.HALF_POSITION, 0x00, self.phys_span * 2, RTS.REALTIME)
        return self._unpack_u16(body) if body else None

    def set_sparse_position(self, pairs: List[Tuple[int, int]]) -> bool:
        """稀疏关节位置控制 (MI 0x30)：pairs=[(关节号, 位置0~255), ...]，省带宽。
        关节号即逻辑/物理子索引(与 MI=0x01 同一类型分组索引空间，见 JOINT_MAP)。
        例：弯五指 root1 → [(0x0A,200),(0x0B,200),(0x0C,200),(0x0D,200),(0x0E,200)]
        """
        data = bytearray()
        for jn, pos in pairs:
            data += bytes([jn & 0xFF, int(pos) & 0xFF])
        return self._write(MI.SPARSE_POS, 0x00, bytes(data))

    # ---- 按指 / 按名 / 按类型 局部写位置（单字节, 只发涉及关节，不必凑齐 25 个） ---- #
    def set_group_position(self, group: str, values: List[int]) -> bool:
        """写入某一「手指」的目标位置（单字节 0~255, MI=0x01）。

        group : 手指名 —— "thumb"/"index"/"middle"/"ring"/"little"
        values: 该指关节位置(0~255)，顺序 横滚→航向→指根1→指根2→指尖
                (见 JOINT_TYPE_ORDER)。可少于 5 个，则从该指第 1 个关节(横滚)起依次写入。
        位置语义与 set_target_position / get_current_position 完全一致。
        ⚠️ O30 物理布局按关节类型分组，单根手指的 5 个关节子索引不连续
           (如食指=[0x01,0x06,0x0B,0x10,0x15])，故逐关节下发(多帧)。
        例：食指整指弯曲 → set_group_position("index", [0,0,200,200,200])
        """
        if group not in JOINT_FINGER_SI:
            print(f"❌ 未知手指 {group}，可选 {list(JOINT_FINGER_SI)}")
            return False
        sis = JOINT_FINGER_SI[group]
        if not values or len(values) > len(sis):
            print(f"❌ {group} 最多 {len(sis)} 个值，收到 {len(values)}")
            return False
        ok = True
        for si, v in zip(sis, values):
            ok = self._write(MI.POSITION, si,
                             bytes([max(0, min(U8_MAX, int(v)))])) and ok
        return ok

    def set_joint_position(self, joint: str, value: int) -> bool:
        """按名称写单个关节目标位置（单字节 0~255, MI=0x01）。joint 见 self.joint_names。"""
        if joint not in self.joint_si:
            print(f"❌ 未知关节 {joint}，可选 {self.joint_names}")
            return False
        return self._write(MI.POSITION, self.joint_si[joint],
                           bytes([max(0, min(U8_MAX, int(value)))]))

    def set_joints(self, mapping: Dict[str, int]) -> bool:
        """按名称批量写多个关节目标位置（单字节 0~255, MI=0x01），逐关节单独下发。
        用于内存中不连续的关节集合（如跨手指的同类关节），各帧独立。
        """
        ok = True
        for name, value in mapping.items():
            ok = self.set_joint_position(name, value) and ok
        return ok

    def set_joint_type_position(self, joint_type: str, values) -> bool:
        """把某一「关节类型」在五指上同时写到目标位置（单字节 0~255, MI=0x01）。

        joint_type : JOINT_TYPE_ORDER 之一 —— roll/yaw/root1/root2/tip。
         物理布局下同一类型的五指连续(如 root1=0x0A~0x0E)，整组单帧偏移写入。
        values     : 标量(广播到五指) 或 长度 5 的列表(顺序 拇→食→中→无名→小)。
        """
        if joint_type not in JOINT_TYPE_GROUPS:
            print(f"❌ 未知关节类型 {joint_type}，可选 {list(JOINT_TYPE_GROUPS)}")
            return False
        base, count = JOINT_TYPE_GROUPS[joint_type]
        if isinstance(values, (list, tuple)):
            if len(values) != count:
                print(f"❌ {joint_type} 需要 {count} 个值，收到 {len(values)}")
                return False
            vals = list(values)
        else:
            vals = [values] * count
        data = bytes(max(0, min(U8_MAX, int(v))) for v in vals)
        return self._write(MI.POSITION, base, data)

    def bend_finger_roots(self, value: int = 200) -> bool:
        """五指指根1(MCP/掌指关节)同时弯曲（单字节 0~255, 默认 200=0xC8）。
        ⚠️ 类型分组布局下五指 root1(0x0A~0x0E)连续，整组单帧偏移写入。
        ⚠️ 200 这一端为「弯」；想伸直写另一端(如 30 附近，见 get_current_position 静止值)。
        """
        return self.set_joint_type_position("root1", value)

    def open_palm(self, value: int = 0, wait: bool = False,
                  timeout: float = 3.0) -> bool:
        """手掌张开：把五指的指根1(root1)与指尖(tip)伸直到 value(0~255, 默认 0=最直)。

        与 bend_finger_roots 反方向（弯曲端≈200, 伸直端≈0, 静止≈25~37）。
        只动弯曲关节，不碰横滚/航向/指根2，避免意外姿态——如需展开手指(航向外展)
        再单独调 set_joint_type_position("yaw", [...])，但航向端值需先标定。
        wait=True 时阻塞轮询 root1/tip 这 10 个关节到位（容差 8）或超时。
        """
        v = max(0, min(U8_MAX, int(value)))
        ok = self.set_joint_type_position("root1", v)
        ok = self.set_joint_type_position("tip", v) and ok
        if not ok or not wait:
            return ok
        # root1/tip 这 10 个关节在 get_current_position() 结果中的下标
        idx = [self.joint_names.index(n) for n in
               ("thumb_root1", "index_root1", "middle_root1", "ring_root1",
                "little_root1", "thumb_tip", "index_tip", "middle_tip",
                "ring_tip", "little_tip")]
        start = time.time()
        while time.time() - start < timeout:
            cur = self.get_current_position()
            if cur and all(abs(cur[i] - v) <= 8 for i in idx):
                return True
            time.sleep(0.05)
        print("⚠️ 张开到位超时")
        return False

    # ================================================================== #
    # 速度 / 电流 / 转矩 / 温度（单字节 uint8, 0~255）
    # ================================================================== #
    def set_target_velocity(self, velocities: List[int]) -> bool:
        if len(velocities) != self.num_joints:
            return False
        return self._write(MI.VELOCITY, 0x00, self._pack_u8(velocities))

    def get_current_velocity(self) -> Optional[List[int]]:
        body = self._request(MI.VELOCITY, 0x00, self.phys_span, RTS.REALTIME)
        return self._unpack_u8(body) if body else None

    def set_target_torque(self, torques: List[int]) -> bool:
        if len(torques) != self.num_joints:
            return False
        return self._write(MI.TORQUE, 0x00, self._pack_u8(torques))

    def get_motor_current(self) -> Optional[List[int]]:
        body = self._request(MI.CURRENT, 0x00, self.phys_span, RTS.REALTIME)
        return self._unpack_u8(body) if body else None

    def get_temperature(self) -> Optional[List[int]]:
        body = self._request(MI.TEMPERATURE, 0x00, self.phys_span, RTS.REALTIME)
        return self._unpack_u8(body) if body else None

    def set_move_time(self, times: List[int]) -> bool:
        """设置运动时间，单位 10ms/格（配合位置实现匀速到位）"""
        if len(times) != self.num_joints:
            return False
        return self._write(MI.MOVE_TIME, 0x00, self._pack_u8(times))

    # ================================================================== #
    # 配置（MI=0x35）
    # ================================================================== #
    def write_config(self, si: int, data: bytes) -> bool:
        """通用配置写（仅更新 RAM，需 save_to_flash 才掉电保留）"""
        return self._write(MI.CONFIG, si, data)

    def set_control_mode(self, mode: int) -> bool:
        """设置控制模式（见 CtrlMode），默认位置模式"""
        return self._write(MI.CONFIG, ConfigSI.CTRL_MODE, bytes([mode]))

    def save_to_flash(self) -> bool:
        """将当前 RAM 待保存区(映射/配置/工程可持久字段)写入 Flash"""
        return self._write(MI.CONFIG, ConfigSI.SAVE_TO_FLASH, bytes([0x01]))

    # ================================================================== #
    # 工程服务（MI=0x42）
    # ================================================================== #
    def get_heartbeat(self) -> Optional[int]:
        """读心跳计数（设备周期自增），用于在线检测"""
        body = self._request(MI.ENGINEERING, EngineeringSI.HEARTBEAT, 4)
        return int.from_bytes(body[:4], 'little') if body and len(body) >= 4 else None

    def is_online(self) -> bool:
        """连续两次心跳计数递增则判定在线"""
        h1 = self.get_heartbeat()
        if h1 is None:
            return False
        time.sleep(0.1)
        h2 = self.get_heartbeat()
        return h2 is not None and h2 != h1

    def get_sensor_type(self) -> Optional[str]:
        """读传感器类型（工程服务 0x42/0x4A 的 uint8 枚举，见 SENSOR_TYPE_ENUM）。
        报文 42 4A 01。未知值返回 '未知(0xNN)'，读取失败返回 None。"""
        b = self._request(MI.ENGINEERING, EngineeringSI.SENSOR_TYPE, 1)
        if not b:
            return None
        return SENSOR_TYPE_ENUM.get(b[0], f"未知(0x{b[0]:02X})")

    # ================================================================== #
    # 错误码（MI=0x4F，只读）
    # ================================================================== #
    def get_last_error(self) -> Optional[int]:
        """读最新通信错误码（0x00=正常，见 ErrorCode）"""
        idx = self._request(MI.ERROR_CODE, 0x00, 1)
        return idx[0] if idx else None

    # ================================================================== #
    # 触觉传感器（MI 0x31 命令 + 0x32/0x33/0x34 数据通道）
    # ================================================================== #
    def select_sensor(self, finger: int) -> bool:
        """选择要读取的手指传感器（见 SensorFinger，1拇指 ~ 6手掌）"""
        return self._write(MI.SENSOR_CMD, 0x2D, bytes([finger & 0xFF]))

    def get_sensor_total_length(self) -> Optional[int]:
        """读当前所选传感器的数据总长度（字节，小端 uint16）"""
        body = self._request(MI.SENSOR_CMD, 0x2B, 2)
        return int.from_bytes(body[:2], 'little') if body and len(body) >= 2 else None

    def get_sensor_info(self) -> Optional[Dict]:
        """读传感器元信息（须先 select_sensor）：类型/单位/量程/行/列/总长"""
        typ = self._request(MI.SENSOR_CMD, 0x00, 32)
        if typ is None:
            return None
        unit   = self._request(MI.SENSOR_CMD, 0x20, 8)  or b''
        srange = self._request(MI.SENSOR_CMD, 0x28, 1)  or b'\x00'
        rows   = self._request(MI.SENSOR_CMD, 0x29, 1)  or b'\x00'
        cols   = self._request(MI.SENSOR_CMD, 0x2A, 1)  or b'\x00'
        total  = self.get_sensor_total_length() or 0
        return {
            "type":         typ.decode('utf-8', 'ignore').rstrip('\x00'),
            "unit":         unit.decode('utf-8', 'ignore').rstrip('\x00'),
            "range":        srange[0],
            "rows":         rows[0],
            "cols":         cols[0],
            "total_length": total,
        }

    def _read_sensor_channel(self, mi: int, count: int) -> bytes:
        """从某数据通道(本地 SI 偏移 0 起)分段读 count 字节，每段≤61(CAN FD 单帧)。
        SI 为单字节，故单通道最多覆盖 256 字节(协议分通道正是此原因)。"""
        out = bytearray()
        off = 0
        while off < count and off <= 0xFF:
            n = min(61, count - off, 0x100 - off)
            body = self._request(mi, off, n)
            if not body:
                break
            out += body
            if len(body) < n:   # 设备返回不足，认为读到末尾
                break
            off += n
        return bytes(out)

    def read_tactile_raw(self, finger: Optional[int] = None) -> Optional[bytes]:
        """读取触觉完整原始数据（文档§2.7 流程）。
        finger 给定时先选指；为 None 时读当前已选传感器。
        通道划分：0x32=[0,255)  0x33=[255,511)  0x34=[511, 末尾)。
        """
        if finger is not None:
            if not self.select_sensor(finger):
                return None
            time.sleep(0.005)
        total = self.get_sensor_total_length()
        if not total:
            return None
        data = bytearray()
        data += self._read_sensor_channel(MI.SENSOR_DATA1, min(255, total))
        if total > 255:
            data += self._read_sensor_channel(MI.SENSOR_DATA2, min(256, total - 255))
        if total > 511:
            data += self._read_sensor_channel(MI.SENSOR_DATA3, total - 511)
        return bytes(data[:total])

    def read_tactile_matrix(self, finger: int) -> Optional[List[List[int]]]:
        """读取触觉并按行优先还原为二维矩阵。
        每格字节数 = 总长 ÷ (行×列)，自动判定 1 或 2 字节/格(小端)。
        """
        if not self.select_sensor(finger):
            return None
        time.sleep(0.005)
        info = self.get_sensor_info()
        if not info:
            return None
        rows, cols, total = info["rows"], info["cols"], info["total_length"]
        if rows == 0 or cols == 0:
            return None
        raw = self.read_tactile_raw(finger=None)
        if raw is None:
            return None
        cells = rows * cols
        bpc = 2 if cells and (total // cells) >= 2 else 1
        vals = []
        for i in range(cells):
            if bpc == 2 and (i * 2 + 2) <= len(raw):
                vals.append(int.from_bytes(raw[i*2:i*2+2], 'little'))
            elif bpc == 1 and i < len(raw):
                vals.append(raw[i])
            else:
                vals.append(0)
        return [vals[r*cols:(r+1)*cols] for r in range(rows)]

    # ================================================================== #
    # 预设动作 / 手势（MI 0x36~0x3A，各 7 字节）
    # ================================================================== #
    def trigger_gesture(self, gesture: int, speed: int = 128, amplitude: int = 100,
                        loops: int = 1, finger: int = 0, slot: int = 1,
                        wait: bool = False, timeout: float = 10.0) -> bool:
        """触发预设手势。
        gesture   : 手势类型（见 Gesture）
        speed     : 动作速度 0~255
        amplitude : 动作幅值 0~255
        loops     : 循环次数 0~255
        finger    : 手指指定，1~5=拇指~小指，0=全手
        slot      : 动作槽位 1~5 → MI 0x36~0x3A
        wait      : True 时轮询执行标志直至完毕
        """
        if not (1 <= slot <= 5):
            return False
        mi = MI.ACTION_1 + (slot - 1)
        payload = bytes([gesture & 0xFF, speed & 0xFF, amplitude & 0xFF,
                         loops & 0xFF, finger & 0xFF])
        if not self._write(mi, 0x00, payload):
            return False
        if not wait:
            return True
        start = time.time()
        time.sleep(0.05)
        while time.time() - start < timeout:
            if self.get_gesture_status(slot) == 0x00:
                return True
            time.sleep(0.05)
        print("⚠️ 手势执行超时")
        return False

    def get_gesture_status(self, slot: int = 1) -> Optional[int]:
        """读动作执行标志：0x00=完毕，0x01=执行中"""
        if not (1 <= slot <= 5):
            return None
        body = self._request(MI.ACTION_1 + (slot - 1), 0x05, 1)
        return body[0] if body else None

    def set_gesture_powerup_default(self, slot: int, enable: bool) -> bool:
        """设置该动作槽位是否上电默认执行（需 save_to_flash 持久化）"""
        if not (1 <= slot <= 5):
            return False
        return self._write(MI.ACTION_1 + (slot - 1), 0x06, bytes([1 if enable else 0]))

    # ================================================================== #
    # 产品信息（MI=0x41，str / 只读）
    # ================================================================== #
    def _read_string(self, si_len: tuple, timeout: float = 0.3) -> Optional[str]:
        si, length = si_len
        body = self._request(MI.PRODUCT_INFO, si, length, RTS.REALTIME, timeout)
        if body is None:
            return None
        return body.decode("utf-8", "ignore").rstrip("\x00")

    def get_product_model(self) -> Optional[str]:
        """获取产品型号（SI=0x00, 16 字节字符串）"""
        return self._read_string(ProductInfoSI.MODEL)

    def get_device_uid(self) -> Optional[str]:
        return self._read_string(ProductInfoSI.DEVICE_UID)

    def get_protocol_version(self) -> Optional[str]:
        return self._read_string(ProductInfoSI.PROTOCOL_VERSION)

    def get_hand_side(self) -> Optional[str]:
        """读左右手。优先工程服务(0x42/0x49: 0xF0=左, 0x0F=右)——不依赖产品信息
        字段偏移，最可靠；失败再回退到产品信息字符串。"""
        b = self._request(MI.ENGINEERING, EngineeringSI.HAND_SIDE, 1)
        if b:
            if b[0] == 0xF0:
                return "LEFT"
            if b[0] == 0x0F:
                return "RIGHT"
        return self._read_string(ProductInfoSI.HAND_SIDE)

    def dump_product_info(self, total: int = 0xF0, chunk: int = 48):
        """逐段读产品信息(MI 0x41)并按 偏移/ASCII 打印，用于核对真实字段偏移。
        换固件或字段错位时跑这个，对照打印结果调整 ProductInfoSI 偏移即可。"""
        print("=" * 60)
        print("产品信息原始转储 (偏移: ASCII)：")
        off = 0
        while off < total and off <= 0xFF:
            n = min(chunk, total - off, 0x100 - off)
            body = self._request(MI.PRODUCT_INFO, off, n)
            if not body:
                print(f"  0x{off:02X}: <读取失败/超时>")
                break
            asc = ''.join(chr(x) if 32 <= x < 127 else '·' for x in body)
            print(f"  0x{off:02X} ({len(body):2d}B): {asc}")
            off += len(body)
        print("=" * 60)

    def get_device_info(self) -> Dict[str, Optional[str]]:
        """读取一组常用产品信息字段（上电优先识别）"""
        fields = {
            "产品型号":     ProductInfoSI.MODEL,
            "供电电压范围": ProductInfoSI.VOLTAGE_RANGE,
            "设备唯一标识": ProductInfoSI.DEVICE_UID,
            "协议名称":     ProductInfoSI.PROTOCOL_NAME,
            "协议版本":     ProductInfoSI.PROTOCOL_VERSION,
            "控制板版本":   ProductInfoSI.HW_VER_CONTROL,
            #"app版本":     ProductInfoSI.APP_VERSION,
            "编译时间":     ProductInfoSI.BUILD_TIME,
            "支持协议":     ProductInfoSI.SUPPORTED_PROTO,
            "左右手":       ProductInfoSI.HAND_SIDE,
        }
        info: Dict[str, Optional[str]] = {
            name: self._read_string(si_len) for name, si_len in fields.items()
        }
        # 传感器类型不在产品信息(0x41)，走工程服务(0x42/0x4A)单独读
        info["传感器类型"] = self.get_sensor_type()
        self.hand_info = info  # 缓存，供外部访问
        return info

    # ================================================================== #
    # 高级控制
    # ================================================================== #
    def setup(self, verify_joints: bool = False) -> bool:
        """上电初始化：① 位置模式 → ② 使能全部有效关节。

        有效关节按 ACTIVE_JOINTS 静态固定（0D 00 24 探测在本机报错，已停用）。
        verify_joints=True 时仅额外用 get_valid_joints() 做一次诊断性比对并告警，
        不会改动静态有效关节集合（避免覆盖类型分组的向量顺序）。"""
        if verify_joints:
            valid = self.get_valid_joints()
            if valid is None:
                print("⚠️ 有效关节探测失败(0D 00 24)，按静态 ACTIVE_JOINTS 继续")
            else:
                detected = set(valid.keys())
                expected = {si for _, si in self.active_joints}
                if detected != expected:
                    print(f"⚠️ 实测有效关节与静态表不一致：实测 si="
                          f"{sorted(detected)}，期望 si={sorted(expected)}（仅告警，不改动）")
                else:
                    print(f"✅ 有效关节核对通过（{len(detected)} 个）")
        ok = self.set_control_mode(CtrlMode.POSITION)
        time.sleep(0.01)
        ok = self.enable_all_joints() and ok
        time.sleep(0.01)
        return ok

    def move_to_position(self, positions: List[int], wait: bool = False,
                         tolerance: int = 8, timeout: float = 10.0) -> bool:
        """移动到目标位置（单字节 0~255）；wait=True 时阻塞直到到位或超时。"""
        if not self.set_target_position(positions):
            return False
        if not wait:
            return True
        start = time.time()
        while time.time() - start < timeout:
            cur = self.get_current_position()
            if cur and all(abs(c - t) <= tolerance for c, t in zip(cur, positions)):
                return True
            time.sleep(0.05)
        print("⚠️ 运动超时")
        return False

    def open_hand(self, wait: bool = True) -> bool:
        """张开手 —— ⚠️ 各关节张开方向(0 或 255)需按实物标定后调整"""
        return self.move_to_position([POS_MIN] * self.num_joints, wait=wait)

    def close_hand(self, wait: bool = True) -> bool:
        """握拳 —— ⚠️ 各关节方向需按实物标定后调整"""
        return self.move_to_position([U8_MAX] * self.num_joints, wait=wait)

    # ================================================================== #
    # 工具方法
    # ================================================================== #
    def print_device_info(self):
        info = self.get_device_info()
        print("=" * 50)
        print("设备信息:")
        for k, v in info.items():
            print(f"  {k}: {v}")
        print("=" * 50)

    def print_joint_positions(self):
        pos = self.get_current_position()
        if not pos:
            print("❌ 无法获取关节位置")
            return
        print("=" * 50)
        print("当前关节位置:")
        for i, (name, p) in enumerate(zip(self.joint_names, pos)):
            print(f"  {i:2d}. {name:15s}: {p:5d}")
        print("=" * 50)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ============================================================================
# 示例
# ============================================================================
if __name__ == "__main__":
    with LinkerHandO30Controller(hand_type="right") as hand:
        if hand.initialize():
            # 上电识别
            print("产品型号:", hand.get_product_model())
            print("协议版本:", hand.get_protocol_version())
            print("左右手  :", hand.get_hand_side())
            hand.print_device_info()
            # 有效关节已按 ACTIVE_JOINTS 静态固定（0D 00 24 探测在本机报错，已停用）
            # hand.print_valid_joints()
            # 位置模式 + 使能全部有效关节
            hand.setup()
            hand.set_target_position([80] * hand.num_joints)  # 先发个中位，确认通信正常
            time.sleep(2)
            hand.open_palm(wait=True)       # 张开（伸直）
            hand.print_joint_positions()
