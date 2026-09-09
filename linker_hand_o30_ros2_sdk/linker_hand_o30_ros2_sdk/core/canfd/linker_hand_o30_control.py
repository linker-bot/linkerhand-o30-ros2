"""
Linker Hand O30 CANFD 控制类

寻址协议：HOP (Hand Object Protocol)
依据文档：**O30-HOP协议-20260907.xlsx（同目录，协议文档版本 v0.0.4 / 2026.08.07）**
          —— 本文件的全部 MI/SI/长度/枚举以该表为唯一准绳，与旧 HandProtocol_v1.0.pdf
          冲突处一律以 xlsx 为准（尤其是传感器 0x31 与产品信息 0x41 的子索引偏移）。

帧格式（CAN 数据域 = 3 字节帧头 + 载荷）：
    byte0  : bit7 = RTS(0=读实时状态值, 1=读设定值), bit6~0 = MI(主索引/功能码)
    byte1  : SI (子索引 / 对象内起始字节偏移)
    byte2  : EDL(有效数据长度；读请求时=期望返回字节数，写请求时=载荷有效字节数)
    byte3+ : LD (载荷数据，多字节小端 LSB first、字符串 UTF-8)

读写判定：无载荷=读；有载荷且 EDL<=实际载荷=写。
响应帧（标准帧，与协议文档及实测抓包一致）：
    - 返回帧 ID = 请求帧 ID | 0x400（请求 ID 最高位置 1）；
    - 数据区与请求同构：**回显 3 字节帧头(MI/SI/EDL)，其后才是有效载荷**。
    例：读 UID 发 ID=0x001 data=41 39 05；
        收 ID=0x401 data=41 39 05 | 4F 33 30 69 00 ("O30")。
    出错时回 MI=0x4F 错误码帧(SI=错误索引, EDL=01, 载荷[0]=错误码位掩码)。
CAN FD 单帧最大载荷 64 字节 → 单次可读写有效数据 61 字节(MAX_LD)；超长对象按
SI 偏移分多帧顺序读取（见 _read_plan / 传感器分通道读取）。
帧 ID：标准帧(11 位)请求=设备帧ID(右手 0x01/左手 0x02)，响应= 请求|0x400；
      0x7FF 为特殊探测 ID，设备无论自身 ID 均响应（DISCOVERY_STD_ID）。
指令示例：
写右手标识：42 49 01 0F
写左手标识：42 49 01 F0
修改标准帧ID：35 00 02 02 00  将手标准帧ID改为02(02 00) 本控制器默认右手为01，左手为02
保存配置到 Flash：35 1E 01 01
断电重连后起效
查询有效关节: 0D 00 24 (MI 0x0D=有效关节；从子索引0x00读到0x23 共36字节，
              每字节非0=该关节有效; 关节故障是 MI 0x0C，见 JointFault)
获取所有关节位置：01 00 19 (读子索引0x00~0x18 共25字节物理区, 内含20个有效电机,
              关节类型分组排列, 空洞位无电机; 见 JOINT_MAP)
传感器（MI 0x31 命令 + 0x32/0x33/0x34 数据）：先选传感器 31 6E 01 0x，再按缓存的
              数据总长度整段读数据通道。总长度/行列/单位/**数据分布描述**等元信息在
              initialize() 阶段一次性探测并缓存(见 probe_sensors)，高频读取时不再发
              元信息帧。
              数据区排布由「数据分布描述」(0x2B)定义，例：
                  FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40
              = 法向力合力 + 切向力合力 + 切向力合力方向 + 保留 + 40 点阵值。
              **合力值没有独立寄存器，就在数据区开头，与点阵一次读回**；解析见
              parse_tactile / get_tactile_force / get_tactile_summary。
              一次读同时要「矩阵 + 合力」用 get_tactile_data / get_all_tactile_data
              （返回字典 key 全英文，合力 key 即协议标签 FnS/FtS/FdS）。
              该规则与 O6 一致，可对照同目录 O6_HOP协议帧格式0.0.4.xlsx「传感器」表。
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

# libcanbus 接收线程空闲时的轮询间隔（秒）。见 CANFDCommunication._receive_loop：
# 同步请求的应答延迟上限就是这个值，高频触觉读取对它敏感，不要往回调大。
RX_IDLE_POLL_SLEEP = 0.0002

# ============================================================================
# 有效电机表（HOP 关节映射，MI=0x00~0x0F 共用此单字节子索引空间）
# ----------------------------------------------------------------------------
# 本表只列 O30 实际存在的 20 个有效电机；不存在的电机一律不收录——以本表为准，
# 与 xlsx「基本信息」的有效关节勾选表一致（横滚仅大拇指、指根2 无大拇指）。
# 物理子索引 si 按 xlsx「单字节数据子索引与关节对应顺序」——「关节类型分组」排布：
# 每种类型占 5 个连续子索引，类型顺序 横滚(roll)→航向(yaw)→指根1(root1)→
# 指根2(root2)→指尖(tip)，组内手指顺序固定 拇指→食指→中指→无名指→小指：
#   0x00~0x04 横滚   0x05~0x09 航向   0x0A~0x0E 指根1
#   0x0F~0x13 指根2  0x14~0x18 指尖
#   0x19~0x1B 手腕(roll/yaw/pitch, 本机无)  0x1C~0x1E 手掌(本机无)
#   0x1F~0x23 备用关节一~五(本机无)
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
    """主索引 / 功能码 (byte0 bit6~0)，取值 0x00~0x7F —— 见 xlsx「协议格式」表"""
    JOINT_MAP      = 0x00   # 关节映射表（36 个映射索引 + 映射长度0x24 + 启用标志0x25）
    POSITION       = 0x01   # 位置          (单字节向量, 36 字节, 0~255)
    VELOCITY       = 0x02   # 速度限制/目标速度   (单字节向量)
    ACCEL          = 0x03   # 加速度限制         (单字节向量)
    CURRENT        = 0x04   # 电流限制/目标电流   (单字节向量)
    VOLTAGE        = 0x05   # 电压限制/目标电压   (单字节向量)
    TORQUE         = 0x06   # 转矩限制/目标转矩   (单字节向量)
    TEMPERATURE    = 0x07   # 温度限制           (单字节向量)
    MOVE_TIME      = 0x08   # 运动时间      (单字节向量, 单位 10ms/格)
    STALL_TIME     = 0x09   # 堵转判定时间
    STALL_THRESH   = 0x0A   # 堵转判定阈值
    STALL_CURRENT  = 0x0B   # 堵转后维持电流
    JOINT_FAULT    = 0x0C   # 关节故障(只读位掩码, 每关节 1 字节, 见 JointFault)
    JOINT_ENABLE   = 0x0D   # 有效关节(可读写, 每关节 1 字节能力/使能位掩码, 见 JointBit)
                            #   上电探测有效关节即读此项：0D 00 24
    INC_POSITION   = 0x0E   # 增量型位置控制——增（单字节向量, 各关节目标位置 +=）
    DEC_POSITION   = 0x0F   # 增量型位置控制——减（单字节向量, 各关节目标位置 -=）
    HALF_POSITION  = 0x20   # 位置（半字）  (半字向量, 72 字节, 0~65535)
    POS_PID_P      = 0x21
    POS_PID_I      = 0x22
    POS_PID_D      = 0x23
    VEL_PID_P      = 0x24
    VEL_PID_I      = 0x25
    VEL_PID_D      = 0x26
    SPARSE_POS     = 0x30   # 稀疏关节位置（关节号+位置值 成对，本机支持前 19 组）
    SENSOR_CMD     = 0x31   # 传感器命令/元信息（对象 115 字节，见 SensorSI）
    SENSOR_DATA1   = 0x32   # 传感器数据通道1（数据第 0~254 字节，只读）
    SENSOR_DATA2   = 0x33   # 传感器数据通道2（数据第 255~509 字节，只读）
    SENSOR_DATA3   = 0x34   # 传感器数据通道3（数据第 510~764 字节，只读）
    CONFIG         = 0x35   # 配置（结构体 32 字节，见 ConfigSI）
    ACTION_1       = 0x36   # 预设动作 1（0x36~0x3A 为动作 1~5，各 7 字节）
    PRODUCT_INFO   = 0x41   # 产品信息（结构体 249 字节，只读，见 ProductInfoSI）
    ENGINEERING    = 0x42   # 调试功能A / 工程服务（见 EngineeringSI）
    UNIT_RANGE     = 0x43   # 控制量单位及物理数值范围查询（先写 SI0x00 指定所查 MI）
    DEBUG_B        = 0x44   # 调试功能B（各关节行程起点位置偏置 int8 向量）
    MOTOR_STATS    = 0x45   # 关节电机状态统计（先写 SI0x00 指定所查状态类型）
    ERROR_CODE     = 0x4F   # 通信错误码（SI0x00=最新错误索引 + 15 条历史，只读）


# MI 0x0D 有效关节 —— 每关节 1 字节位掩码（协议表未逐位列出，取实测含义）
class JointBit:
    PRESENT      = 0x01   # bit0 关节存在
    CONTROLLABLE = 0x02   # bit1 允许控制
    ENABLED      = 0x04   # bit2 已使能
    FEEDBACK     = 0x08   # bit3 支持反馈
    CALIBRATABLE = 0x10   # bit4 支持校准
    ENABLE_DEFAULT = 0x0F  # 存在+可控+已使能+反馈（文档典型上电使能值）


class JointFault:
    """MI 0x0C 关节故障 —— 每关节 1 字节位掩码（xlsx「关节故障码附表」）"""
    HAND_STALL     = 0x01   # bit0 灵巧手层判定执行器堵转（较易触发，条件可调）
    MOTOR_STALL    = 0x02   # bit1 执行器层判定堵转（较难触发，多数情况可调）
    OVER_CURRENT   = 0x04   # bit2 执行器过流
    OVER_TEMP      = 0x08   # bit3 执行器过温
    MOTOR_ABNORMAL = 0x40   # bit6 执行器异常（电机/编码器/驱动器等不可恢复故障）
    MOTOR_OFFLINE  = 0x80   # bit7 执行器离线（无法与执行器建立通信）


# 故障位 → 名称，供 decode_joint_fault 逐位翻译
JOINT_FAULT_NAMES = {
    JointFault.HAND_STALL:     "灵巧手层判定堵转",
    JointFault.MOTOR_STALL:    "执行器层判定堵转",
    JointFault.OVER_CURRENT:   "执行器过流",
    JointFault.OVER_TEMP:      "执行器过温",
    JointFault.MOTOR_ABNORMAL: "执行器异常",
    JointFault.MOTOR_OFFLINE:  "执行器离线",
}


class ProductInfoSI:
    """产品信息子索引 (MI=0x41)，全部 str / 只读 —— (SI, 字节长度)

    偏移完全按 xlsx「产品信息子索引」表(v0.0.4)：设备唯一标识码 32 字节，
    三块硬件版本各 16 字节（旧 0.0.2 固件为 8 字节，故 0x99 之后整体右移）。
    换固件若发现字段错位，用 dump_product_info() 打印原始区再校准。
    """
    MODEL              = (0x00, 16)   # 产品型号全名 例:O30
    VOLTAGE_RANGE      = (0x10, 16)   # 产品供电电压范围  例:12V~24V
    MCU_UID            = (0x20, 25)   # MCU 唯一标识码
    DEVICE_UID         = (0x39, 32)   # 设备唯一标识码 例:LHT20XXXXXXXXXXXX
    PROTOCOL_NAME      = (0x59, 8)    # 协议名称 例:HOP
    PROTOCOL_VERSION   = (0x61, 8)    # 协议版本 例:1.0.0
    HW_VER_INTERFACE   = (0x69, 16)   # 接口板硬件名称及版本 例:241024/V1.2.0
    HW_VER_ADAPTER     = (0x79, 16)   # 转接板硬件名称及版本 例:241024/V1.1.0
    HW_VER_CONTROL     = (0x89, 16)   # 控制板硬件名称及版本 例:241024/V0.9.0
    BOOTLOADER_VERSION = (0x99, 8)    # bootloader 软件版本
    APP_VERSION        = (0xA1, 8)    # app 程序版本 例:1.0.3
    MECH_VERSION       = (0xA9, 8)    # 机械结构版本
    BUILD_TIME         = (0xB1, 32)   # 程序编译时间 例:2025-12-26 18:41:22
    SUPPORTED_PROTO    = (0xD1, 32)   # 支持协议/接口类型 例:CAN
    HAND_SIDE          = (0xF1, 8)    # 左右手标识 LEFT/RIGHT


class ConfigSI:
    """配置指令子索引 (MI=0x35)。⚠️ 标 [不支持] 者本机固件未实现（xlsx 该列为「否」）"""
    STD_FRAME_ID     = 0x00   # uint16  标准帧 id (0x000~0x3FE)
    EXT_FRAME_ID     = 0x02   # uint32  扩展帧 id (0~0xFFFFFFE)
    CAN_TYPE         = 0x06   # uint8   1=CAN2.0, 2=FDCAN
    BRS_ENABLE       = 0x07   # uint8   1=启用波特率切换（默认 0）
    ARB_BAUD         = 0x08   # uint8   仲裁域波特率 1=1000k 2=800k 3=500k … 8=100k
    DATA_BAUD        = 0x09   # uint8   数据域波特率 1=8000k 2=5000k(默认) … 13=100k
    SILENT           = 0x0A   # uint8   1=屏蔽返回帧
    MODBUS_ADDR      = 0x0B   # uint8   [不支持] modbus 从机地址 1~247
    MODBUS_BAUD      = 0x0C   # uint8   [不支持] modbus 串口波特率枚举
    MODBUS_MODE      = 0x0D   # uint8   [不支持] 1=rtu, 2=ascii
    MODBUS_PARITY    = 0x0E   # uint8   [不支持] 1=none 2=even 3=odd
    MODBUS_STOP_BITS = 0x0F   # uint8   [不支持] 1=1 2=1.5 3=2
    ECAT_ALIAS       = 0x10   # uint16  [不支持] EtherCAT 别名地址
    ECAT_POSITION    = 0x12   # uint16  [不支持] EtherCAT 物理位置
    ECAT_DC_ENABLE   = 0x14   # uint8   [不支持] 启用 DC 分布式时钟
    ECAT_WD_ENABLE   = 0x15   # uint8   [不支持] 启用 EtherCAT watchdog
    ECAT_WD_TIMEOUT  = 0x16   # uint16  [不支持] watchdog 超时时间
    CTRL_MODE        = 0x18   # uint8   [不支持] 控制模式，见 CtrlMode（默认 1 位置模式）
    LED_ENABLE       = 0x19   # uint8   [不支持] 指示灯开关（默认 1）
    LED_FAULT_ONLY   = 0x1A   # uint8   [不支持] 指示灯只指示故障
    BUZZER_ENABLE    = 0x1B   # uint8   [不支持] 蜂鸣器开关（默认 0）
    RESTORE_CONFIG   = 0x1C   # uint8   写 1 恢复配置数据为默认值
    RESTORE_ALL      = 0x1D   # uint8   写 1 恢复所有数据（须先写工程服务密码）
    SAVE_TO_FLASH    = 0x1E   # uint8   写 1 保存参数到 Flash
    SAVE_MOTOR_PARAM = 0x1F   # uint8   [不支持] 保存电机参数到电机非易失存储


class EngineeringSI:
    """调试功能A / 工程服务子索引 (MI=0x42)。标 [不支持] 者本机固件未实现"""
    PASSWORD        = 0x00   # str8       写入配置密码(6 位, 默认 123456)
    AUTO_ZERO       = 0x08   # uint8[5]   [不支持] 按位开启对应关节自动找零点
    CLEAR_MAX_LIMIT = 0x0D   # uint8      [不支持] 写 1 清除/放宽最大限位
    SET_MAX_LIMIT   = 0x0E   # uint8[5]   [不支持] 按位设当前位置为最大限位
    CLEAR_MIN_LIMIT = 0x13   # uint8      [不支持] 写 1 清除/放宽最小限位
    SET_MIN_LIMIT   = 0x14   # uint8[5]   [不支持] 按位设当前位置为最小限位
    SET_MID_POINT   = 0x19   # uint8[5]   [不支持] 按位设当前位置为行程中点
    CALIBRATED      = 0x1E   # uint8[5]   按位表示相应关节是否已校准
    CALIB_COUNT     = 0x23   # uint16     [不支持] 校准次数
    CALIB_DATE      = 0x25   # str24      [不支持] 校准日期 YYYY-MM-DD hh:mm:ss
    IF_FAULT_FLAGS  = 0x3D   # uint8[8]   [不支持] 所有通信接口故障标志
    HEARTBEAT       = 0x45   # uint32     [不支持] 连接指示心跳(下位机每秒 +1)
    HAND_SIDE       = 0x49   # uint8      0xF0=左手, 0x0F=右手（出厂设定）
    SENSOR_TYPE     = 0x4A   # enum uint8 [不支持] 传感器类型，见 SENSOR_TYPE_ENUM
    POWER_ON_TIME   = 0x4B   # str16      [不支持] 本次上电运行时间 "00:00:22:25"
    TOTAL_RUN_TIME  = 0x5B   # str16      [不支持] 总运行时间
    MOTOR_RUN_TIME  = 0x6B   # str16      [不支持] 电机运行时长
    TASK_FREQ       = 0x7B   # uint16[24] [不支持] 任务执行频率(48 字节)
    DEVICE_UID      = 0xAB   # str32      设备唯一标识码
    UPDATE_APP      = 0xCB   # uint8      写 1 跳转 Bootloader 更新 app
    UPDATE_MOTOR_FW = 0xCC   # uint8      [不支持] 写 1 切换到更新电机固件模式
    PROTOCOL_SWITCH = 0xCD   # uint8      接口协议切换（新旧协议过渡期用）


class CtrlMode:
    """控制模式 (ConfigSI.CTRL_MODE 取值)，默认 0x01。
    ⚠️ xlsx 标注控制模式子索引本机固件「不支持该功能」，写入可能被忽略。"""
    POSITION      = 0x01   # 位置模式
    VELOCITY      = 0x02   # 速度模式
    CURRENT       = 0x03   # 电流模式
    TORQUE        = 0x04   # 转矩模式
    OPEN_LOOP     = 0x05   # 开环模式
    COMPLIANT     = 0x06   # 柔顺控制模式
    TEACH         = 0x07   # 教学习模式
    DRAG          = 0x08   # 拖动模式


class ErrorCode:
    """通信错误码 (MI=0x4F)，按位错误标志可组合（xlsx「通信协议错误码」）"""
    OK              = 0x00
    MI_NOT_EXIST    = 0x01   # bit0 主索引不存在
    SI_NOT_EXIST    = 0x02   # bit1 子索引不存在
    NOT_WRITABLE    = 0x04   # bit2 寄存器不可写
    LEN_MISMATCH    = 0x08   # bit3 数据长度不匹配
    NO_PERMISSION   = 0x10   # bit4 权限不足（未输入密码）
    DATA_PADDED     = 0x20   # bit5 返回数据存在补零


ERROR_CODE_NAMES = {
    ErrorCode.MI_NOT_EXIST:  "主索引不存在",
    ErrorCode.SI_NOT_EXIST:  "子索引不存在",
    ErrorCode.NOT_WRITABLE:  "寄存器不可写",
    ErrorCode.LEN_MISMATCH:  "数据长度不匹配",
    ErrorCode.NO_PERMISSION: "权限不足(未输入密码)",
    ErrorCode.DATA_PADDED:   "返回数据存在补零",
}
ERROR_LATEST_INDEX_SI = 0x00   # MI 0x4F SI0x00：最新一次错误索引 (1~15)
ERROR_HISTORY_COUNT   = 15     # SI 0x01~0x0F：15 条历史错误码


class SensorFinger:
    """选择传感器取值 (MI 0x31 SI 0x6E) —— 手指编号 1~5，手掌 6"""
    THUMB  = 1   # 大拇指
    INDEX  = 2   # 食指
    MIDDLE = 3   # 中指
    RING   = 4   # 无名指
    PINKY  = 5   # 小拇指
    PALM   = 6   # 手掌（本机通常未装，probe_sensors 会自动跳过）


SENSOR_FINGER_NAMES = {
    SensorFinger.THUMB:  "thumb",
    SensorFinger.INDEX:  "index",
    SensorFinger.MIDDLE: "middle",
    SensorFinger.RING:   "ring",
    SensorFinger.PINKY:  "pinky",
    SensorFinger.PALM:   "palm",
}
# probe_sensors 默认探测的传感器编号（按协议“选择传感器”取值域顺序）
SENSOR_PROBE_ORDER = (SensorFinger.THUMB, SensorFinger.INDEX, SensorFinger.MIDDLE,
                      SensorFinger.RING, SensorFinger.PINKY, SensorFinger.PALM)


class SensorSI:
    """传感器命令/元信息子索引 (MI=0x31) —— 严格按 xlsx「传感器」表 v0.0.4。
    ⚠️ 与旧固件(0.0.2)相比 数据总长度 0x2B→0x6C、选择传感器 0x2D→0x6E。

    ⚠️ 数据分布描述在 **0x2B**：该子索引读回的直接就是描述字符串本身
       （ASCII，形如 ``FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40``），
       描述 0x00 数据区的排布——合力值就在数据区开头，没有独立寄存器。
       xlsx 表格行把 0x2B 记为「描述长度」、0x2C 记为「描述内容」，与实测不符；
       为兼容这类固件保留 DIST_DESC_ALT(0x2C)，仅当 0x2B 读回不像描述时才用。
    """
    TYPE          = (0x00, 32)   # str32 传感器类型   ro
    UNIT          = (0x20, 8)    # str8  数据单位     ro
    RANGE         = 0x28         # uint8 传感器量程 0~255            ro
    MAX_ROWS      = 0x29         # uint8 数据最大行(点阵类有效)      ro
    MAX_COLS      = 0x2A         # uint8 数据最大列(点阵类有效)      ro
    DIST_DESC     = (0x2B, 64)   # str   数据分布描述(ASCII)         ro
    DIST_DESC_ALT = (0x2C, 64)   # str64 数据分布描述(旧表述: 0x2B 为长度) ro
    TOTAL_LENGTH  = (0x6C, 2)    # uint16 数据总长度（小端）         ro
    SELECT        = 0x6E         # uint8 选择传感器（见 SensorFinger）rw
    DATA_ROWS     = 0x6F         # uint8 数据行                      rw
    DATA_COLS     = 0x70         # uint8 数据列                      rw
    FLAG_MODE     = 0x71         # uint8 切换到传感器有效数据标识模式(0/1, 默认0) rw
                                 #   置 1 时数据区固定为 有效=1/无效=0，用于确认
                                 #   异形传感器补零后的位置关系
    PAD_RECT      = 0x72         # uint8 是否填充无效数据把外轮廓补成矩形(0/1, 默认1) rw
                                 #   置 0 时数据区是一维有效数据，需自行按分布描述还原


# 传感器数据通道：0x32/0x33/0x34 各覆盖 255 字节（SI 0x00~0xFE）
SENSOR_CHANNEL_MIS  = (MI.SENSOR_DATA1, MI.SENSOR_DATA2, MI.SENSOR_DATA3)
SENSOR_CHANNEL_SPAN = 255
MAX_LD = 61            # CAN FD 单帧有效载荷上限 = 64 - 3 字节帧头


# ---------------------------------------------------------------------------
# 数据分布描述（MI 0x31 SI 0x2B，读回即 ASCII 描述串本身）
# ---------------------------------------------------------------------------
# 传感器数据区（0x32/0x33/0x34）不是纯点阵！其排布由「数据分布描述」字符串定义，
# 字段以 ';' 分隔，每段格式 <标签>_<类型>_<个数>，按先后顺序连续存放。
# O6 协议原文示例（O6 与 O30 传感器规则完全一致，见 O6_HOP协议帧格式0.0.4.xlsx
# 「传感器」表 r30）：
#     FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40
#   = 1×uint16 法向力合力 + 1×uint16 切向力合力 + 1×uint16 切向力合力方向
#     + 1×uint16 保留 + 40×uint8 点阵值        （合计 8 字节头 + 40 字节点阵）
# 即 **合力值就在数据区开头**，与点阵一次读回，无需额外指令。
# ---------------------------------------------------------------------------
SENSOR_FIELD_TYPE_SIZE = {
    "U8": 1, "I8": 1, "S8": 1,
    "U16": 2, "I16": 2, "S16": 2,
    "U32": 4, "I32": 4, "S32": 4, "F32": 4,
}
SENSOR_FIELD_SIGNED = ("I8", "S8", "I16", "S16", "I32", "S32")
SENSOR_FIELD_NAMES = {
    "FnS": "法向力合力",
    "FtS": "切向力合力",
    "FdS": "切向力合力方向",
    "Fn":  "法向力点阵",
    "Ft":  "切向力点阵",
    "Fd":  "方向点阵",
    "Rsv": "保留",
}
# 合力类标量字段（个数为 1，位于数据区头部）
SENSOR_FORCE_TAGS = ("FnS", "FtS", "FdS")
# 点阵类字段标签（个数 = 行×列）
SENSOR_MATRIX_TAGS = ("Fn", "Ft", "Fd")


class SensorType:
    """传感器类型字符串枚举 (MI 0x31 SI 0x00 返回的 str32)"""
    NO_SENSOR     = "NO_SENSOR"
    TASHAN_GATHER = "TASHAN_GATHER"   # 他山(外部采集板)
    HUAWEIKE      = "HUAWEIKE"        # 华威科
    FULAI         = "FULAI"           # 福莱
    TSSP_GENGLE   = "TSSP_GENGLE"     # TSSP 福莱适配版
    TSSP_JZG      = "TSSP_JZG"        # TSSP 晶致感


# 工程服务 0x42/0x4A 传感器设置 uint8 枚举 → 名称（xlsx「调试功能A」0x4A）
SENSOR_TYPE_ENUM = {
    0x00: "无传感器",
    0x01: "TS传感器(采集板)",
    0x02: "HWK传感器",
    0x03: "FL传感器",
    0x04: "TSSP_GENGLE传感器",
    0x05: "TSSP_JZG传感器",
}


class Gesture:
    """预设手势/动作类型 (MI 0x36~0x3A SI 0x00)。
    ⚠️ xlsx「预设动作及动作编排」全部子索引标注本机固件「不支持该功能」，
       写入通常无动作，仅保留接口备用。"""
    NONE         = 0x00   # 无效手势，无动作执行
    NUMBER_1     = 0x01   # 0x01~0x0A 为数字手势 1~10
    NUMBER_10    = 0x0A
    HALF_GRIP    = 0x0B   # 半握
    CIRCLE       = 0x0C   # 手指画圆
    ELLIPSE      = 0x0D   # 手指画椭圆
    FINGER_DANCE = 0x0E   # 手指舞
    ROCK         = 0x0F   # 猜拳-石头
    SCISSORS     = 0x10   # 猜拳-剪刀
    PAPER        = 0x11   # 猜拳-布
    GRASP        = 0x12   # 抓握
    EXTEND       = 0x13   # 伸展
    PACK         = 0x14   # 打包手势


class ActionSI:
    """预设动作子索引 (MI 0x36~0x3A，各 7 字节)"""
    TYPE       = 0x00   # uint8 动作类型（见 Gesture）
    SPEED      = 0x01   # uint8 动作执行速度 0~255
    AMPLITUDE  = 0x02   # uint8 动作幅值 0~255
    LOOPS      = 0x03   # uint8 动作循环次数 0~255
    FINGER     = 0x04   # uint8 手指指定 1~5
    RUN_FLAG   = 0x05   # uint8 动作执行标志 0=执行完毕 1=正在执行
    POWERUP_EN = 0x06   # uint8 是否上电默认执行 0/1


# 稀疏关节位置控制 (MI 0x30)：每组 2 字节(关节序号 + 目标位置)，
# xlsx 标注本机固件仅前 19 组「支持该功能」，第 20~36 组为否。
SPARSE_MAX_PAIRS = 19


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
                    continue      # 有帧就立刻再取，不睡——连续分段读时省掉整段延迟
            except Exception:
                pass
            # 空闲让出 CPU。⚠️ 这个值直接决定同步请求的应答延迟：应答若落在睡眠
            # 期间，_request 就要多等这么久。原为 2ms，高频触觉(五指 30Hz 需要
            # 13 个带应答事务/周期)时平均每次读凭空多 1ms，是当时的主要瓶颈。
            # 降到 0.2ms：最坏额外延迟 0.2ms，代价是空闲时每秒多几千次
            # CANFD_Receive 调用（该调用自带 10ms 等待，实测 CPU 占用可忽略）。
            time.sleep(RX_IDLE_POLL_SLEEP)

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
    继承 CANFDCommunication，仅重写底层传输方法；帧格式与厂商设备完全一致。
    """

    def __init__(self, channel: str = "can0", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True):
        self.channel = channel      # 接口名，如 "can0"
        self.bitrate = bitrate      # 仲裁段波特率，默认 1Mbps
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
            # 激活CAN/CANFD设备
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
    """Linker Hand O30 灵巧手 CANFD 控制类
    （HOP 协议，依据 O30-HOP协议-20260907.xlsx / 文档版本 v0.0.4）"""

    DLC2LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

    def __init__(self, hand_type: str = "right", canfd_device: int = 0,
                 frame_id: Optional[int] = None, comm_type: str = "libcanbus",
                 channel: str = "can0", bitrate: int = 1000000,
                 dbitrate: int = 5000000, auto_setup: bool = True,
                 probe_sensor: bool = True):
        """
        hand_type   : "left"/"right"，仅作标签；HOP 协议左右手实际靠帧 ID 或读
                      产品信息(0x41/0xF1)/工程服务(0x42/0x49)区分，不内嵌于 CAN ID。
        frame_id    : 设备标准帧 ID(11 位)。不指定时按惯例 右手=0x01 / 左手=0x02
                      （同总线双手需各自配置不同 ID）。
        comm_type   : 通信后端 —— "libcanbus"(默认, 厂商私有库) 或
                      "socketcan"(内核原生 can0 + python-can, 透明塑封 USB-CANFD 设备)。
        canfd_device: 仅 libcanbus 用——设备序号。
        channel     : libcanbus 下为通道号(默认 0)；socketcan 下为接口名(默认 "can0")。
        bitrate/dbitrate/auto_setup : 仅 socketcan 用——仲裁/数据段波特率与是否自动拉起接口。
        probe_sensor: True(默认) 时在 initialize() 里一次性探测并缓存各传感器的
                      数据总长度/行列/单位/量程及分段读取计划(见 probe_sensors)；
                      之后高频读取只需「选择传感器 + 按缓存长度读数据通道」，
                      不再重复发元信息帧。
        """
        self.hand_type = hand_type
        if frame_id is None:
            frame_id = (DeviceID.LEFT_HAND.value if hand_type == "left"
                        else DeviceID.RIGHT_HAND.value)
        self.device_id = frame_id
        self.canfd_device = canfd_device
        self.comm_type = comm_type
        self._probe_sensor = probe_sensor

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

        # ---- 传感器缓存（高频读取的核心：元信息只在初始化时读一次） ----
        # sensors[finger] = {name, type, unit, range, rows, cols, total_length,
        #                    dist_desc, layout, force_fields, bytes_per_cell, plan}
        # plan = [(mi, si, n), ...] 预先算好的分段读取计划，读一帧数据零计算开销。
        # 只有初始化探测确认「可用」的传感器才会进 sensors；不可用的编号记进
        # sensor_unavailable(原因)，后续任何读取会被本地直接拒绝，不上总线。
        self.sensors: Dict[int, Dict] = {}
        self.sensor_lengths: Dict[int, int] = {}   # {finger: 数据总长度}
        self.available_sensors: List[int] = []     # 可用传感器编号（升序）
        self.sensor_names: Dict[int, str] = {}     # {finger: 名称} 仅含可用
        self.sensor_unavailable: Dict[int, str] = {}   # {finger: 不可用原因}
        self.has_finger_sensors: bool = False      # 五指是否有传感器
        self.has_palm_sensor: bool = False         # 手掌是否有传感器（本机通常无）
        self._sensor_probed: bool = False          # 是否已完成可用性探测
        self._probing_sensors: bool = False        # 探测进行中（暂不做可用性拦截）
        self._selected_sensor: Optional[int] = None  # 当前已选传感器(免重复选择)

        # 响应匹配状态保护锁
        self._lock = threading.Lock()

        # 事务串行锁：一次只允许一个请求在途，发完等到回复/超时才放行下一帧，
        # 防止上一帧响应未到就发新帧导致响应被覆盖/错认。
        self._txn_lock = threading.Lock()

        # 同步请求/应答：按当前在途请求的回显 **MI + SI** 关联响应。
        # ⚠️ 只比 MI 是不够的：同一 MI 下（尤其 MI 0x31 传感器命令）有几十个子索引，
        #    上一次读的迟到响应或写(如选择传感器 0x6E)的回显 MI 完全相同，会被当成
        #    本次读的应答 → 读到别的字段的内容（曾表现为「数据总长度=21332」，
        #    21332=0x5354 正是类型串 "TS" 的两个字节）。故必须同时校验 SI。
        self._pending_event: Optional[threading.Event] = None
        self._pending_resp: Optional[bytes] = None
        self._pending_mi: Optional[int] = None
        self._pending_si: Optional[int] = None
        self._si_mismatch: Optional[Tuple[int, int]] = None   # 最近一次被丢弃的串扰帧
        self._si_warned: bool = False                         # 串扰提示只打一次
        self.strict_si_match: bool = True   # 固件若不回显 SI 可置 False 退回只比 MI
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
        """初始化通信层（委托给 self.comm）并完成上电准备。

        上电准备包含传感器可用性探测：probe_sensor=True 时逐个候选编号判定传感器
        是否真的存在（类型/数据总长度/数据通道），可用的把「数据总长度/行/列/单位/
        量程/分布描述/读取计划」缓存进 sensors，并把编号列表赋给 available_sensors；
        不可用的（如本机没有手掌传感器）记进 sensor_unavailable，之后任何触觉读取
        都会被本地直接拒绝，不会为不存在的传感器发帧等超时。
        """
        if not self.comm.initialize():
            return False

        # 有效关节按 ACTIVE_JOINTS 静态固定（0D 00 24 自动探测在本机报错，已停用）。
        print(f"✅ 有效关节 {self.num_joints} 个")
        if self._probe_sensor:
            self.probe_sensors()
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
            # 仅当回显 MI **且 SI** 与在途请求匹配（或为 0x4F 错误帧）时才认作本次
            # 响应，否则丢弃并继续等——防止写回显/同 MI 其它子索引的陈旧帧被错认。
            resp_mi = payload[0] & 0x7F
            resp_si = payload[1] if len(payload) > 1 else -1
            if resp_mi != MI.ERROR_CODE:
                if resp_mi != self._pending_mi:
                    return
                if self.strict_si_match and self._pending_si is not None \
                        and resp_si != self._pending_si:
                    self._si_mismatch = (resp_mi, resp_si)   # 串扰帧：记下、丢弃
                    return
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
        超时 / 帧头(MI 或 SI)不符 / 设备回错误码帧，均返回 None。

        事务串行：整个「发送→等回复→取走」过程持 _txn_lock，确保同一时刻只有
        一个请求在途，新请求会等上一请求拿到回复或超时后才发出，避免响应被覆盖。
        帧头校验同时比对 **MI 与 SI**：宁可本次超时返回 None，也不能把别的子索引
        的响应当成本次结果——后者会静默产生错位数据（见 _process_response 注释）。
        """
        with self._txn_lock:
            ev = threading.Event()
            with self._lock:
                self._pending_event = ev
                self._pending_resp = None
                self._pending_mi = mi & 0x7F
                self._pending_si = si & 0xFF
                self._si_mismatch = None

            self._send_frame(self._tx_id, self._frame(mi, si, length, b'', rts))
            got = ev.wait(timeout)

            with self._lock:
                body = self._pending_resp
                mismatch = self._si_mismatch
                self._pending_event = None
                self._pending_resp = None
                self._pending_mi = None
                self._pending_si = None

        if not got or body is None or len(body) < 3:
            if mismatch and not self._si_warned:
                self._si_warned = True
                print(f"⚠️ 收到 MI 匹配但 SI 不符的帧(MI=0x{mismatch[0]:02X} "
                      f"SI=0x{mismatch[1]:02X}，本次请求 MI=0x{mi:02X} SI=0x{si:02X})，"
                      f"已按串扰丢弃。若固件不回显 SI，请设 strict_si_match=False")
            return None
        # 响应回显 3 字节帧头(MI/SI/EDL)；MI/SI 不符 → 错误帧或串扰，丢弃
        resp_mi = body[0] & 0x7F
        if resp_mi != (mi & 0x7F):
            if resp_mi == MI.ERROR_CODE and len(body) >= 4:
                self.last_error_code = body[3]
                print(f"⚠️ 设备返回错误码 0x{body[3]:02X}（请求 MI=0x{mi:02X} SI=0x{si:02X}）")
            return None
        if self.strict_si_match and body[1] != (si & 0xFF):
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
    # ⚠️ 按 xlsx「协议格式」修正主索引：0x0C=关节故障(JointFault)，
    #    0x0D=有效关节(JointBit)。旧代码两者互换，此处以协议为准。
    # ================================================================== #
    def enable_all_joints(self) -> bool:
        """使能全部有效关节（MI 0x0D 有效关节写 0x0F=存在+可控+已使能+反馈）。
        上电后写位置前必须执行。空洞子索引(本机无电机)写 0，不会被误标为有效。"""
        return self._write(MI.JOINT_ENABLE, 0x00,
                           self._pack_u8([JointBit.ENABLE_DEFAULT] * self.num_joints))

    def set_joint_enable(self, masks: List[int]) -> bool:
        """逐关节设置有效/使能位掩码（MI 0x0D，见 JointBit）"""
        if len(masks) != self.num_joints:
            return False
        return self._write(MI.JOINT_ENABLE, 0x00, self._pack_u8(masks))

    def get_joint_enable(self) -> Optional[List[int]]:
        """读取有效关节的能力/使能位掩码（MI 0x0D，见 JointBit），按有效关节顺序返回。"""
        body = self._request(MI.JOINT_ENABLE, 0x00, self.phys_span)
        return self._unpack_u8(body) if body else None

    def get_joint_fault(self) -> Optional[List[int]]:
        """读取各有效关节的故障位掩码（MI 0x0C 关节故障，见 JointFault / 关节故障码附表）。
        返回按有效关节顺序的位掩码列表，0 表示无故障；逐位中文名见 decode_joint_fault。"""
        body = self._request(MI.JOINT_FAULT, 0x00, self.phys_span)
        return self._unpack_u8(body) if body else None

    @staticmethod
    def decode_joint_fault(mask: int) -> List[str]:
        """把 MI 0x0C 的一个关节故障位掩码翻译成故障名称列表（无故障返回空表）。"""
        return [name for bit, name in JOINT_FAULT_NAMES.items() if mask & bit]

    def get_joint_faults(self) -> Optional[Dict[str, List[str]]]:
        """读取并翻译全部有效关节的故障，返回 {关节名: [故障名, ...]}，仅含有故障者。"""
        masks = self.get_joint_fault()
        if masks is None:
            return None
        return {name: self.decode_joint_fault(m)
                for name, m in zip(self.joint_names, masks) if m}

    def print_joint_faults(self):
        """打印各关节故障（MI 0x0C）。"""
        faults = self.get_joint_faults()
        if faults is None:
            print("❌ 无法读取关节故障（0C 00 xx 超时/失败）")
            return
        if not faults:
            print("✅ 全部有效关节无故障")
            return
        print("=" * 50)
        print("关节故障:")
        for name, items in faults.items():
            print(f"  {name:15s}: {'、'.join(items)}")
        print("=" * 50)

    def get_valid_joints(self) -> Optional[Dict[int, Dict]]:
        """探测本机有效关节（发 0D 00 24：MI 0x0D 有效关节，子索引 0x00~0x23 共 36 字节）。

        上电流程第一步——返回每字节为该逻辑关节的存在/能力位掩码(见 JointBit)：
        非 0 即有效(实测 0x0B=存在+可控+反馈)，0 为无效(无对应电机)。结果按 si
        与 JOINT_MAP 比对，返回 {si: {name, description, mask}} 仅含有效关节。
        读取失败返回 None。可据此增删 JOINT_MAP 的条目(换型号时)。
        """
        body = self._request(MI.JOINT_ENABLE, 0x00, LOGICAL_JOINT_COUNT)
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
        ⚠️ xlsx「稀疏关节位置控制」标注本机固件仅支持前 19 组(SPARSE_MAX_PAIRS)，
           超出部分会被截掉并告警；20 组以上为协议保留、本固件为「否」。
        """
        if len(pairs) > SPARSE_MAX_PAIRS:
            print(f"⚠️ 稀疏位置最多 {SPARSE_MAX_PAIRS} 组，收到 {len(pairs)} 组，超出部分已截掉")
            pairs = pairs[:SPARSE_MAX_PAIRS]
        data = bytearray()
        for jn, pos in pairs:
            data += bytes([jn & 0xFF, max(0, min(U8_MAX, int(pos)))])
        return self._write(MI.SPARSE_POS, 0x00, bytes(data))

    # ---- 增量型位置控制（MI 0x0E 增 / 0x0F 减，单字节向量，与 0x01 同一索引空间） ---- #
    def set_position_increment(self, deltas: List[int]) -> bool:
        """各有效关节目标位置「增」指定增量（MI 0x0E，单字节 0~255）。
        入参个数须等于 self.num_joints；0 表示该关节不动。"""
        if len(deltas) != self.num_joints:
            print(f"❌ 需要 {self.num_joints} 个增量，收到 {len(deltas)}")
            return False
        return self._write(MI.INC_POSITION, 0x00, self._pack_u8(deltas))

    def set_position_decrement(self, deltas: List[int]) -> bool:
        """各有效关节目标位置「减」指定增量（MI 0x0F，单字节 0~255）。
        入参个数须等于 self.num_joints；0 表示该关节不动。"""
        if len(deltas) != self.num_joints:
            print(f"❌ 需要 {self.num_joints} 个增量，收到 {len(deltas)}")
            return False
        return self._write(MI.DEC_POSITION, 0x00, self._pack_u8(deltas))

    def step_joint_position(self, joint: str, delta: int) -> bool:
        """按名称给单个关节做增量位置控制（delta>0 走 MI 0x0E，delta<0 走 0x0F）。
        只发 1 帧 1 字节，适合手柄/摇杆类高频微调。"""
        if joint not in self.joint_si:
            print(f"❌ 未知关节 {joint}，可选 {self.joint_names}")
            return False
        step = max(-U8_MAX, min(U8_MAX, int(delta)))
        if step == 0:
            return True
        mi = MI.INC_POSITION if step > 0 else MI.DEC_POSITION
        return self._write(mi, self.joint_si[joint], bytes([abs(step)]))

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
        print(f"设置目标速度: {velocities}")
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
        报文 42 4A 01。未知值返回 '未知(0xNN)'，读取失败返回 None。
        ⚠️ xlsx v0.0.4 未列出该子索引，本机固件可能不支持；传感器真实类型请用
           get_sensor_info()['type']（MI 0x31 SI 0x00 字符串）。"""
        b = self._request(MI.ENGINEERING, EngineeringSI.SENSOR_TYPE, 1)
        if not b:
            return None
        return SENSOR_TYPE_ENUM.get(b[0], f"未知(0x{b[0]:02X})")

    # ================================================================== #
    # 错误码（MI=0x4F，只读）
    # ------------------------------------------------------------------ #
    # SI 0x00 = 最新一次错误所在的历史槽号(1~15)；SI 0x01~0x0F = 15 条历史错误码。
    # 每条错误码是 bit0~bit5 的可组合标志（见 ErrorCode / ERROR_CODE_NAMES）。
    # ================================================================== #
    def get_last_error_index(self) -> Optional[int]:
        """读最新一次错误所在的历史槽号（1~15；0 表示尚无错误记录）"""
        idx = self._request(MI.ERROR_CODE, ERROR_LATEST_INDEX_SI, 1)
        return idx[0] if idx else None

    def get_error_at(self, index: int) -> Optional[int]:
        """读第 index 条历史错误码（index 取 1~15）"""
        if not 1 <= index <= ERROR_HISTORY_COUNT:
            return None
        b = self._request(MI.ERROR_CODE, index, 1)
        return b[0] if b else None

    def get_last_error(self) -> Optional[int]:
        """读最新一次通信错误码（0x00=正常，见 ErrorCode）。
        先取最新槽号(SI 0x00)，再按槽号读该条错误码；槽号为 0 时返回 0x00。"""
        idx = self.get_last_error_index()
        if idx is None:
            return None
        if idx == 0:
            return ErrorCode.OK
        return self.get_error_at(idx)

    def get_error_history(self) -> Optional[List[int]]:
        """读全部 15 条历史错误码（SI 0x01~0x0F），返回按槽号 1~15 排列的列表"""
        out = []
        for i in range(1, ERROR_HISTORY_COUNT + 1):
            b = self._request(MI.ERROR_CODE, i, 1)
            if b is None:
                return None
            out.append(b[0])
        return out

    @staticmethod
    def decode_error_code(code: int) -> List[str]:
        """把通信错误码按位解成中文描述列表（0x00 返回 ['正常']）"""
        if code == ErrorCode.OK:
            return ["正常"]
        return [name for bit, name in ERROR_CODE_NAMES.items() if code & bit] or \
               [f"未知(0x{code:02X})"]

    def print_error_history(self):
        """打印最新错误槽号与 15 条历史错误码解析结果"""
        idx = self.get_last_error_index()
        hist = self.get_error_history()
        if hist is None:
            print("❌ 错误码读取失败")
            return
        print("=" * 50)
        print(f"通信错误码（最新槽号={idx}）:")
        for i, code in enumerate(hist, start=1):
            mark = " ←最新" if i == idx else ""
            print(f"  [{i:2d}] 0x{code:02X}  {'、'.join(self.decode_error_code(code))}{mark}")
        print("=" * 50)

    # ================================================================== #
    # 触觉传感器（MI 0x31 命令/元信息 + 0x32/0x33/0x34 数据通道）
    # ------------------------------------------------------------------ #
    # 初始化必须走的流程（每个传感器一遍，全部结果缓存）：
    #   ① 读数据分布描述(0x2B)  ② 选择传感器(0x6E)  ③ 读最大行/列(0x29/0x2A)
    #   ④ 把最大行/列写到数据行/列(0x6F/0x70)      ⑤ 读数据总长度(0x6C)
    # 实现上把「选择」提到最前（②→①）：0x2B/0x29/0x2A/0x6C 读的都是**当前所选**
    # 传感器的属性，不先选就会读到上一个传感器的值。其余次序与上表一致。
    #
    # 运行期（高频路径）只剩两步，不再发任何元信息帧：
    #   选择传感器(0x6E，已选中则连这帧也省) → 按缓存的数据总长度读 SI 0x00 起的数据
    # 每帧最多取 MAX_LD=61 字节(CAN FD 64 - 3 帧头)，每通道覆盖 255 字节，
    # 分段方案在初始化时已预生成为 info["plan"]，运行期零计算。
    #
    # ⚠️ 数据区不是纯点阵：按 0x2B 的分布描述，开头通常是 法向力合力/切向力合力/
    #    合力方向 等 uint16 标量，点阵在其后。合力值没有独立寄存器，与点阵一次读回，
    #    解析见 parse_tactile()。示例描述：
    #      FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40
    #      = 1×uint16 法向力合力 + 1×uint16 切向力合力 + 1×uint16 合力方向
    #        + 1×uint16 保留 + 40 个 uint8 点阵值（共 48 字节）
    # ================================================================== #
    @staticmethod
    def _decode_sensor_desc(raw: Optional[bytes]) -> str:
        """把 0x2B 读回的字节还原成分布描述字符串。

        设备按固定长度回填，尾部可能是 0x00 或随机填充；只取**开头连续的可打印
        ASCII**，遇到 NUL/非打印字符即截断，避免把填充字节当成字段名。
        """
        if not raw:
            return ''
        out = []
        for b in raw:
            if 0x20 <= b < 0x7F:
                out.append(chr(b))
            else:
                break
        return ''.join(out).strip()

    @staticmethod
    def _looks_like_sensor_desc(desc: str) -> bool:
        """粗判一段字符串是否是分布描述（形如 Tag_TYPE_N，可用 ';' 拼接）。"""
        if not desc or '_' not in desc:
            return False
        for seg in desc.split(';'):
            parts = seg.strip().rsplit('_', 2)
            if len(parts) == 3 and parts[1].upper() in SENSOR_FIELD_TYPE_SIZE \
                    and parts[2].isdigit():
                return True
        return False

    @staticmethod
    def _parse_sensor_layout(desc: str) -> List[Dict]:
        """解析「数据分布描述」为字段列表（顺序即数据区排布顺序）。

        输入形如 ``FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40``，
        每段 ``<标签>_<类型>_<个数>``。返回:
        ``[{tag,name,type,signed,elem_size,count,offset,size}, ...]``
        无法解析的段跳过并告警；描述为空时返回 []（调用方回退到纯点阵假设）。
        """
        fields: List[Dict] = []
        off = 0
        for seg in (desc or '').split(';'):
            seg = seg.strip()
            if not seg:
                continue
            parts = seg.rsplit('_', 2)          # 标签本身可能含下划线，从右侧切
            if len(parts) != 3:
                print(f"⚠️ 传感器分布描述字段无法解析: {seg!r}")
                continue
            tag, typ, cnt = parts[0], parts[1].upper(), parts[2]
            elem = SENSOR_FIELD_TYPE_SIZE.get(typ)
            if elem is None or not cnt.isdigit():
                print(f"⚠️ 传感器分布描述字段无法解析: {seg!r}")
                continue
            count = int(cnt)
            fields.append({
                "tag":       tag,
                "name":      SENSOR_FIELD_NAMES.get(tag, tag),
                "type":      typ,
                "signed":    typ in SENSOR_FIELD_SIGNED,
                "elem_size": elem,
                "count":     count,
                "offset":    off,
                "size":      elem * count,
            })
            off += elem * count
        return fields

    def probe_sensors(self, fingers: Tuple[int, ...] = SENSOR_PROBE_ORDER,
                      force: bool = False, apply_shape: bool = True,
                      verify_data: bool = True) -> Dict[int, Dict]:
        """探测传感器**可用性**，并缓存可用者的元信息、字段布局与读取计划。

        对每个候选编号(1拇指~5小指, 6手掌)按规定流程逐项判定，任一步不过即判为
        「不可用」并记入 self.sensor_unavailable，**不进 self.sensors**：
          ① 选择传感器(0x6E) 写失败                    → 不可用「选择失败」
          ② 传感器类型(0x00) 读不到 / 为空 / NO_SENSOR  → 不可用「未安装」
          ③ 数据总长度(0x6C) 为 0 或超出三通道上限且无法
             由分布描述推算                             → 不可用「数据总长度无效」
          ④ verify_data=True 时试读数据通道首帧失败      → 不可用「数据通道无响应」
        通过的按次序读分布描述(0x2B)/最大行列(0x29,0x2A)/单位/量程，把最大行列写到
        数据行列(0x6F/0x70，apply_shape=True 且与读回值不一致时才写)，再读数据总
        长度(0x6C)，解析字段布局并预生成分段读取计划。

        探测结果同时落到这些实例变量，运行期直接查、不再上总线：
          available_sensors / sensor_names / sensor_lengths / sensors
          sensor_unavailable / has_finger_sensors / has_palm_sensor
        探测完成后，读取不可用编号会被本地直接拒绝（见 select_sensor）。
        force=False 时若已探测过则直接返回缓存。返回 {finger: 传感器信息字典}。
        """
        if self._sensor_probed and not force:
            return self.sensors
        sensors: Dict[int, Dict] = {}
        unavailable: Dict[int, str] = {}
        self._probing_sensors = True          # 探测期间不做可用性拦截
        try:
            for finger in fingers:
                info, reason = self._collect_sensor_info(finger, apply_shape=apply_shape,
                                                         verify_data=verify_data)
                if info:
                    sensors[finger] = info
                else:
                    unavailable[finger] = reason or "不可用"
        finally:
            self._probing_sensors = False
        self._set_sensor_cache(sensors, unavailable)
        self._sensor_probed = True
        self._print_sensor_probe_result()
        return self.sensors

    def _set_sensor_cache(self, sensors: Dict[int, Dict], unavailable: Dict[int, str]):
        """把探测结果写入各缓存变量，并把当前选择落到一个真实存在的传感器上。"""
        self.sensors = sensors
        self.sensor_lengths = {f: v["total_length"] for f, v in sensors.items()}
        self.available_sensors = sorted(sensors)
        self.sensor_names = {f: v["name"] for f, v in sensors.items()}
        self.sensor_unavailable = unavailable
        self.has_finger_sensors = any(f != SensorFinger.PALM for f in sensors)
        self.has_palm_sensor = SensorFinger.PALM in sensors
        if self.available_sensors:
            # 探测过程可能停在「无传感器」的编号上，收尾落回第一个可用传感器
            self.select_sensor(self.available_sensors[0], force=True)
        else:
            self._selected_sensor = None

    def _print_sensor_probe_result(self):
        """打印可用/不可用传感器清单（初始化时给一眼看清的结论）。"""
        if self.sensors:
            desc = "  ".join(f"{v['name']}={v['total_length']}B"
                             f"({v['rows']}x{v['cols']})" for v in self.sensors.values())
            print(f"✅ 可用传感器 {len(self.sensors)} 个 {self.available_sensors}，"
                  f"长度已缓存: {desc}")
            forces = [v['name'] for v in self.sensors.values() if v["force_fields"]]
            if forces:
                print(f"   合力字段随数据区一并返回: {'、'.join(forces)}")
        else:
            print("ℹ️ 未探测到可用传感器，所有触觉读取接口将直接返回 None")
        if self.sensor_unavailable:
            miss = "  ".join(f"{SENSOR_FINGER_NAMES.get(f, f)}({r})"
                             for f, r in sorted(self.sensor_unavailable.items()))
            print(f"ℹ️ 不可用传感器 {len(self.sensor_unavailable)} 个: {miss}")

    def _collect_sensor_info(self, finger: int, apply_shape: bool = True,
                             verify_data: bool = True) -> Tuple[Optional[Dict], str]:
        """按规定流程读齐单个传感器的元信息并组装缓存条目。

        次序：选择(0x6E) → 类型(0x00，可用性判据) → 分布描述(0x2B) →
        最大行/列(0x29/0x2A) → 写数据行/列(0x6F/0x70) → 数据总长度(0x6C)
        → 解析布局 → 预生成分段读取计划 → (可选)试读首帧确认数据通道。

        返回 (info, reason)：可用时 info 为缓存条目、reason 为 ''；
        不可用时 info 为 None、reason 是中文原因（用于 sensor_unavailable）。
        """
        name = SENSOR_FINGER_NAMES.get(finger, f"传感器{finger}")
        # 选择传感器：必须最先做，之后 0x2B/0x29/0x2A/0x6C 读的才是这一个的属性
        if not self.select_sensor(finger, force=True):
            return None, "选择失败"
        # 传感器类型是最可靠的“装没装”判据：未装时为空或 NO_SENSOR
        typ = self._request(MI.SENSOR_CMD, *SensorSI.TYPE)
        if typ is None:
            return None, "类型无响应"
        typ_s = typ.decode('utf-8', 'ignore').rstrip('\x00').strip()
        if not typ_s or typ_s.upper().startswith(SensorType.NO_SENSOR):
            return None, "未安装"
        # 分布描述(0x2B) + 最大行/列(0x29/0x2A) + 单位/量程
        info = self._read_sensor_meta(sensor_type=typ_s)
        info["finger"] = finger
        info["name"] = name
        # 把最大行/列写到数据行/列(0x6F/0x70)——与读回值一致时不写，省帧
        if apply_shape and info["rows"] and info["cols"]:
            cur_r = self._request(MI.SENSOR_CMD, SensorSI.DATA_ROWS, 1)
            cur_c = self._request(MI.SENSOR_CMD, SensorSI.DATA_COLS, 1)
            if cur_r and cur_r[0] != info["rows"]:
                self._write(MI.SENSOR_CMD, SensorSI.DATA_ROWS, bytes([info["rows"]]))
            if cur_c and cur_c[0] != info["cols"]:
                self._write(MI.SENSOR_CMD, SensorSI.DATA_COLS, bytes([info["cols"]]))
        # 数据总长度(0x6C)——三通道最多覆盖 765 字节，超出即视为读到了脏数据
        limit = SENSOR_CHANNEL_SPAN * len(SENSOR_CHANNEL_MIS)
        expect = sum(f["size"] for f in self._parse_sensor_layout(info["dist_desc"]))
        total = self.get_sensor_total_length()
        if total and total > limit:
            print(f"⚠️ {name}数据总长度读回 {total} 字节不合理(三通道上限 {limit})，重读一次")
            again = self.get_sensor_total_length()
            total = again if (again and again <= limit) else None
        if not total:
            if 0 < expect <= limit:      # 描述可信：按各字段之和推算
                print(f"ℹ️ {name}数据总长度不可用，按分布描述 {info['dist_desc']} "
                      f"推算为 {expect} 字节")
                total = expect
            else:
                return None, "数据总长度无效"
        info["total_length"] = total
        self._apply_sensor_layout(info)
        info["plan"] = self._build_sensor_plan(total)
        if not info["plan"]:
            return None, "读取计划为空"
        # 试读数据通道首帧，确认数据区真的能读（部分固件只有 0x32 通道可用）
        if verify_data:
            mi, si, n = info["plan"][0]
            if self._request(mi, si, n) is None:
                return None, "数据通道无响应"
        return info, ""

    @staticmethod
    def _apply_sensor_layout(info: Dict):
        """由分布描述解析字段布局，填充 layout/force_fields/matrix_* /bytes_per_cell。

        分布描述缺失或解析不出点阵字段时，回退到「整段数据都是点阵」的旧假设：
        matrix_offset=0，每格字节数 = 总长 ÷ (行×列)。
        """
        rows, cols, total = info["rows"], info["cols"], info["total_length"]
        cells = rows * cols
        layout = LinkerHandO30Controller._parse_sensor_layout(info.get("dist_desc", ""))
        info["layout"] = layout
        info["layout_length"] = sum(f["size"] for f in layout)
        info["force_fields"] = [f for f in layout
                                if f["count"] == 1 and f["tag"] in SENSOR_FORCE_TAGS]
        # 点阵字段：优先取标签匹配的，否则取个数最大且 >1 的字段
        mat = next((f for f in layout if f["tag"] in SENSOR_MATRIX_TAGS and f["count"] > 1),
                   None)
        if mat is None:
            cand = [f for f in layout if f["count"] > 1]
            mat = max(cand, key=lambda f: f["count"]) if cand else None
        if mat is not None:
            info["matrix_field"]   = mat
            info["matrix_offset"]  = mat["offset"]
            info["matrix_count"]   = mat["count"]
            info["bytes_per_cell"] = mat["elem_size"]
            if cells and mat["count"] != cells:
                print(f"⚠️ {info['name']}分布描述点阵数 {mat['count']} 与行×列 "
                      f"{rows}x{cols}={cells} 不一致，按分布描述为准")
        else:
            info["matrix_field"]   = None
            info["matrix_offset"]  = 0
            info["matrix_count"]   = cells
            info["bytes_per_cell"] = max(1, total // cells) if cells else 1
        if layout and info["layout_length"] != total:
            print(f"⚠️ {info['name']}分布描述总长 {info['layout_length']} 字节与数据总长度 "
                  f"{total} 字节不一致（按各字段偏移解析，多余字节忽略）")

    def _read_sensor_meta(self, sensor_type: Optional[str] = None) -> Dict:
        """读当前所选传感器的静态元信息（不含总长度）。仅探测阶段使用。

        次序按规定流程：分布描述(0x2B) → 最大行(0x29) → 最大列(0x2A)，
        之后才补读单位(0x20)/量程(0x28)。sensor_type 已在可用性判定时读过时传进来，
        避免重复读 0x00 的 32 字节。
        """
        if sensor_type is None:
            typ = self._request(MI.SENSOR_CMD, *SensorSI.TYPE) or b''
            sensor_type = typ.decode('utf-8', 'ignore').rstrip('\x00').strip()
        desc   = self._read_sensor_dist_desc()
        rows   = self._request(MI.SENSOR_CMD, SensorSI.MAX_ROWS, 1) or b'\x00'
        cols   = self._request(MI.SENSOR_CMD, SensorSI.MAX_COLS, 1) or b'\x00'
        unit   = self._request(MI.SENSOR_CMD, *SensorSI.UNIT)      or b''
        srange = self._request(MI.SENSOR_CMD, SensorSI.RANGE, 1)    or b'\x00'
        return {
            "type":      sensor_type,
            "unit":      unit.decode('utf-8', 'ignore').rstrip('\x00').strip(),
            "range":     srange[0],
            "rows":      rows[0],
            "cols":      cols[0],
            "dist_desc": desc,
        }

    def _read_sensor_dist_desc(self) -> str:
        """读当前所选传感器的「数据分布描述」（MI 0x31 SI 0x2B），返回 ASCII 描述串。

        0x2B 有两种固件实现，这里先用 **EDL=1 探一个字节** 再决定怎么读——对只有
        1 字节的子索引直接请求 64 字节可能被固件判为非法长度：
          · 首字节是字母  → 0x2B 本身就是描述串，接着整段读回来；
          · 首字节是 1~64 → 0x2B 是描述长度，内容在 0x2C（xlsx 表格行的表述，
                            实机固件即此种：0x2B=8、0x2C="Fn_U8_70"）；
          · 首字节为 0    → 该传感器没有描述（手掌无传感器时即为 0）。
        两条路都拿不到就返回 ''（调用方回退到「整段都是点阵」的假设）。
        """
        head = self._request(MI.SENSOR_CMD, SensorSI.DIST_DESC[0], 1)
        if not head:
            return ''
        b0 = head[0]
        if 0x20 <= b0 < 0x7F and chr(b0).isalpha():          # 0x2B = 描述串本身
            desc = self._read_desc_string(SensorSI.DIST_DESC[0], SensorSI.DIST_DESC[1])
            if desc:
                return desc
        if 0 < b0 <= SensorSI.DIST_DESC_ALT[1]:              # 0x2B = 长度, 0x2C = 内容
            desc = self._read_desc_string(SensorSI.DIST_DESC_ALT[0], b0)
            if desc:
                return desc
        if b0:
            print(f"ℹ️ 分布描述(0x2B 首字节 0x{b0:02X})既不像描述串也不像长度，"
                  f"按整段点阵解析")
        return ''

    def _read_desc_string(self, si: int, length: int) -> str:
        """从 si 起读一段描述字符串：长度上限 length，单帧装不下时按可行长度递减重试。
        （固件对超出字段长度的 EDL 可能直接回错误码，故逐档试探。）"""
        want = min(length, MAX_LD)
        for n in (want, 48, 32, 16, 8):
            if n > want:
                continue
            desc = self._decode_sensor_desc(
                self._request(MI.SENSOR_CMD, si, n))
            if self._looks_like_sensor_desc(desc):
                return desc
        return ''

    @staticmethod
    def _build_sensor_plan(total: int) -> List[Tuple[int, int, int]]:
        """把「数据总长度」拆成 [(MI, SI, 本帧字节数), ...] 的读取计划。
        通道划分：0x32=[0,255) 0x33=[255,510) 0x34=[510,765)，各通道 SI 从 0 起。"""
        plan: List[Tuple[int, int, int]] = []
        off = 0
        while off < total:
            ch = off // SENSOR_CHANNEL_SPAN
            if ch >= len(SENSOR_CHANNEL_MIS):
                print(f"⚠️ 传感器数据总长 {total} 字节超出 0x32/0x33/0x34 三通道覆盖范围"
                      f"({SENSOR_CHANNEL_SPAN * len(SENSOR_CHANNEL_MIS)} 字节)，超出部分不读")
                break
            si = off % SENSOR_CHANNEL_SPAN
            n = min(MAX_LD, total - off, SENSOR_CHANNEL_SPAN - si)
            plan.append((SENSOR_CHANNEL_MIS[ch], si, n))
            off += n
        return plan

    # ---- 传感器可用性（全部纯缓存查询，不发帧） ---- #
    def is_sensor_available(self, finger: int) -> bool:
        """该传感器是否可用（初始化探测结论，纯缓存查询）。
        未做过探测时返回 True——此时无从判断，交由实读决定。"""
        if not self._sensor_probed:
            return True
        return finger in self.sensors

    def get_available_sensors(self) -> List[int]:
        """可用传感器编号列表（升序），不发帧。例：[1,2,3,4,5] 表示只有五指有传感器。"""
        return list(self.available_sensors)

    def get_unavailable_sensors(self) -> Dict[int, str]:
        """不可用传感器及原因 {finger: 原因}，不发帧。例：{6: '未安装'}。"""
        return dict(self.sensor_unavailable)

    def _sensor_unavailable_msg(self, finger: int) -> str:
        name = SENSOR_FINGER_NAMES.get(finger, f"传感器{finger}")
        reason = self.sensor_unavailable.get(finger, "未探测到")
        return (f"❌ {name}传感器不可用({reason})，本机可用传感器: "
                f"{[SENSOR_FINGER_NAMES.get(f, f) for f in self.available_sensors]}")

    def select_sensor(self, finger: int, force: bool = False) -> bool:
        """选择要读取的传感器（MI 0x31 SI 0x6E，见 SensorFinger：1拇指~5小指, 6手掌）。

        force=False 时，若目标已是当前所选传感器则直接返回 True 不发帧——高频
        连续读同一传感器时省掉一半帧。
        初始化探测已判定不可用的编号（如本机没有手掌传感器）在此**直接拒绝、
        不上总线**，避免为不存在的传感器反复等超时。
        """
        if self._sensor_probed and not self._probing_sensors \
                and finger not in self.sensors:
            print(self._sensor_unavailable_msg(finger))
            return False
        if not force and self._selected_sensor == finger:
            return True
        ok = self._write(MI.SENSOR_CMD, SensorSI.SELECT, bytes([finger & 0xFF]))
        self._selected_sensor = finger if ok else None
        return ok

    def get_selected_sensor(self) -> Optional[int]:
        """回读设备当前所选传感器编号（MI 0x31 SI 0x6E），并同步本地记录。"""
        body = self._request(MI.SENSOR_CMD, SensorSI.SELECT, 1)
        if not body:
            return None
        self._selected_sensor = body[0]
        return body[0]

    def get_sensor_total_length(self) -> Optional[int]:
        """读当前所选传感器的数据总长度（MI 0x31 SI 0x6C，小端 uint16）。
        ⚠️ 高频场景请改用 get_sensor_length()（走初始化缓存，不发帧）。"""
        body = self._request(MI.SENSOR_CMD, *SensorSI.TOTAL_LENGTH)
        return int.from_bytes(body[:2], 'little') if body and len(body) >= 2 else None

    def get_sensor_length(self, finger: Optional[int] = None) -> Optional[int]:
        """取传感器数据总长度——纯缓存查询，不发任何帧（初始化时已探测）。
        finger 为 None 时取当前所选传感器。缓存里没有则返回 None。"""
        if finger is None:
            finger = self._selected_sensor
        return self.sensor_lengths.get(finger) if finger is not None else None

    def get_sensor_info(self, finger: Optional[int] = None, refresh: bool = False,
                        apply_shape: bool = True) -> Optional[Dict]:
        """取传感器元信息：类型/单位/量程/行/列/总长/分布描述/字段布局/每格字节数。

        默认走 initialize() 阶段的缓存（不发帧）；refresh=True 时重新选择该传感器
        并实读一遍元信息并更新缓存。finger 为 None 时取当前所选传感器。
        apply_shape=False 时不把最大行/列写回数据行/列（用于手动改过形态后刷新缓存）。
        """
        if finger is None:
            finger = self._selected_sensor
        if finger is None:
            return None
        if not refresh and finger in self.sensors:
            return self.sensors[finger]
        if self._sensor_probed and finger not in self.sensors:
            print(self._sensor_unavailable_msg(finger))   # 不可用：本地拒绝，不上总线
            return None
        info, _ = self._collect_sensor_info(finger, apply_shape=apply_shape)
        if info is None:
            return None
        self.sensors[finger] = info
        self.sensor_lengths[finger] = info["total_length"]
        self.sensor_names[finger] = info["name"]
        self.sensor_unavailable.pop(finger, None)
        if finger not in self.available_sensors:
            self.available_sensors = sorted(self.sensors)
        return info

    def _read_plan(self, plan: List[Tuple[int, int, int]]) -> Optional[bytes]:
        """按预生成的读取计划顺序取数据；任一段失败即返回 None（避免半帧数据）。"""
        out = bytearray()
        for mi, si, n in plan:
            body = self._request(mi, si, n)
            if not body:
                return None
            out += body
            if len(body) < n:      # 设备返回不足，认为读到末尾
                break
        return bytes(out)

    def _read_sensor_channel(self, mi: int, count: int) -> bytes:
        """从某数据通道(本地 SI 偏移 0 起)分段读 count 字节，每段≤MAX_LD(CAN FD 单帧)。
        单通道最多覆盖 SENSOR_CHANNEL_SPAN=255 字节(协议分三通道正是此原因)。"""
        out = bytearray()
        off = 0
        while off < count and off < SENSOR_CHANNEL_SPAN:
            n = min(MAX_LD, count - off, SENSOR_CHANNEL_SPAN - off)
            body = self._request(mi, off, n)
            if not body:
                break
            out += body
            if len(body) < n:   # 设备返回不足，认为读到末尾
                break
            off += n
        return bytes(out)

    def read_tactile_raw(self, finger: Optional[int] = None) -> Optional[bytes]:
        """读取触觉完整原始数据（高频路径）。

        finger 给定时先选该传感器（已选中则不发选择帧）；为 None 时读当前所选。
        长度与分段计划全部来自初始化缓存，**不发元信息帧**；缓存缺失且未做过探测
        时才回退到实读一次元信息补建缓存。
        初始化已判定不可用的编号（本机手掌）直接返回 None，不上总线。
        返回的是数据区原始字节：开头是分布描述里的合力等标量，点阵在其后，
        解析用 parse_tactile() / get_tactile_force() / read_tactile_matrix()。
        """
        target = finger if finger is not None else self._selected_sensor
        if target is None:
            print("❌ 未选择传感器，请先 select_sensor(...)")
            return None
        if self._sensor_probed and target not in self.sensors:
            print(self._sensor_unavailable_msg(target))
            return None
        if finger is not None and not self.select_sensor(finger):
            return None
        cached = self.sensors.get(target)
        if cached is None:          # 未探测过 → 实读一次元信息补建缓存
            cached = self.get_sensor_info(target, refresh=True)
            if cached is None:
                return None
        data = self._read_plan(cached["plan"])
        return data[:cached["total_length"]] if data else None

    # ---- 数据区解析（合力标量 + 点阵，全部来自同一次读取） ---- #
    @staticmethod
    def _get_int(raw: bytes, off: int, size: int, signed: bool = False) -> Optional[int]:
        """从 raw 的 off 处取 size 字节小端整数；越界返回 None。"""
        if off + size > len(raw):
            return None
        return int.from_bytes(raw[off:off+size], 'little', signed=signed)

    @classmethod
    def parse_tactile(cls, raw: bytes, info: Dict) -> Dict:
        """按缓存的字段布局解析一帧触觉原始数据（纯计算，不发帧）。

        返回::

            {"fields": {标签: 值或值列表, ...},      # 分布描述里的全部字段
             "force":  {"法向力合力": v, "切向力合力": v, "切向力合力方向": v},
             "cells":  [点阵值...],                  # 一维，按分布描述顺序
             "matrix": [[...], ...]}                 # 行优先二维（行列取自元信息）

        分布描述缺失时退化为「整段都是点阵」，force 为空字典。
        """
        fields: Dict[str, object] = {}
        force: Dict[str, int] = {}
        for f in info.get("layout") or ():
            if f["count"] == 1:
                v = cls._get_int(raw, f["offset"], f["elem_size"], f["signed"])
                fields[f["tag"]] = v
                if f["tag"] in SENSOR_FORCE_TAGS and v is not None:
                    force[f["name"]] = v
            else:
                vals = [cls._get_int(raw, f["offset"] + i * f["elem_size"],
                                     f["elem_size"], f["signed"]) or 0
                        for i in range(f["count"])]
                fields[f["tag"]] = vals
        # 点阵：优先用布局里的点阵字段，否则整段当点阵
        rows, cols = info.get("rows", 0), info.get("cols", 0)
        mat_field = info.get("matrix_field")
        if mat_field is not None:
            cells = list(fields.get(mat_field["tag"]) or ())
        else:
            bpc = info.get("bytes_per_cell", 1)
            n = info.get("matrix_count") or (rows * cols)
            off0 = info.get("matrix_offset", 0)
            cells = [cls._get_int(raw, off0 + i * bpc, bpc) or 0 for i in range(n)]
        matrix = None
        if rows and cols:
            padded = cells + [0] * max(0, rows * cols - len(cells))
            matrix = [padded[r*cols:(r+1)*cols] for r in range(rows)]
        return {"fields": fields, "force": force, "cells": cells, "matrix": matrix}

    def read_tactile(self, finger: Optional[int] = None) -> Optional[Dict]:
        """读一帧并完整解析（合力 + 点阵 + 全部字段）——高频推荐入口。
        一次数据读取拿到全部信息，比分别调 get_tactile_force / read_tactile_matrix
        省一半总线流量。"""
        raw = self.read_tactile_raw(finger)
        if raw is None:
            return None
        target = finger if finger is not None else self._selected_sensor
        info = self.sensors.get(target)
        if info is None:
            return None
        out = self.parse_tactile(raw, info)
        out["finger"] = target
        out["name"] = info["name"]
        out["unit"] = info.get("unit", "")
        out["raw"] = raw
        return out

    def get_tactile_force(self, finger: Optional[int] = None) -> Optional[Dict[str, int]]:
        """读取合力值：{法向力合力, 切向力合力, 切向力合力方向}（原始计数值）。

        合力不是独立寄存器——它由「数据分布描述」(0x2B) 声明在数据区开头
        （示例 ``FnS_U16_1;FtS_U16_1;FdS_U16_1;Rsv_U16_1;Fn_U8_40``），
        与点阵一次读回。若该传感器的分布描述里没有合力字段则返回空字典。
        ⚠️ 协议只给了「数据单位」(0x20) 和「量程」(0x28)，未给原始值→物理量的
           换算公式，要 N/g 需实测标定。
        """
        raw = self.read_tactile_raw(finger)
        if raw is None:
            return None
        target = finger if finger is not None else self._selected_sensor
        info = self.sensors.get(target)
        if info is None:
            return None
        return self.parse_tactile(raw, info)["force"]

    def get_all_tactile_force(self) -> Dict[int, Dict[str, int]]:
        """依次读取全部已缓存传感器的合力值，返回 {finger: {字段名: 值}}。"""
        out: Dict[int, Dict[str, int]] = {}
        for finger in self.sensors:
            f = self.get_tactile_force(finger)
            if f:
                out[finger] = f
        return out

    def get_tactile_summary(self, finger: Optional[int] = None) -> Optional[Dict]:
        """一次读取给出抓取判定常用量：设备上报的合力 + 点阵统计。

        返回 {finger, name, force(设备合力), cell_sum(点阵求和), cell_max,
        contact(非零点数), centroid(质心 row,col 或 None)}。
        cell_sum 是上位机对点阵求和的「合力近似」，与设备上报的 force 可互相校核。
        """
        frame = self.read_tactile(finger)
        if frame is None:
            return None
        cells = frame["cells"]
        info = self.sensors.get(frame["finger"]) or {}
        cols = info.get("cols", 0)
        total = sum(cells)
        contact = sum(1 for v in cells if v)
        centroid = None
        if total and cols:
            sr = sum((i // cols) * v for i, v in enumerate(cells))
            sc = sum((i % cols) * v for i, v in enumerate(cells))
            centroid = (round(sr / total, 2), round(sc / total, 2))
        return {
            "finger":   frame["finger"],
            "name":     frame["name"],
            "force":    frame["force"],
            "cell_sum": total,
            "cell_max": max(cells) if cells else 0,
            "contact":  contact,
            "centroid": centroid,
        }

    def read_tactile_matrix(self, finger: int) -> Optional[List[List[int]]]:
        """读取触觉并按行优先还原为二维点阵矩阵（布局/行列取自初始化缓存）。

        点阵起始偏移与每格字节数由「数据分布描述」给出——数据区开头的合力等
        标量会被跳过（旧实现把它们当成前几个点阵值，是错的）。描述缺失时退化为
        「整段都是点阵、每格 = 总长÷(行×列)」。
        ⚠️ 该还原假设设备处于「填充无效数据补矩形」模式(SI 0x72 默认 1)；若关闭
           该模式，数据区是一维有效数据，需按 dist_desc 自行映射。
        """
        info = self.sensors.get(finger) or self.get_sensor_info(finger, refresh=True)
        if not info:
            return None
        if not info["rows"] or not info["cols"]:
            return None
        raw = self.read_tactile_raw(finger)
        if raw is None:
            return None
        return self.parse_tactile(raw, info)["matrix"]

    def read_all_tactile_raw(self) -> Dict[int, bytes]:
        """依次读取全部已缓存传感器的原始数据，返回 {finger: bytes}（读失败者不收录）。"""
        out: Dict[int, bytes] = {}
        for finger in self.sensors:
            raw = self.read_tactile_raw(finger)
            if raw is not None:
                out[finger] = raw
        return out

    def read_all_tactile_matrix(self) -> Dict[int, List[List[int]]]:
        """依次读取全部已缓存传感器并还原为矩阵，返回 {finger: 矩阵}。"""
        out: Dict[int, List[List[int]]] = {}
        for finger in self.sensors:
            m = self.read_tactile_matrix(finger)
            if m is not None:
                out[finger] = m
        return out

    def get_tactile_data(self, finger: Optional[int] = None) -> Optional[Dict]:
        """**一次读取**同时返回点阵矩阵与合力值，字典 key 全为英文（协议标签）。

        合力与点阵本来就在同一个数据区里（合力在开头、点阵在其后），所以这里只发
        一次数据读——比分别调 read_tactile_matrix() + get_tactile_force() 省一半
        总线流量，高频循环用这个。返回::

            {"finger":   1,                    # 传感器编号(1拇指~5小指,6手掌)
             "name":     "大拇指",
             "rows":     10, "cols": 7,        # 点阵行列（元信息缓存）
             "unit":     "N",                  # 设备自报数据单位
             "force":    {"FnS": 123, "FtS": 45, "FdS": 90},
             "matrix":   [[...], ...],         # 行优先二维点阵
             "cells":    [...],                # 一维点阵（按分布描述顺序）
             "cell_sum": 0, "cell_max": 0}     # 点阵求和/最大值（上位机算的）

        force 的 key 直接用协议「数据分布描述」里的标签：
          ``FnS`` 法向力合力、``FtS`` 切向力合力、``FdS`` 切向力合力方向。
        **描述里没有声明的合力字段不会出现**（如实机 ``Fn_U8_70`` 只有点阵，
        force 即为空字典）——不臆造设备没上报的量，这种情况用 cell_sum 近似。
        读失败 / 传感器不可用时返回 None。
        """
        frame = self.read_tactile(finger)
        if frame is None:
            return None
        info = self.sensors.get(frame["finger"]) or {}
        cells = frame["cells"]
        fields = frame["fields"]
        return {
            "finger":   frame["finger"],
            "name":     frame["name"],
            "rows":     info.get("rows", 0),
            "cols":     info.get("cols", 0),
            "unit":     frame.get("unit", ""),
            "force":    {tag: fields[tag] for tag in SENSOR_FORCE_TAGS
                         if isinstance(fields.get(tag), int)},
            "matrix":   frame["matrix"],
            "cells":    cells,
            "cell_sum": sum(cells),
            "cell_max": max(cells) if cells else 0,
        }

    def get_all_tactile_data(self) -> Dict[int, Dict]:
        """依次读取全部可用传感器的「矩阵 + 合力」，返回 {finger: get_tactile_data(...)}。
        每个传感器只发「选择 + 数据」两步，读失败者不收录。"""
        out: Dict[int, Dict] = {}
        for finger in self.sensors:
            d = self.get_tactile_data(finger)
            if d is not None:
                out[finger] = d
        return out

    # ---- 传感器数据区形态设置（MI 0x31 可写项） ---- #
    def set_sensor_pad_rect(self, enable: bool) -> bool:
        """设置是否填充无效数据把传感器外轮廓补成矩形（SI 0x72，默认 1=补齐）。
        关闭后数据区变为一维有效数据，总长度会变——本方法会刷新对应缓存。"""
        if not self._write(MI.SENSOR_CMD, SensorSI.PAD_RECT,
                           bytes([1 if enable else 0])):
            return False
        if self._selected_sensor is not None:
            self.get_sensor_info(self._selected_sensor, refresh=True)
        return True

    def set_sensor_flag_mode(self, enable: bool) -> bool:
        """切换「传感器有效数据标识模式」（SI 0x71，默认 0）。
        置 1 后数据区固定为 有效=1 / 无效=0，用于查看补零后的位置关系。
        ⚠️ 实测部分固件该项无效（写入被忽略），数据区仍为真实读数。"""
        return self._write(MI.SENSOR_CMD, SensorSI.FLAG_MODE,
                           bytes([1 if enable else 0]))

    def set_sensor_data_shape(self, rows: int, cols: int) -> bool:
        """设置数据行/列（SI 0x6F / 0x70，可写）。改动后刷新对应缓存。
        注意刷新时不会再把最大行/列写回，以保留这里设置的形态。"""
        ok = self._write(MI.SENSOR_CMD, SensorSI.DATA_ROWS, bytes([rows & 0xFF]))
        ok = self._write(MI.SENSOR_CMD, SensorSI.DATA_COLS, bytes([cols & 0xFF])) and ok
        if ok and self._selected_sensor is not None:
            self.get_sensor_info(self._selected_sensor, refresh=True, apply_shape=False)
        return ok

    def print_sensors(self):
        """打印初始化探测结论：可用传感器的元信息/字段布局 + 不可用清单（不发帧）。"""
        print("=" * 50)
        if not self._sensor_probed:
            print("ℹ️ 尚未探测传感器（probe_sensor=False，可手动调 probe_sensors()）")
        if not self.sensors:
            print("ℹ️ 无可用传感器")
        else:
            print(f"可用传感器 {len(self.sensors)} 个 {self.available_sensors}"
                  f"（五指={'有' if self.has_finger_sensors else '无'}, "
                  f"手掌={'有' if self.has_palm_sensor else '无'}）:")
        for finger, v in self.sensors.items():
            print(f"  [{finger}] {v['name']:6s} 类型={v['type'] or '-':16s} "
                  f"总长={v['total_length']:4d}B 行列={v['rows']}x{v['cols']} "
                  f"每格={v['bytes_per_cell']}B 单位={v['unit'] or '-'} "
                  f"量程={v['range']} 帧数={len(v['plan'])}")
            if v["dist_desc"]:
                print(f"        分布描述: {v['dist_desc']}")
            for f in v.get("layout") or ():
                print(f"          偏移 {f['offset']:3d} +{f['size']:3d}B  "
                      f"{f['type']}×{f['count']:<3d} {f['name']}")
            if not v.get("force_fields"):
                print("        ⚠️ 该传感器分布描述中无合力字段，合力需自行对点阵求和")
        if self.sensor_unavailable:
            print(f"不可用传感器 {len(self.sensor_unavailable)} 个（读取会被本地直接拒绝）:")
            for finger, reason in sorted(self.sensor_unavailable.items()):
                name = SENSOR_FINGER_NAMES.get(finger, f"传感器{finger}")
                print(f"  [{finger}] {name:6s} {reason}")
        print("=" * 50)

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

    def dump_product_info(self, total: int = 0xF9, chunk: int = 48):
        """逐段读产品信息(MI 0x41)并按 偏移/ASCII 打印，用于核对真实字段偏移。
        默认覆盖到 0xF9（最后一个字段：左右手 0xF1+8）。
        换固件或字段错位时跑这个，对照打印结果调整 ProductInfoSI 偏移即可。"""
        print("=" * 60)
        print("产品信息原始转储 (偏移: ASCII)：")
        off = 0
        while off < total and off <= 0xFF:
            n = min(chunk, total - off, 0x100 - off, MAX_LD)
            body = self._request(MI.PRODUCT_INFO, off, n)
            if not body:
                print(f"  0x{off:02X}: <读取失败/超时>")
                break
            asc = ''.join(chr(x) if 32 <= x < 127 else '·' for x in body)
            print(f"  0x{off:02X} ({len(body):2d}B): {asc}")
            off += len(body)
        print("=" * 60)

    def get_device_info(self) -> Dict[str, Optional[str]]:
        """读取一组常用产品信息字段（上电优先识别，偏移见 ProductInfoSI/xlsx v0.0.4）"""
        fields = {
            "产品型号":     ProductInfoSI.MODEL,
            "供电电压范围": ProductInfoSI.VOLTAGE_RANGE,
            "设备唯一标识": ProductInfoSI.DEVICE_UID,
            "协议名称":     ProductInfoSI.PROTOCOL_NAME,
            "协议版本":     ProductInfoSI.PROTOCOL_VERSION,
            "接口板版本":   ProductInfoSI.HW_VER_INTERFACE,
            "转接板版本":   ProductInfoSI.HW_VER_ADAPTER,
            "控制板版本":   ProductInfoSI.HW_VER_CONTROL,
            "app版本":      ProductInfoSI.APP_VERSION,
            "机械结构版本": ProductInfoSI.MECH_VERSION,
            "编译时间":     ProductInfoSI.BUILD_TIME,
            "支持协议":     ProductInfoSI.SUPPORTED_PROTO,
        }
        info: Dict[str, Optional[str]] = {
            name: self._read_string(si_len) for name, si_len in fields.items()
        }
        # 左右手优先用工程服务枚举(0x42/0x49)，不依赖产品信息字段偏移
        info["左右手"] = self.get_hand_side()
        # 传感器：类型/长度直接取初始化缓存，不再发帧
        if self.sensors:
            info["传感器类型"] = "、".join(
                f"{v['name']}:{v['type'] or '-'}" for v in self.sensors.values())
        else:
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
    # initialize(probe_sensor=True) 时会在上电阶段一次性探测并缓存各传感器的
    # 数据总长度/行列/分段读取计划；之后 read_tactile_raw() 只发数据帧。
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

            # ---- 传感器：初始化已判定可用性并缓存元信息 ---- #
            hand.print_sensors()                       # 可用/不可用清单 + 字段布局
            print("可用传感器:", hand.get_available_sensors())      # 例 [1,2,3,4,5]
            print("不可用   :", hand.get_unavailable_sensors())     # 例 {6:'未安装'}
            print("有手掌传感器:", hand.has_palm_sensor)
            # 不可用编号会被本地直接拒绝，不发帧、不等超时
            if not hand.is_sensor_available(SensorFinger.PALM):
                print("跳过手掌传感器读取")
            for finger in hand.available_sensors:
                s = hand.get_tactile_summary(finger)
                print(f"  {hand.sensor_names[finger]}: 合力={s['force']} "
                      f"点阵和={s['cell_sum']} 峰值={s['cell_max']} "
                      f"触点={s['contact']} 质心={s['centroid']}")
            if hand.available_sensors:
                finger = hand.available_sensors[0]
                print(f"缓存长度: {hand.get_sensor_length(finger)} 字节")  # 不发帧
                t0 = time.time()
                loops = 20
                for _ in range(loops):                 # 同一传感器连读：选择帧只发 1 次
                    frame = hand.read_tactile(finger)  # 一次读取 → 合力 + 点阵
                dt = (time.time() - t0) / loops
                print(f"read_tactile 平均 {dt*1000:.1f} ms/次")
                if frame:
                    # 合力值（法向力合力/切向力合力/方向）来自数据区开头，无需额外帧
                    print("合力:", frame["force"])
                    for row in frame["matrix"] or []:
                        print("  " + " ".join(f"{v:4d}" for v in row))
                print("全部合力:", hand.get_all_tactile_force())

            # ---- 故障与错误码诊断 ---- #
            hand.print_joint_faults()      # MI 0x0C 关节故障位
            hand.print_error_history()     # MI 0x4F 最新槽号 + 15 条历史
