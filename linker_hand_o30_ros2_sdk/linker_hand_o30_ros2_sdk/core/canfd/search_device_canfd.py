# -*- coding: utf-8 -*-
"""CAN FD 总线灵巧手设备搜索 —— Linker Hand O6 / L6（HOP 协议 v0.0.4）

用途：探测指定 CAN FD 线路上是否存在灵巧手设备；存在则读回并打印
      产品型号 / 供电电压范围 / 设备唯一标识 / 协议名称与版本 /
      接口板与控制板版本 / app 版本 / 编译时间 / 支持协议 / 左右手 / 传感器类型。
      只做「搜索 + 读身份信息」，不含任何运动控制或配置写入。

支持两种 CAN FD 通讯方式（与 linker_hand_o6_canfd.py 一致）：
    socketcan : 透明塑封 USB 转 CANFD 设备，走内核原生 can0 + python-can
    libcanbus : 厂商私有库 /usr/local/lib/libcanbus.so 的 USB-CANFD 设备
    默认 auto —— 通道名是网卡且硬件支持 FD（mtu 72 或 maxmtu 72）就走 socketcan，
                 否则有 libcanbus.so 就走厂商库。所以 can0 上插着经典 CAN 适配器
                 （pcan_usb 那类 maxmtu 16 的）时不会误选 socketcan。

原理（协议格式表）：
    请求帧数据域 = byte0(bit7=RTS | bit6~0=MI) + byte1(SI) + byte2(EDL) [+ 负载]
        不带负载 → 读，EDL = 期望返回的数据长度
    返回帧 CAN ID = 请求帧 ID 最高位置 1（标准帧 | 0x400，扩展帧 | 0x10000000）
    返回帧数据域 = 回显 3 字节帧头(MI/SI/EDL) + 负载数据

    0x7FF（标准帧）/ 0x1FFFFFFF（扩展帧）是设备的特殊响应 ID：不知道从机 ID 时
    向该 ID 发指令，设备会用「自己的返回 ID」应答 —— 于是从应答帧的 CAN ID
    即可反推出设备 ID：device_id = reply_id & ~0x400。本模块就用这一步做搜索。

    身份信息取自产品信息表 MI 0x41（全部 ro，O6 / L6 通用）；左右手取自调试功能 A
    的 MI 0x42/0x49 枚举（不依赖产品信息字段偏移），传感器类型取自传感器表
    MI 0x31/0x00 的类型字符串。全程只读不写。

CAN FD 单帧数据域 64 字节 - 3 字节帧头 = 单帧最多 61 字节负载，本模块要读的字段
最长 32 字节，因此每个字段一帧问答即可（经典 CAN 版本要拆 4~7 帧，见
../can/search_device_can.py）。帧间按 FRAME_INTERVAL 间隔发送。

一次完整搜索固定 12 帧（TX 12 / RX 12），≈ 12 × 2 ms = 24 ms：
    探测：协议名称  8 字节  0x41/0x59     1 帧（顺带反推设备 ID，其值直接复用）
    产品型号       16 字节  0x41/0x00     1 帧
    供电电压范围   16 字节  0x41/0x10     1 帧
    设备唯一标识   32 字节  0x41/0x39     1 帧
    协议版本        8 字节  0x41/0x61     1 帧
    接口板版本     16 字节  0x41/0x69     1 帧
    控制板版本     16 字节  0x41/0x89     1 帧
    app 版本        8 字节  0x41/0xA1     1 帧
    编译时间       32 字节  0x41/0xB1     1 帧
    支持协议       32 字节  0x41/0xD1     1 帧
    左右手          1 字节  0x42/0x49     1 帧（读不到时回退 0x41/0xF1，多 1 帧）
    传感器类型     32 字节  0x31/0x00     1 帧（为空时回退 0x42/0x4A，多 1 帧）

用法：
    python3 search_device_canfd.py                      # 自动选传输，can0 @ 1M/5M
    python3 search_device_canfd.py can0 1000000 5000000 # 指定接口与仲裁/数据波特率
    python3 search_device_canfd.py --libcanbus          # 强制走厂商库（设备0 通道0）
    python3 search_device_canfd.py --libcanbus 0:0      # 厂商库的 设备号:通道号
    python3 search_device_canfd.py --id=0x2             # 跳过探测，直问已知设备 ID
    python3 search_device_canfd.py --ext                # 用 29 位扩展帧搜索
    python3 search_device_canfd.py --setup              # 强制 down→重配→up
    python3 search_device_canfd.py --no-setup           # 完全不碰接口配置
    python3 search_device_canfd.py --no-reset           # 搜不到也不重置接口重试

    socketcan 方式下接口没 up 时会自动执行
    `sudo ip link set <ch> type can bitrate <b> dbitrate <d> fd on` + `... up`
    （四条命令合并成一次 sudo，只提示一次密码，需在有 tty 的终端里运行）；
    接口已经 up 则直接跳过，不改动现有配置。

    接口已经 up、但一帧应答都收不到时（gs_usb 那类透明塑封适配器在上个进程没
    正常关闭总线后会卡成这样），会自动 down→重配→up 一次再搜一遍，见
    SearchDeviceCANFD 的 reset_retry 参数。

    from search_device_canfd import SearchDeviceCANFD
    with SearchDeviceCANFD("can0", 1000000, 5000000) as s:
        info = s.search()
        if info:
            print(info.report())        # 只要设备身份那 12 行
            print(info.link_report())   # 只要链路参数与收发统计
            print(info.report_full())   # 两段都要（命令行默认输出）
            info.as_dict()["device_uid"]        # 逐字段取值
            info.device_uid, info.tx_count      # 或直接属性访问
"""

import os
import shutil
import subprocess
import sys
import threading
import time
from collections import deque
from ctypes import (CDLL, POINTER, RTLD_GLOBAL, Structure, byref, c_char,
                    c_uint, c_ubyte, c_ushort, cast, cdll)
from typing import Deque, Dict, List, NamedTuple, Optional, Tuple

STATUS_OK = 0

# ============================================================================
# 传输参数
# ============================================================================
FRAME_INTERVAL = 0.002       # 每帧通讯间隔（秒）= 2 ms，改这里即可调整节流
CANFD_MAX_DLEN = 64          # CAN FD 数据域上限
HEADER_LEN = 3               # MI/SI/EDL
MAX_PAYLOAD = CANFD_MAX_DLEN - HEADER_LEN        # = 61，单帧最大负载
REPLY_TIMEOUT = 0.3          # 单帧应答等待上限（秒）

BITRATE_DEFAULT = 1000000    # 仲裁段波特率（设备侧 0x35/0x08 默认 1=1000k）
DBITRATE_DEFAULT = 5000000   # 数据段波特率（设备侧 0x35/0x09 默认 5=2000k）

# 特殊搜索 ID —— 设备一定响应，用于在不知道从机 ID 时建立通信
DISCOVERY_ID_STD = 0x7FF
DISCOVERY_ID_EXT = 0x1FFFFFFF
# 返回帧 ID = 请求帧 ID 最高位置 1
REPLY_BIT_STD = 0x400        # 11 位 ID 的最高位
REPLY_BIT_EXT = 0x10000000   # 29 位 ID 的最高位
STD_ID_MAX = 0x3FE           # 标准帧可配置的设备 ID 上限
EXT_ID_MAX = 0xFFFFFFE       # 扩展帧可配置的设备 ID 上限

MI_PRODUCT_INFO = 0x41       # 产品信息表主索引
MI_ENGINEERING = 0x42        # 调试功能 A 主索引
MI_SENSOR = 0x31             # 传感器命令/元信息主索引
MI_ERROR_CODE = 0x4F         # 错误码帧主索引
RTS_REALTIME = 0             # byte0 bit7：0=返回实时状态值

# 调试功能 A 里两个只读枚举项 (SI, 字节长度)
SI_HAND_SIDE = (0x49, 1)     # uint8 0xF0=左手 0x0F=右手
SI_SENSOR_TYPE = (0x4A, 1)   # uint8 传感器设置（出厂配置项，可能没写，仅作回退）
SI_HAND_SIDE_STR = (0xF1, 8) # 产品信息表里的左右手字符串（回退用）
# 传感器自报的类型字符串 (MI 0x31 / SI 0x00, str32)，报文 31 00 20，例 "TSSP_FULAI"
SI_SENSOR_TYPE_STR = (0x00, 32)

HAND_SIDE_ENUM = {0xF0: "LEFT", 0x0F: "RIGHT"}
# 出厂配置枚举 (0x42/0x4A)，只在传感器不报类型字符串时用来兜底
SENSOR_TYPE_ENUM = {
    0x00: "无传感器",
    0x01: "TS传感器(使用采集板)",
    0x02: "HWK传感器",
    0x03: "FL传感器",
    0x04: "TSSP_FL传感器",
    0x05: "TSSP_JZG传感器",
    0x06: "TSSP_HWK传感器",
}
# 错误码位掩码（MI 0x4F 载荷，可组合）
ERROR_CODE_NAMES = {
    0x01: "主索引不存在", 0x02: "子索引不存在", 0x04: "寄存器不可写",
    0x08: "数据长度不匹配", 0x10: "权限不足（未输入密码）", 0x20: "返回数据存在补零",
}

# ip 命令路径：sudo 下 PATH 常常不含 /usr/sbin，写全路径最稳
IP_CMD = shutil.which("ip") or "/usr/sbin/ip"
RESTART_MS = 100             # 总线关闭后自动恢复间隔（ms）
LINK_RESET_SETTLE = 0.3      # 接口重新激活后等它稳定的时间（秒），太快发帧会丢
TXQUEUELEN = 1000
LIBCANBUS_PATH = "/usr/local/lib/libcanbus.so"
LIBUSB_PATH = "/usr/local/lib/libusb-1.0.so"

# 产品信息表字段 (键, 中文标签, SI, 字节长度) —— 顺序即打印顺序
INFO_FIELDS: Tuple[Tuple[str, str, int, int], ...] = (
    ("model",            "产品型号",     0x00, 16),
    ("voltage_range",    "供电电压范围", 0x10, 16),
    ("device_uid",       "设备唯一标识", 0x39, 32),
    ("protocol",         "协议名称",     0x59,  8),
    ("protocol_version", "协议版本",     0x61,  8),
    ("hw_interface",     "接口板版本",   0x69, 16),
    ("hw_control",       "控制板版本",   0x89, 16),
    ("app_version",      "app版本",      0xA1,  8),
    ("build_time",       "编译时间",     0xB1, 32),
    ("supported_proto",  "支持协议",     0xD1, 32),
)

# 搜索用的探测帧：读协议名称（8 字节），回来的必是 "HOP" —— 既确认是灵巧手，
# 又能从应答帧的 CAN ID 反推设备 ID。选它是因为返回值内容确定，好判真假。
PROBE_SI, PROBE_LEN = 0x59, 8
PROBE_EXPECT = b"HOP"

DLC2LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def _dlc_of(length: int) -> int:
    """字节数 → CAN FD 的 DLC 编码（合法帧长 0~8,12,16,20,24,32,48,64）"""
    for dlc, n in enumerate(DLC2LEN):
        if length <= n:
            return dlc
    return 15


CANFD_MTU = 72               # SocketCAN 的 canfd_frame 长度；经典 CAN 是 16


def iface_mtu(channel: str) -> Optional[int]:
    """接口当前 mtu（16=经典 CAN 模式，72=已开 fd）；接口不存在返回 None"""
    try:
        with open(f"/sys/class/net/{channel}/mtu") as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def iface_max_mtu(channel: str) -> Optional[int]:
    """接口硬件支持的最大 mtu（rtnetlink 的 maxmtu，sysfs 里没有，只能问 ip）

    经典 CAN 适配器（pcan_usb 等）maxmtu 恒为 16；FD 适配器为 72，
    即使当前没开 fd（mtu 还是 16）也能据此判断硬件到底支不支持 FD。
    """
    try:
        out = subprocess.run([IP_CMD, "-details", "link", "show", channel],
                             check=False, timeout=5, stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL).stdout.decode(errors="ignore")
    except Exception:
        return None
    toks = out.split()
    for i, tok in enumerate(toks[:-1]):
        if tok == "maxmtu":
            try:
                return int(toks[i + 1])
            except ValueError:
                return None
    return None


def iface_supports_fd(channel: str) -> bool:
    """接口硬件是否支持 CAN FD（当前 mtu 已是 72，或硬件 maxmtu 到 72）"""
    mtu = iface_mtu(channel)
    if mtu is None:
        return False
    if mtu >= CANFD_MTU:
        return True
    return (iface_max_mtu(channel) or 0) >= CANFD_MTU


def libcanbus_present() -> bool:
    return os.path.exists(LIBCANBUS_PATH)


# ============================================================================
# 搜索结果
# ============================================================================
class DeviceInfo(NamedTuple):
    """搜到的设备身份信息与本次搜索的通信统计"""
    model: str
    voltage_range: str
    device_uid: str
    protocol: str
    protocol_version: str
    hw_interface: str
    hw_control: str
    app_version: str
    build_time: str
    supported_proto: str
    hand_side: str
    sensor_type: str
    device_id: int               # 设备（请求）帧 ID
    reply_id: int                # 返回帧 ID = device_id | 最高位
    transport: str               # "socketcan" / "libcanbus"
    channel: str
    bitrate: int                 # 仲裁段波特率
    dbitrate: int                # 数据段波特率
    frame_interval: float        # 秒
    tx_count: int
    rx_count: int
    extended: bool = False

    @property
    def family(self) -> str:
        """从型号里判断是 O6 还是 L6（型号形如 LHO6 / LHL6）"""
        m = self.model.upper()
        for tag in ("O6", "L6"):
            if tag in m:
                return tag
        return "未知"

    def report(self) -> str:
        """设备身份信息：按固定字段顺序渲染成多行文本"""
        lines = ["设备信息:"]
        for key, label, _si, _n in INFO_FIELDS:
            lines.append(f"  {label}: {getattr(self, key)}")
        lines.append(f"  左右手: {self.hand_side}")
        lines.append(f"  传感器类型: {self.sensor_type}")
        return "\n".join(lines)

    def link_report(self) -> str:
        """本次搜索的链路参数与收发统计（DeviceInfo 里除身份信息外的字段）"""
        frame = "扩展帧 29 位" if self.extended else "标准帧 11 位"
        return "\n".join([
            "链路信息:",
            f"  请求/返回 ID: 0x{self.device_id:X} / 0x{self.reply_id:X}"
            f"（{frame}）",
            f"  传输方式: {self.transport}",
            f"  通道: {self.channel}",
            f"  仲裁/数据波特率: {self.bitrate} / {self.dbitrate}",
            f"  帧间隔: {self.frame_interval * 1000:.3f} ms",
            f"  单帧最大载荷: {MAX_PAYLOAD} 字节"
            f"（CAN FD {CANFD_MAX_DLEN} - {HEADER_LEN} 帧头）",
            f"  收发帧数: TX {self.tx_count} / RX {self.rx_count}",
        ])

    def report_full(self) -> str:
        """身份信息 + 链路信息，命令行默认打印这个"""
        return f"{self.report()}\n{self.link_report()}"

    def as_dict(self) -> Dict[str, object]:
        """全部字段的字典（键就是上面的字段名），要程序里取值用这个"""
        return dict(self._asdict())

    def __str__(self) -> str:
        return self.report_full()


# ============================================================================
# 传输层 —— 只需三件事：开/关、发一帧、同步收一帧
# ----------------------------------------------------------------------------
# 搜索是严格的一问一答，不需要后台接收线程，recv() 直接阻塞等到超时即可。
# 两种实现的帧格式完全一致，上层 SearchDeviceCANFD 不关心底层是哪种设备。
# ============================================================================
class _Transport:
    """传输层接口"""
    name = "base"

    def open(self) -> bool:
        raise NotImplementedError

    def send(self, can_id: int, data: bytes, extended: bool = False) -> bool:
        raise NotImplementedError

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        """收一帧 → (can_id, 数据域)；超时返回 None"""
        raise NotImplementedError

    def close(self):
        pass

    @property
    def describe(self) -> str:
        return self.name


class SocketCANFDTransport(_Transport):
    """内核原生 SocketCAN (python-can, fd=True) —— 透明塑封 USB 转 CANFD 设备

    该设备在 Linux 下走标准 SocketCAN，无需厂商私有库；type-c 下方的开关要拨到
    Linux 模式，否则只会枚举成 STM32 Virtual ComPort，不会出现 can0。
    """
    name = "socketcan"

    def __init__(self, channel: str = "can0",
                 bitrate: int = BITRATE_DEFAULT,
                 dbitrate: int = DBITRATE_DEFAULT,
                 auto_setup: bool = True, force_setup: bool = False,
                 verbose: bool = True):
        self.channel = channel
        self.bitrate = int(bitrate)
        self.dbitrate = int(dbitrate)
        self.auto_setup = auto_setup
        self.force_setup = force_setup
        self.verbose = verbose
        self.bus = None
        self._can = None             # python-can 模块引用

    @property
    def describe(self) -> str:
        return f"socketcan {self.channel} @ {self.bitrate}/{self.dbitrate}"

    # ---------------- 接口状态 ---------------- #
    def link_state(self) -> Optional[str]:
        """读 /sys 取接口状态（'up' / 'down'），接口不存在返回 None —— 不需要 sudo

        看 flags 里的 IFF_UP(0x1) 而不是 operstate：不上报 carrier 的接口拉起后
        operstate 仍是 'unknown'（lo 就是这样），据此判断会误报「配置后仍不是 up」。
        """
        try:
            with open(f"/sys/class/net/{self.channel}/flags") as f:
                return "up" if int(f.read().strip(), 16) & 0x1 else "down"
        except (OSError, ValueError):
            return None

    def _setup_interface(self, force: bool = False) -> bool:
        """用 sudo ip link 配置并拉起 CAN FD 接口；已经 up 就直接跳过。

        几条命令合并成一次 `sudo sh -c`，所以最多只提示一次密码。
        不重定向输出 —— sudo 的密码提示要能出现在终端上供手动输入。

        必需项（bitrate/dbitrate/fd on、up）用 && 串联，任一失败即判定配置失败；
        可选项（restart-ms、txqueuelen）单独执行且失败不影响结果 —— 部分驱动
        （如 gs_usb 的透明塑封 USB-CANFD）不支持 Bus Off 自动恢复，会报
        "Device doesn't support restart from Bus Off"，不能让它拖垮整条配置。
        """
        state = self.link_state()
        if state is None:
            print(f"❌ 未找到网络接口 {self.channel}，无法配置。", flush=True)
            print("   透明塑封设备需把 type-c 下方开关拨到 Linux 模式后重新插拔；", flush=True)
            print("   确认命令：ip -br link show type can", flush=True)
            return False
        if state == "up" and not force:
            if self.verbose:
                print(f"[CANFD] {self.channel} 已经是 up 状态，跳过配置"
                      f"（需强制重配请用 --setup）", flush=True)
            return True

        ch, br, dbr = self.channel, self.bitrate, self.dbitrate
        # restart-ms 要在接口 down 的状态下设，所以排在 up 之前；两个可选项都用
        # `... 2>/dev/null || true` 兜住，驱动不支持时只是没设上，不算配置失败。
        script = (f"{IP_CMD} link set {ch} down; "
                  f"{IP_CMD} link set {ch} type can bitrate {br} "
                  f"dbitrate {dbr} fd on && "
                  f"{{ {IP_CMD} link set {ch} type can restart-ms {RESTART_MS} "
                  f"2>/dev/null || true; }} && "
                  f"{IP_CMD} link set {ch} up && "
                  f"{{ {IP_CMD} link set {ch} txqueuelen {TXQUEUELEN} "
                  f"2>/dev/null || true; }}")
        print(f"[CANFD] 配置 {ch} @ {br}/{dbr} fd on（需要 sudo，请在终端输入密码）：", flush=True)
        print(f"        sudo {IP_CMD} link set {ch} type can bitrate {br} "
              f"dbitrate {dbr} fd on", flush=True)
        print(f"        sudo {IP_CMD} link set {ch} up", flush=True)
        sys.stdout.flush()          # 先把说明刷出去，再让 sudo 的密码提示出现
        try:
            rc = subprocess.call(["sudo", "sh", "-c", script])
        except Exception as e:
            print(f"❌ 执行 sudo 失败: {e}", flush=True)
            return False
        if rc != 0:
            print(f"❌ 配置 {ch} 失败（sudo 返回 {rc}），具体原因见上方 ip 命令的报错。", flush=True)
            print(f"   若提示 \"a terminal is required to read the password\"，"
                  f"说明当前环境没有 tty，请在普通终端里运行；", flush=True)
            print(f"   若提示波特率/参数无效，说明该驱动不支持这组参数，"
                  f"可先手动执行上面两条命令确认。", flush=True)
            return False
        time.sleep(0.2)
        if self.link_state() != "up":
            print(f"❌ {ch} 配置后仍不是 up 状态", flush=True)
            return False
        if self.verbose:
            self._show_timing()
        return True

    def fd_capable(self) -> bool:
        """接口当前是否处于 CAN FD 模式 —— 看 mtu：经典 CAN 是 16，CAN FD 是 72

        经典 CAN 适配器（如 pcan_usb，maxmtu 16）上发 FD 帧会被内核直接拒绝，
        报的是很难懂的 "Failed to transmit: Invalid argument [Error Code 22]"，
        所以开总线前先看一眼，把原因说清楚。
        """
        mtu = iface_mtu(self.channel)
        return mtu is None or mtu >= CANFD_MTU   # 读不到就别拦，让发送去报错

    def _explain_not_fd(self):
        """mtu 不到 72 时说清是「硬件不支持」还是「只是没开 fd」"""
        if iface_supports_fd(self.channel):
            print(f"❌ {self.channel} 当前不是 CAN FD 模式（mtu "
                  f"{iface_mtu(self.channel)}），但硬件支持 FD。", flush=True)
            print(f"   用 --setup 强制重配（down → fd on → up）：", flush=True)
            print(f"   python3 {os.path.basename(__file__)} {self.channel} "
                  f"{self.bitrate} {self.dbitrate} --setup", flush=True)
            return
        print(f"❌ {self.channel} 不支持 CAN FD（mtu 16，经典 CAN 适配器）。", flush=True)
        print("   本模块搜的是 CAN FD 设备；若手上这条是经典 CAN 线路，"
              "请用 ../can/search_device_can.py。", flush=True)
        if libcanbus_present():
            print("   若 CANFD 设备走的是厂商库，请加 --libcanbus 明确指定：", flush=True)
            print(f"   python3 {os.path.basename(__file__)} --libcanbus", flush=True)
        print(f"   确认命令：ip -det link show {self.channel}"
              f"（FD 接口 mtu 为 72，且时序行里带 dbitrate）", flush=True)

    def _show_timing(self):
        """回读实际生效的时序与状态，便于定位波特率不匹配 / BUS-OFF"""
        try:
            ret = subprocess.run([IP_CMD, "-details", "link", "show", self.channel],
                                 check=False, timeout=5,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out = ret.stdout.decode(errors="ignore")
            timing = next((l.strip() for l in out.splitlines() if "bitrate" in l), "")
            state = next((l.strip() for l in out.splitlines() if "state" in l), "")
            print(f"[CANFD] {self.channel} 已拉起，实际生效: {timing or '(未读到)'}", flush=True)
            if "BUS-OFF" in state or "ERROR-PASSIVE" in state:
                print("⚠️ 总线处于错误状态，通常是波特率与设备不一致或接线/终端电阻问题。", flush=True)
        except Exception:
            pass

    # ---------------- 收发 ---------------- #
    def open(self) -> bool:
        try:
            import can
        except ImportError:
            print("❌ 缺少 python-can 库，请执行: pip install python-can", flush=True)
            return False
        self._can = can

        state = self.link_state()
        if state is None and not (self.auto_setup or self.force_setup):
            print(f"❌ 未找到网络接口 {self.channel}。"
                  f"请检查设备是否插好、是否处于 Linux 模式", flush=True)
            return False

        if self.auto_setup or self.force_setup:
            if not self._setup_interface(force=self.force_setup):
                return False
        elif state != "up":
            print(f"❌ {self.channel} 当前是 {state} 状态，请先拉起：", flush=True)
            print(f"   sudo {IP_CMD} link set {self.channel} type can "
                  f"bitrate {self.bitrate} dbitrate {self.dbitrate} fd on", flush=True)
            print(f"   sudo {IP_CMD} link set {self.channel} up", flush=True)
            return False

        if not self.fd_capable():
            self._explain_not_fd()
            return False

        try:
            self.bus = can.interface.Bus(channel=self.channel,
                                         interface="socketcan", fd=True)
        except Exception as e:
            print(f"❌ 打开 {self.channel} 失败: {e}", flush=True)
            print(f"   接口是否已 up？ip -br link show {self.channel}", flush=True)
            return False
        return True


    def send(self, can_id: int, data: bytes, extended: bool = False) -> bool:
        if self.bus is None:
            return False
        data = bytes(data)[:CANFD_MAX_DLEN]
        # 零填充到合法 CAN FD 帧长（0~8,12,16,20,24,32,48,64）
        payload = data + b"\x00" * (DLC2LEN[_dlc_of(len(data))] - len(data))
        msg = self._can.Message(
            arbitration_id=can_id, is_extended_id=extended, is_fd=True,
            # 关闭 BRS：实测 BRS 开启 + 高速数据段会把总线打入 BUS-OFF，设备无法
            # 应答；设备侧配置项 0x35/0x07「是否使能波特率切换」出厂默认也是 0。
            bitrate_switch=False, data=payload)
        self.bus.send(msg, timeout=0.2)
        return True

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        if self.bus is None:
            return None
        msg = self.bus.recv(timeout=timeout)
        return None if msg is None else (msg.arbitration_id, bytes(msg.data))

    def close(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None


class CanFD_Config(Structure):
    _fields_ = [
        ("NomBaud", c_uint), ("DatBaud", c_uint), ("NomPres", c_ushort),
        ("NomTseg1", c_char), ("NomTseg2", c_char), ("NomSJW", c_char),
        ("DatPres", c_char), ("DatTseg1", c_char), ("DatTseg2", c_char),
        ("DatSJW", c_char), ("Config", c_char), ("Model", c_char),
        ("Cantype", c_char),
    ]


class CanFD_Msg(Structure):
    _fields_ = [
        ("ID", c_uint), ("TimeStamp", c_uint), ("FrameType", c_ubyte),
        ("DLC", c_ubyte), ("ExternFlag", c_ubyte), ("RemoteFlag", c_ubyte),
        ("BusSatus", c_ubyte), ("ErrSatus", c_ubyte), ("TECounter", c_ubyte),
        ("RECounter", c_ubyte), ("Data", c_ubyte * 64),
    ]


class LibCanBusTransport(_Transport):
    """厂商私有库 libcanbus.so 的 USB-CANFD 设备

    CANFD_Receive 一次可能带回多帧，多出来的先存进队列，下次 recv 直接取。
    """
    name = "libcanbus"
    RECV_BATCH = 64

    def __init__(self, device: int = 0, channel: int = 0,
                 bitrate: int = BITRATE_DEFAULT,
                 dbitrate: int = DBITRATE_DEFAULT,
                 verbose: bool = True):
        self.device = int(device)
        self.channel = int(channel)
        self.bitrate = int(bitrate)
        self.dbitrate = int(dbitrate)
        self.verbose = verbose
        self.canDLL = None
        self._rx: Deque[Tuple[int, bytes]] = deque()

    @property
    def describe(self) -> str:
        return (f"libcanbus 设备{self.device} 通道{self.channel} "
                f"@ {self.bitrate}/{self.dbitrate}")

    def open(self) -> bool:
        if not os.path.exists(LIBCANBUS_PATH):
            print(f"❌ 未找到厂商库 {LIBCANBUS_PATH}", flush=True)
            print("   若用的是透明塑封 USB-CANFD（走内核 can0），请加 --socketcan", flush=True)
            return False
        try:
            if os.path.exists(LIBUSB_PATH):
                CDLL(LIBUSB_PATH, RTLD_GLOBAL)
                time.sleep(0.1)
            self.canDLL = cdll.LoadLibrary(LIBCANBUS_PATH)

            n = self.canDLL.CAN_ScanDevice()
            if self.verbose:
                print(f"[CANFD] 扫描 libcanbus 设备：找到 {n} 个", flush=True)
            if n <= 0:
                print("❌ 未找到 CANFD 设备（libcanbus）", flush=True)
                return False

            ret = self.canDLL.CAN_OpenDevice(self.device, self.channel)
            if ret != STATUS_OK:
                print(f"❌ 打开设备失败: {ret}", flush=True)
                return False

            cfg = CanFD_Config(self.bitrate, self.dbitrate, 0, 0, 0, 0,
                               0, 0, 0, 0, 0x04, 0x0, 0x1)
            ret = self.canDLL.CANFD_Init(self.device, self.channel, byref(cfg))
            if ret != STATUS_OK:
                print(f"❌ CANFD 初始化失败: {ret}", flush=True)
                self.canDLL.CAN_CloseDevice(self.device, self.channel)
                return False

            ret = self.canDLL.CAN_SetFilter(self.device, self.channel,
                                            0, 0, 0, 0, 1)
            if ret != STATUS_OK:
                print(f"❌ 设置过滤器失败: {ret}", flush=True)
                self.canDLL.CAN_CloseDevice(self.device, self.channel)
                return False
            return True
        except Exception as e:
            print(f"❌ libcanbus 初始化失败: {e}", flush=True)
            return False

    def send(self, can_id: int, data: bytes, extended: bool = False) -> bool:
        if self.canDLL is None:
            return False
        data = bytes(data)[:CANFD_MAX_DLEN]
        buf = (c_ubyte * CANFD_MAX_DLEN)()
        for i, b in enumerate(data):
            buf[i] = b
        # FrameType=4(CANFD)，ExternFlag=0 标准帧 / 1 扩展帧
        msg = CanFD_Msg(can_id, 0, 4, _dlc_of(len(data)),
                        1 if extended else 0, 0, 0, 0, 0, 0, buf)
        return self.canDLL.CANFD_Transmit(self.device, self.channel,
                                          byref(msg), 1, 200) >= 1

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        if self.canDLL is None:
            return None
        deadline = time.perf_counter() + max(0.0, timeout)
        while True:
            if self._rx:
                return self._rx.popleft()
            remain = deadline - time.perf_counter()
            if remain <= 0:
                return None
            arr = (CanFD_Msg * self.RECV_BATCH)()
            wait_ms = max(1, min(20, int(remain * 1000)))
            try:
                n = self.canDLL.CANFD_Receive(
                    self.device, self.channel, cast(arr, POINTER(CanFD_Msg)),
                    self.RECV_BATCH, wait_ms)
            except Exception:
                return None
            for i in range(max(0, n)):
                m = arr[i]
                size = DLC2LEN[m.DLC] if m.DLC < len(DLC2LEN) else CANFD_MAX_DLEN
                self._rx.append((m.ID, bytes(m.Data[:size])))

    def close(self):
        if self.canDLL is not None:
            try:
                self.canDLL.CAN_CloseDevice(self.device, self.channel)
            except Exception:
                pass
        self.canDLL = None
        self._rx.clear()


# ============================================================================
# 搜索器
# ============================================================================
class SearchDeviceCANFD:
    """在一条 CAN FD 线路上搜索灵巧手设备并读回身份信息。

    只读不写：全程只发 MI 0x41 / 0x42 / 0x31 的读请求，不会改动设备任何参数、
    不会驱动关节。
    典型用法：
        with SearchDeviceCANFD("can0", 1000000, 5000000) as s:
            info = s.search()
    """

    def __init__(self, channel="can0",
                 bitrate: int = BITRATE_DEFAULT,
                 dbitrate: int = DBITRATE_DEFAULT,
                 canfd_device: str = "auto",
                 use_extended: bool = False,
                 frame_interval: float = FRAME_INTERVAL,
                 timeout: float = REPLY_TIMEOUT,
                 auto_setup: bool = True,
                 force_setup: bool = False,
                 reset_retry: bool = True,
                 verbose: bool = True):
        """
        channel        : socketcan 时为接口名 "can0"（写成纯数字 "0" 会被补成
                         "can0"）；libcanbus 时为设备号 0、"0" 或
                         "设备号:通道号"（如 "0:1"）
        bitrate        : 仲裁段波特率，须与设备一致（设备侧 0x35/0x08）
        dbitrate       : 数据段波特率（设备侧 0x35/0x09）
        canfd_device   : "auto" / "socketcan" / "libcanbus"
                         auto → 通道名在 /sys/class/net 下存在就走 socketcan，
                                否则有 libcanbus.so 就走厂商库
        use_extended   : True 用 29 位扩展帧搜索，False 用 11 位标准帧
        frame_interval : 每帧通讯间隔（秒），默认 2 ms；<=0 表示不节流
        timeout        : 单帧应答等待上限（秒）
        auto_setup     : socketcan 且接口没 up 时自动 `sudo ip link` 拉起（默认开）；
                         已经 up 则跳过，不打扰、也不改动现有配置
        force_setup    : 即使接口已经 up 也强制 down→重配→up（换波特率时用）
        reset_retry    : socketcan 下一帧应答都没收到时，自动 down→重配→up 一次
                         再搜一遍（默认开）；force_setup / auto_setup=False 时不生效
        verbose        : 打印搜索过程
        """
        self.channel = str(channel)
        self.bitrate = int(bitrate)
        self.dbitrate = int(dbitrate)
        self.use_extended = bool(use_extended)
        self.frame_interval = max(0.0, float(frame_interval))
        self.timeout = float(timeout)
        self.auto_setup = auto_setup
        self.force_setup = force_setup
        self.reset_retry = bool(reset_retry)
        self.verbose = verbose
        self.canfd_device = self._resolve_transport(canfd_device)

        self.comm: Optional[_Transport] = None
        self.is_open = False
        self.tx_count = 0                # 本次搜索发出的帧数
        self.rx_count = 0                # 本次搜索收到的有效应答帧数
        self.rx_ignored = 0              # 被丢弃的无关帧（总线上的其它流量）
        self.last_error_code: Optional[int] = None
        self._reset_done = False         # 接口重置只兜一次，避免反复要 sudo 密码
        self._tx_lock = threading.Lock()
        self._next_tx = 0.0              # 下一帧最早可发送时刻（perf_counter 轴）

    # ------------------------------------------------------------------ #
    # 传输选择
    # ------------------------------------------------------------------ #
    def _as_iface(self, channel: str) -> str:
        """socketcan 下把纯数字通道号补成接口名：'0' → 'can0'、'1' → 'can1'

        channel 这个参数在两种传输下含义不同（socketcan 是接口名 can0，
        libcanbus 是设备号 0），launch / 参数文件里只有一个字段，改
        canfd_device 时很容易忘了同步改它，这里兜一下，免得报「未找到网络接口 0」。
        """
        ch = str(channel).strip()
        if ch.isdigit():
            if self.verbose:
                print(f"[CANFD] socketcan 的通道名应为接口名，"
                      f"已把 channel={ch!r} 视作 can{ch}", flush=True)
            return f"can{ch}"
        return ch

    def _resolve_transport(self, kind: str) -> str:
        """auto 时挑一条真能跑 CAN FD 的路

        判定顺序：
            1) 通道名是网卡，且硬件支持 FD（mtu 已 72 或 maxmtu 到 72）→ socketcan
            2) 有 libcanbus.so → libcanbus
               （can0 存在但只是经典 CAN 适配器时走这条：pcan_usb 那类 maxmtu 16
                 的设备发不了 FD 帧，不能因为「网卡存在」就占用选择）
            3) 都不满足 → socketcan，让它去报「接口不存在 / 不支持 FD」并给提示

        auto 下通道名是纯数字（如 '0'）时按「libcanbus 的设备号」理解，优先厂商库；
        只有明确指定 socketcan、或最终回落到 socketcan 时，才把它当接口号补成 canN。
        """
        kind = (kind or "auto").lower()
        if kind == "socketcan":
            self.channel = self._as_iface(self.channel)
            return kind
        if kind == "libcanbus":
            return kind

        iface_exists = os.path.exists(f"/sys/class/net/{self.channel}")
        if iface_exists and iface_supports_fd(self.channel):
            return "socketcan"
        if libcanbus_present():
            if self.verbose and iface_exists:
                print(f"[CANFD] {self.channel} 不支持 CAN FD"
                      f"（mtu {iface_mtu(self.channel)}，经典 CAN 适配器），"
                      f"改用厂商库 libcanbus", flush=True)
            # auto 选中厂商库时，通道名要换成「设备号:通道号」的写法
            if not self.channel.replace(":", "").isdigit():
                self.channel = "0:0"
            return "libcanbus"
        self.channel = self._as_iface(self.channel)
        return "socketcan"

    def _make_transport(self) -> _Transport:
        if self.canfd_device == "libcanbus":
            dev, _, ch = self.channel.partition(":")
            try:
                dev_i, ch_i = int(dev or "0", 0), int(ch or "0", 0)
            except ValueError:           # 通道名不是数字（如 "can0"），按设备0 通道0
                dev_i, ch_i = 0, 0
                self.channel = "0:0"
            return LibCanBusTransport(
                device=dev_i, channel=ch_i,
                bitrate=self.bitrate, dbitrate=self.dbitrate,
                verbose=self.verbose)
        return SocketCANFDTransport(
            channel=self.channel, bitrate=self.bitrate, dbitrate=self.dbitrate,
            auto_setup=self.auto_setup, force_setup=self.force_setup,
            verbose=self.verbose)

    # ------------------------------------------------------------------ #
    # 帧间隔节流
    # ------------------------------------------------------------------ #
    def _pace(self):
        """阻塞到距上一帧至少 frame_interval 秒，然后预约下一帧的最早发送时刻。

        下一帧时刻按「上次预约时刻 + 间隔」推进，而不是「醒来时刻 + 间隔」——
        这样 time.sleep 的固有过冲不会逐帧累积，连发时平均间隔稳定在设定值。
        空闲很久之后走 else 分支把时间轴拉回当前，避免攒出一串零间隔帧。
        """
        if self.frame_interval <= 0:
            return
        with self._tx_lock:
            now = time.perf_counter()
            wait = self._next_tx - now
            if wait > 0:
                time.sleep(wait)
                self._next_tx += self.frame_interval
            else:
                self._next_tx = now + self.frame_interval

    # ------------------------------------------------------------------ #
    # 总线
    # ------------------------------------------------------------------ #
    def open(self) -> bool:
        """按选定的传输方式打开 CAN FD 通道"""
        if self.is_open:
            return True
        self.comm = self._make_transport()
        if not self.comm.open():
            self.comm = None
            return False
        self.is_open = True
        self._next_tx = time.perf_counter()
        if self.verbose:
            print(f"[CANFD] {self.comm.describe} 已打开"
                  f"（帧间隔 {self.frame_interval * 1000:.3f} ms，"
                  f"单帧负载 ≤ {MAX_PAYLOAD} 字节）", flush=True)
        return True

    def close(self):
        if self.comm is not None:
            self.comm.close()
        self.comm = None
        self.is_open = False

    def _reset_link(self) -> bool:
        """socketcan 下 down→重配→up 再重新打开总线；不适用/失败时返回 False

        接口看着是 up、时序也对，却一帧应答都收不到，通常是适配器自己卡住了
        （gs_usb 那类透明塑封 USB-CANFD 在上个进程没正常关闭总线后就这样），
        把接口重新激活一次即恢复。所以搜不到设备时先自动重置一次再搜一遍，
        省得用户手动敲 ip link down/up。
        只兜一次（_reset_done）：多次重置既救不回来，还要反复输 sudo 密码。

        跳过的情况：
            libcanbus       —— 不走内核接口，没有 ip link 可重置
            auto_setup=False —— 调用方明确要求不碰接口配置
            force_setup=True —— open() 时已经 down→重配→up 过，再来一遍没意义
        """
        if (self.canfd_device != "socketcan" or self._reset_done
                or not self.auto_setup or self.force_setup):
            return False
        self._reset_done = True
        print(f"[CANFD] {self.channel} 上收不到任何应答，先重置接口"
              f"（down → 重配 → up）再搜一次 …", flush=True)
        self.close()                     # 先关掉套接字，再让 ip link down 生效
        self.force_setup = True          # 让 open() 强制走一遍 down→重配→up
        try:
            if not self.open():
                return False
        finally:
            self.force_setup = False     # 恢复原值，不影响后续 open()
        time.sleep(LINK_RESET_SETTLE)    # 等适配器与设备稳定，太快发帧会丢
        self._drain()
        return True

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    # ------------------------------------------------------------------ #
    # 收发
    # ------------------------------------------------------------------ #
    @property
    def _discovery_id(self) -> int:
        return DISCOVERY_ID_EXT if self.use_extended else DISCOVERY_ID_STD

    @property
    def _reply_bit(self) -> int:
        return REPLY_BIT_EXT if self.use_extended else REPLY_BIT_STD

    def reply_id_of(self, device_id: int) -> int:
        """设备（请求）ID → 返回帧 ID：最高位置 1"""
        return device_id | self._reply_bit

    @staticmethod
    def _frame(mi: int, si: int, edl: int, data: bytes = b"",
               rts: int = RTS_REALTIME) -> bytes:
        """组数据域：byte0=RTS|MI, byte1=SI, byte2=EDL, byte3+=负载"""
        return bytes([((rts & 1) << 7) | (mi & 0x7F), si & 0xFF, edl & 0xFF]) + data

    def _drain(self):
        """清掉总线上残留的旧帧，避免把上一次的应答当成本次的"""
        while self.comm is not None:
            try:
                if self.comm.recv(0.0) is None:
                    return
                self.rx_ignored += 1
            except Exception:
                return

    def _send(self, can_id: int, data: bytes) -> bool:
        if not self.is_open or self.comm is None:
            return False
        try:
            self._pace()                 # ← 2 ms 帧间隔在此生效
            if not self.comm.send(can_id, data, self.use_extended):
                print(f"❌ 发送失败（传输层拒绝）: 0x{can_id:X} "
                      f"{bytes(data).hex(' ').upper()}", flush=True)
                return False
            self.tx_count += 1
            return True
        except Exception as e:
            # 总线上没有任何节点 ACK 时，控制器会无限重发，发送队列很快填满，
            # 这里表现为 "Transmit buffer full" —— 属于接线/供电问题，不是软件问题
            print(f"❌ 发送失败: {e}", flush=True)
            if "buffer" in str(e).lower():
                print("   发送队列已满：总线上没有节点应答 ACK。请检查 CANH/CANL "
                      "接线、120Ω 终端电阻与设备供电。", flush=True)
            return False

    def _wait_reply(self, mi: int, si: int,
                    device_id: Optional[int] = None) -> Optional[Tuple[int, bytes]]:
        """等一帧应答 → (应答帧 CAN ID, 剥掉 3 字节帧头的负载)

        过滤条件：CAN ID 最高位为 1（是返回帧）、回显的 MI/SI 与请求一致。
        device_id 给定时还要求 ID 精确等于该设备的返回 ID。
        设备返回 MI=0x4F 的错误码帧时打印错误并返回 None。
        """
        deadline = time.perf_counter() + self.timeout
        want = self.reply_id_of(device_id) if device_id is not None else None
        while True:
            remain = deadline - time.perf_counter()
            if remain <= 0:
                return None
            try:
                got = self.comm.recv(remain)
            except Exception:
                return None
            if got is None:
                return None
            arb, body = got
            # 只认返回帧（最高位为 1），把自己发出去的回显帧滤掉
            if not arb & self._reply_bit:
                continue
            if want is not None and arb != want:
                self.rx_ignored += 1
                continue
            if len(body) < HEADER_LEN:
                self.rx_ignored += 1
                continue
            rmi, rsi = body[0] & 0x7F, body[1]
            if rmi == MI_ERROR_CODE and rmi != mi:      # 设备回了错误码帧
                code = body[HEADER_LEN] if len(body) > HEADER_LEN else 0
                self.last_error_code = code
                names = [n for bit, n in ERROR_CODE_NAMES.items() if code & bit]
                print(f"⚠️ 设备返回错误码 0x{code:02X}"
                      f"（{'/'.join(names) or '未知'}）"
                      f" —— 请求 MI=0x{mi:02X} SI=0x{si:02X}", flush=True)
                self.rx_count += 1
                return None
            if rmi != mi or rsi != si:
                self.rx_ignored += 1
                continue
            self.rx_count += 1
            return arb, body[HEADER_LEN:]

    def _read(self, device_id: Optional[int], si: int, length: int,
              mi: int = MI_PRODUCT_INFO) -> Optional[Tuple[int, bytes]]:
        """读一个字段 → (应答帧 ID, 负载)；length ≤ 61，单帧一问一答

        device_id=None 时发到搜索 ID（0x7FF / 0x1FFFFFFF）。
        """
        can_id = self._discovery_id if device_id is None else device_id
        self._drain()
        if not self._send(can_id, self._frame(mi, si, length)):
            return None
        return self._wait_reply(mi, si, device_id)

    def _read_str(self, device_id: int, si: int, length: int,
                  mi: int = MI_PRODUCT_INFO) -> Optional[str]:
        """读定长字符串字段（UTF-8，尾部补 0）

        CAN FD 单帧带 61 字节，本模块的字段最长 32 字节，正常一帧读完；
        length 超过单帧上限时按 SI 偏移续读（SI 单字节，故对象最多 256 字节）。
        """
        buf = bytearray()
        off = 0
        while off < length and (si + off) <= 0xFF:
            n = min(MAX_PAYLOAD, length - off, 0x100 - (si + off))
            got = self._read(device_id, si + off, n, mi)
            if got is None:
                return self._decode(buf) if buf else None
            buf += got[1]
            if len(got[1]) < n:                  # 返回不足，认为读到末尾
                break
            off += n
        return self._decode(buf)

    def _read_u8(self, device_id: int, si: int, mi: int) -> Optional[int]:
        got = self._read(device_id, si, 1, mi)
        return got[1][0] if got and got[1] else None

    @staticmethod
    def _decode(buf: bytes) -> str:
        """定长字符串 → 去掉补 0 尾部的 str"""
        return bytes(buf).split(b"\x00")[0].decode("utf-8", "ignore").strip()

    # ------------------------------------------------------------------ #
    # 左右手 / 传感器类型（调试功能 A，只读）
    # ------------------------------------------------------------------ #
    def _hand_side(self, device_id: int) -> str:
        """左右手：优先 0x42/0x49 的枚举（不依赖产品信息字段偏移），
        读不到再回退产品信息字符串 0x41/0xF1。"""
        v = self._read_u8(device_id, SI_HAND_SIDE[0], MI_ENGINEERING)
        if v in HAND_SIDE_ENUM:
            return HAND_SIDE_ENUM[v]
        return self._read_str(device_id, *SI_HAND_SIDE_STR) or ""

    def _sensor_type(self, device_id: int) -> str:
        """传感器类型：优先读传感器自报的类型字符串 MI 0x31/SI 0x00（报文 31 00 20）

        例：31 00 20 → "TSSP_FULAI"（返回 32 字节，尾部补 0）。
        这是传感器自己报的名字，装了就有；读不到或为空才回退调试功能 A 的
        0x42/0x4A 枚举 —— 那是出厂配置项，实测有的整机没写过，读回 0x00 会
        误报「无传感器」，所以不能拿它当主判据。
        """
        name = self._read_str(device_id, SI_SENSOR_TYPE_STR[0],
                              SI_SENSOR_TYPE_STR[1], MI_SENSOR)
        if name:
            return name
        v = self._read_u8(device_id, SI_SENSOR_TYPE[0], MI_ENGINEERING)
        if v is None:
            return ""
        return SENSOR_TYPE_ENUM.get(v, f"未知(0x{v:02X})")

    # ------------------------------------------------------------------ #
    # 搜索
    # ------------------------------------------------------------------ #
    def discover(self) -> Optional[Tuple[int, bytes]]:
        """向搜索 ID 发一帧读请求，从应答帧的 CAN ID 反推设备 ID

        返回 (device_id, 探测帧拿到的负载) —— 负载即协议名称字段，
        正常应为 "HOP" + 补 0。没有设备应答时返回 None。
        """
        if self.verbose:
            print(f"[搜索] 向 0x{self._discovery_id:X} 发探测帧 "
                  f"{self._frame(MI_PRODUCT_INFO, PROBE_SI, PROBE_LEN).hex(' ').upper()}"
                  f"（读协议名称）...", flush=True)
        got = self._read(None, PROBE_SI, PROBE_LEN)
        if got is None:
            return None
        reply_id, payload = got
        # 设备用「自己的返回 ID」应答，去掉最高位就是它的请求 ID
        device_id = reply_id & ~self._reply_bit
        limit = EXT_ID_MAX if self.use_extended else STD_ID_MAX
        if not 0 <= device_id <= limit:
            print(f"⚠️ 应答帧 ID 0x{reply_id:X} 推不出合法设备 ID"
                  f"（应 ≤ 0x{limit:X}）—— 设备可能直接用搜索 ID 应答，"
                  f"请改用 search(device_id=...) 指定已知 ID", flush=True)
            return None
        if PROBE_EXPECT not in bytes(payload):
            print(f"⚠️ 0x{device_id:X} 的应答不含 \"HOP\"（收到 {bytes(payload)!r}），"
                  f"可能不是灵巧手设备或协议不同", flush=True)
        if self.verbose:
            print(f"[搜索] ✅ 发现设备：请求 ID 0x{device_id:X} / "
                  f"返回 ID 0x{reply_id:X}", flush=True)
        return device_id, bytes(payload)

    def search(self, device_id: Optional[int] = None) -> Optional[DeviceInfo]:
        """搜索本线路上的设备并读回身份信息；没搜到返回 None

        device_id=None  → 先用搜索 ID 广播探测，自动得到设备 ID
        device_id=0xNN  → 跳过探测，直接问这个已知 ID
        """
        if not self.is_open and not self.open():
            return None
        self.tx_count = self.rx_count = self.rx_ignored = 0
        self.last_error_code = None

        probe_payload = b""
        if device_id is None:
            # 重置接口失败时 self.comm 会变成 None，描述先存下来
            link_desc = self.comm.describe
            found = self.discover()
            # 一帧应答都没有：可能只是适配器卡住了，重置接口后再探一次
            if found is None and self.reset_retry and self._reset_link():
                found = self.discover()
            if found is None:
                if self.verbose:
                    print(f"[搜索] ❌ {link_desc} 上未发现设备。请检查：", flush=True)
                    print(f"   1) 仲裁/数据波特率是否与设备一致"
                          f"（当前 {self.bitrate}/{self.dbitrate}）", flush=True)
                    print("   2) 设备侧 can 接口类型是否为 fdcan"
                          "（配置项 0x35/0x06 应为 2）", flush=True)
                    print("   3) 接线、120Ω 终端电阻、设备是否上电", flush=True)
                    print("   4) 若设备写过配置项 0x35/0x0A「屏蔽返回帧」，"
                          "它不会应答任何指令", flush=True)
                    if self._reset_done:
                        print(f"   （已自动重置 {self.channel} 一次仍无应答，"
                              f"可再手动拔插一次适配器）", flush=True)
                return None
            device_id, probe_payload = found

        info: Dict[str, str] = {}
        for key, label, si, length in INFO_FIELDS:
            # 探测帧已经把协议名称整个字段读回来了，不重复发
            if key == "protocol" and probe_payload:
                info[key] = self._decode(probe_payload)
                continue
            val = self._read_str(device_id, si, length)
            info[key] = val if val is not None else ""
            if not info[key] and self.verbose:
                print(f"⚠️ {label}（SI 0x{si:02X}）读取失败或为空", flush=True)

        return DeviceInfo(
            model=info["model"], voltage_range=info["voltage_range"],
            device_uid=info["device_uid"], protocol=info["protocol"],
            protocol_version=info["protocol_version"],
            hw_interface=info["hw_interface"], hw_control=info["hw_control"],
            app_version=info["app_version"], build_time=info["build_time"],
            supported_proto=info["supported_proto"],
            hand_side=self._hand_side(device_id),
            sensor_type=self._sensor_type(device_id),
            device_id=device_id, reply_id=self.reply_id_of(device_id),
            transport=self.canfd_device, channel=self.channel,
            bitrate=self.bitrate, dbitrate=self.dbitrate,
            frame_interval=self.frame_interval,
            tx_count=self.tx_count, rx_count=self.rx_count,
            extended=self.use_extended,
        )


# ============================================================================
# 命令行
# ============================================================================
def main(argv: List[str]) -> int:
    if "--help" in argv or "-h" in argv:
        print(__doc__, flush=True)
        return 0

    args = [a for a in argv if not a.startswith("--")]
    flags = {a.split("=")[0] for a in argv if a.startswith("--")}
    opts = dict(a[2:].split("=", 1) for a in argv if a.startswith("--") and "=" in a)

    canfd_device = ("libcanbus" if "--libcanbus" in flags else
                 "socketcan" if "--socketcan" in flags else "auto")
    default_ch = "0" if canfd_device == "libcanbus" else "can0"
    channel = args[0] if args else os.environ.get("CANFD_CHANNEL", default_ch)
    bitrate = int(args[1]) if len(args) > 1 else int(
        os.environ.get("CANFD_BITRATE", BITRATE_DEFAULT))
    dbitrate = int(args[2]) if len(args) > 2 else int(
        os.environ.get("CANFD_DBITRATE", DBITRATE_DEFAULT))
    dev_id = int(opts["id"], 0) if "id" in opts else None

    with SearchDeviceCANFD("0", bitrate, dbitrate,
                           canfd_device="auto",
                           use_extended="--ext" in flags,
                           auto_setup="--no-setup" not in flags,
                           force_setup="--setup" in flags,
                           reset_retry="--no-reset" not in flags,
                           verbose="--quiet" not in flags) as s:
        if not s.is_open:
            return 2
        info = s.search(dev_id)
        if info is None:
            return 1
        print("=" * 56, flush=True)
        print(info.report_full(), flush=True)
        print("=" * 56, flush=True)
        if s.rx_ignored and "--quiet" not in flags:
            print(f"（另丢弃 {s.rx_ignored} 帧与本次请求无关的总线流量）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
