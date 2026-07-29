<img src="resource/logo.png" width="800">

# LinkerHand 灵巧手 ROS2 SDK For O30

# 1. 概述

LinkerHand 灵巧手 ROS2 SDK 是灵心巧手(北京)科技有限公司开发，用于 O30 等 LinkerHand 灵巧手的驱动软件和功能示例源码，可用于真机与仿真器使用。

O30 为 **20 自由度** 灵巧手，采用 **HOP (Hand Object Protocol) v0.0.3** CANFD 协议、**11 位标准帧** 通信，支持两种 CANFD 通信后端（金属 CANFD 盒 / 透明塑封 USB-CANFD 设备），并可选配指尖阵列式触觉传感器。

| Name | Version | Link |
| --- | --- | --- |
| Python SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue?style=flat-square&logo=python&logoColor=white) ![Ubuntu 22.04+](https://img.shields.io/badge/OS-Ubuntu%2022.04%2B-E95420?style=flat-square&logo=ubuntu&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linker-hand-O30-ros2.git) |
| ROS2 SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python&logoColor=white) ![Ubuntu 22.04](https://img.shields.io/badge/OS-Ubuntu%2022.04-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-00B3E6?style=flat-square&logo=ros) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linker-hand-O30-ros2.git) |

# 2. 警告

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。
2. 执行动作前请务必进行安全评估，以防止发生碰撞。
3. 请保护好灵巧手，通电自检期间(约 5 秒)手指会小幅运动，请勿触碰。

# 3. 版本说明

**V1.0.0**

1. 支持 O30 版 Linker Hand ROS2 驱动（HOP v0.0.3 协议）。
2. 支持 `libcanbus`（金属 CANFD 盒）与 `socketcan`（透明塑封 USB-CANFD）两种通信后端。
3. GUI 控制界面，带有手指舞、数字手势等预设动作。
4. 可选指尖触觉传感器数据发布。

# 4. O30 关节说明

['拇指横滚', '拇指航向', '食指航向', '中指航向', '无名指航向', '小指航向', '拇指指根1', '食指指根1', '中指指根1', '无名指指根1', '小指指根1', '食指指根2', '中指指根2', '无名指指根2', '小指指根2', '拇指指尖', '食指指尖', '中指指尖', '无名指指尖', '小指指尖']

['thumb_cmc_roll', 'thumb_cmc_yaw', 'index_mcp_roll', 'middle_mcp_roll', 'ring_mcp_roll', 'pinky_mcp_roll', 'thumb_mcp', 'index_mcp_pitch', 'middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch', 'index_pip', 'middle_pip', 'ring_pip', 'pinky_pip', 'thumb_ip', 'index_dip', 'middle_dip', 'ring_dip', 'pinky_dip']

关节最小弧度值:[0.000, 0.000, -0.400, -0.380, -0.280, -0.280, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000],

关节最大弧度值：[0.611, 2.094, 0.037, 0.054, 0.188, 0.281, 1.713, 1.729, 1.884, 1.963, 1.850, 1.661, 1.707, 1.662, 1.703, 1.733, 1.763, 1.586, 1.675, 1.733],


O30 共 **20 个有效电机**，关节按「类型分组」排列（横滚 → 航向 → 指根1 → 指根2 → 指尖），组内手指顺序固定为 **拇指 → 食指 → 中指 → 无名指 → 小指**。控制/状态话题中的 `position` 数组长度为 20，顺序与下表完全一致，取值范围 **0 ~ 255**（单字节）。

| 序号 | 中文名称 | 英文名称 | 说明 |
| --- | --- | --- | --- |
| 0  | 拇指横滚 | thumb_roll   | 横滚组（仅拇指有横滚电机） |
| 1  | 拇指航向 | thumb_yaw    | 航向组 |
| 2  | 食指航向 | index_yaw    | 航向组 |
| 3  | 中指航向 | middle_yaw   | 航向组 |
| 4  | 无名指航向 | ring_yaw   | 航向组 |
| 5  | 小指航向 | little_yaw   | 航向组 |
| 6  | 拇指指根1 | thumb_root1  | 指根1(掌指)组 |
| 7  | 食指指根1 | index_root1  | 指根1组 |
| 8  | 中指指根1 | middle_root1 | 指根1组 |
| 9  | 无名指指根1 | ring_root1 | 指根1组 |
| 10 | 小指指根1 | little_root1 | 指根1组 |
| 11 | 食指指根2 | index_root2  | 指根2组（该组无拇指电机） |
| 12 | 中指指根2 | middle_root2 | 指根2组 |
| 13 | 无名指指根2 | ring_root2 | 指根2组 |
| 14 | 小指指根2 | little_root2 | 指根2组 |
| 15 | 拇指指尖 | thumb_tip    | 指尖组 |
| 16 | 食指指尖 | index_tip    | 指尖组 |
| 17 | 中指指尖 | middle_tip   | 指尖组 |
| 18 | 无名指指尖 | ring_tip   | 指尖组 |
| 19 | 小指指尖 | little_tip   | 指尖组 |

> 位置语义：一般 `200` 附近为「弯曲」端、`0` 附近为「伸直」端，静止值约 `25~37`（以实测 `/cb_*_hand_state` 为准，各关节方向以实物标定为准）。

# 5. 准备工作 (Ubuntu 22.04+)

## 5.1 系统与硬件需求

* 操作系统：Ubuntu 22.04+
* ROS2 版本：Humble（或对应发行版）
* Python 版本：3.10+
* 硬件：amd64_x86 / arm64，配备 USB-CANFD 设备

O30 支持以下两类 CANFD 设备，对应不同的通信后端 `comm_type`：

| CANFD 设备 | comm_type | 说明 |
| --- | --- | --- |
| 蓝色 / 黑色金属 CANFD 盒 | `libcanbus` | 使用厂商私有库 `libcanbus.so`，需按 5.2 解压安装 |
| 透明塑封 USB-CANFD 设备 | `socketcan` | 内核原生 SocketCAN（`can0` + python-can），无需安装驱动 |

## 5.2 金属 CANFD 盒配置 (comm_type = libcanbus)

将对应架构的 `libcanbus` 库解压到 `/usr/local/lib/` 目录下：

```bash
$ tar -xvf "libcanbus(ubuntu22).tar" -C /usr/local/lib/
```

配置环境变量：

```bash
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

库文件说明（`libcanbus.a` 为静态库，`libcanbus.so` 为共享库）：

| 库文件 | 适用编译器 / 环境 |
| --- | --- |
| libcanbus_arm       | arm-linux-gnueabihf-gcc |
| libcanbus_arm64     | aarch64-linux-gnu-gcc |
| libcanbus(ubuntu20) | gcc version 9.4.0 |
| libcanbus(ubuntu22) | gcc version 11.3.0 |

常见问题：

* 编译提示 `pthread` 相关错误，说明自带 libusb 库不匹配：`sudo apt-get install libusb-1.0-0-dev`，并删除 `/usr/local/lib` 下的 libusb 相关文件。
* 提示 `ludev` 错误：`sudo apt-get install libudev-dev`。
* 找不到 `cc1plus`：`sudo apt-get install --reinstall build-essential`。

设备权限：root 用户可直接读写 usbcan 设备。非 root 用户需修改 usbcan 模块操作权限，可将 `99-canfd.rules` 放到 `/etc/udev/rules.d/`，然后执行：

```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

重启系统生效。（同总线双手可参考 `99-double-canfd.rules`。）

## 5.3 透明塑封 USB-CANFD 设备配置 (comm_type = socketcan)

1. 确保 type-c 接口下方的开关拨到 **Linux 模式**（若 `lsusb` 显示为 `STM32 Virtual ComPort`，说明仍是串口模式）。
2. 重新插拔 USB，确认接口枚举为原生 CAN：

```bash
$ ip -br link show type can
```

3. SDK 默认 `auto_setup=True`，会自动执行 `sudo ip link` 拉起 `can0` 接口（仲裁 1Mbps / 数据段 5Mbps）。因此运行需具备 `sudo` 权限；也可提前手动配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on
sudo ip link set can0 up
```

> 注意：波特率必须与灵巧手一致（默认仲裁 1Mbps、数据段 5Mbps）。若接口状态出现 `BUS-OFF` / `ERROR-PASSIVE`，通常是波特率不匹配、接线或终端电阻问题。

## 5.4 下载

```bash
$ mkdir -p Linker_Hand_O30_ROS2_SDK/src    # 创建工程目录
$ cd Linker_Hand_O30_ROS2_SDK/src
$ git clone https://github.com/linker-bot/linkerhand-o30-ros2.git
```

## 5.5 安装依赖与编译

```bash
$ cd Linker_Hand_O30_ROS2_SDK/src
$ pip install -r requirements.txt    # 安装所需依赖(含 python-can)
$ cd Linker_Hand_O30_ROS2_SDK       # 回到工程根目录
$ colcon build --symlink-install     # 编译和构建 ROS 包
```

# 6. 启动 SDK

编辑 `src/linker_hand_o30_ros2_sdk/launch/linker_hand_o30.launch.py`，按下表配置参数：

| 参数 | 取值 | 说明 |
| --- | --- | --- |
| `hand_type`    | `left` \| `right` | 左右手（小写）。同总线双手需各自配置不同帧 ID |
| `hand_joint`   | `O30` | 型号（大写） |
| `is_touch`     | `True` \| `False` | 是否装配指尖触觉传感器 |
| `canfd_device` | `0` \| `1` | **仅 libcanbus**：CANFD 设备编号，先插为 0，后插为 1；单设备填 0 |
| `comm_type`    | `libcanbus` \| `socketcan` | 通信后端：金属盒填 `libcanbus`，透明塑封 USB-CANFD 填 `socketcan` |
| `channel`      | `can0` | **仅 socketcan**：SocketCAN 接口名 |
| `bitrate`      | `1000000` | **仅 socketcan**：仲裁段波特率（须与灵巧手一致） |
| `dbitrate`     | `5000000` | **仅 socketcan**：数据段波特率 |
| `auto_setup`   | `True` \| `False` | **仅 socketcan**：`True` 时自动 `sudo ip link` 拉起接口 |

启动：

```bash
$ cd Linker_Hand_O30_ROS2_SDK
$ source ./install/setup.bash
$ ros2 launch linker_hand_o30_ros2_sdk linker_hand_o30.launch.py
```

显示以下信息并读到设备产品信息表则连接成功（初始化后会通电自检约 5 秒，请勿触碰）：

```text
产品型号     O30
供电电压范围 12V~24V
设备唯一标识 2A00370...
协议名称     HOP
协议版本     0.0.2
控制板版本   V1.x.x
编译时间     2026-05-xx xx:xx:50
支持协议     CAN,UART
左右手       RIGHT
正在初始化 O30-RIGHT 设备，请稍候...
✅ 有效关节 20 个
✅ O30-RIGHT 设备初始化成功
✅ Topics话题初始化完毕
```

## 6.1 启动 GUI 控制 O30 灵巧手

启动 SDK 后，新开一个终端启动 GUI。

> 注：GUI 需要图形界面，不能通过纯 SSH 远程连接开启，请使用带图形界面的终端或 X11 转发。

编辑 `src/gui_control/launch/gui_control.launch.py`，按参数说明配置左手 / 右手、是否有触觉等，然后启动：

```bash
$ cd Linker_Hand_O30_ROS2_SDK
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```

GUI 提供 20 个关节滑块，以及「握拳 / 张开 / 数字手势(壹~捌) / 赞」等预设动作按钮。

<img src="resource/gui.png" width="550">

# 7. Topic 说明

SDK 按 `hand_type` 动态生成话题名（下表以左手 `left` 为例，右手将 `left` 替换为 `right`）。触觉相关话题仅在 `is_touch=True` 时创建。

| Topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/cb_left_hand_control_cmd`       | `sensor_msgs/JointState` | 订阅 | 左手控制命令 |
| `/cb_left_hand_state`             | `sensor_msgs/JointState` | 发布 | 左手实时关节状态 |
| `/cb_left_hand_info`              | `std_msgs/String`        | 发布 | 左手温度/电流/错误等信息 |
| `/cb_left_hand_matrix_touch`      | `std_msgs/String`        | 发布 | 左手指尖压力传感器矩阵（`is_touch=True`） |
| `/cb_left_hand_matrix_touch_mass` | `std_msgs/String`        | 发布 | 左手指尖压力传感器和值（`is_touch=True`） |
| `/cb_hand_setting_cmd`            | `std_msgs/String`        | 订阅 | 设置命令（如设置速度） |

查看话题列表：

```bash
$ source ./install/setup.bash
$ ros2 topic list
```

## 7.1 控制命令 `/cb_*_hand_control_cmd`

控制话题使用 `sensor_msgs/JointState`：

* `position`：20 个目标位置，顺序见 [第 4 节](#4-o30-关节说明)，取值 `0 ~ 255`。
* `velocity`：20 个目标速度（可选），取值 `0 ~ 255`。速度即便为 0 也不会停止，`0` 映射电机速度 8000，`255` 映射 12000。
* `effort`：暂不支持扭矩设置，可留空或填 0。

命令行示例（右手全部关节移动到中位 80）：

```bash
$ ros2 topic pub --once /cb_right_hand_control_cmd sensor_msgs/msg/JointState \
"{position: [80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80]}"
```

## 7.2 状态反馈 `/cb_*_hand_state`

`sensor_msgs/JointState`，`name` 为 20 个关节英文名，`position` 为实时位置（`0 ~ 255`）。

```bash
$ ros2 topic echo /cb_right_hand_state
```

## 7.3 设置命令 `/cb_hand_setting_cmd`

`std_msgs/String`，内容为 JSON 字符串。当前支持设置速度：

```bash
$ ros2 topic pub --once /cb_hand_setting_cmd std_msgs/msg/String \
'{data: "{\"setting_cmd\": \"set_speed\", \"params\": {\"hand_type\": \"right\", \"speed\": [255]}}"}'
```

> `speed` 为列表；若长度不等于 20，将以首元素广播到全部 20 个关节。扭矩设置(`set_max_torque_limits`)暂不支持。

# 8. 常见问题 (FAQ)

* **未找到 CANFD 设备 / 打开设备失败**（libcanbus）：确认已按 5.2 安装 `libcanbus.so` 并设置 `LD_LIBRARY_PATH`，检查设备权限（udev 规则）。
* **未找到网络接口 can0**（socketcan）：确认 type-c 下方开关拨到 Linux 模式并重新插拔 USB，用 `ip -br link show type can` 确认接口出现。
* **总线 BUS-OFF / 设备无应答**（socketcan）：检查波特率是否与灵巧手一致（仲裁 1M / 数据段 5M）、接线与终端电阻。
* **设备初始化失败（设备唯一标识为空）**：检查硬件连接、`hand_type` 与 `comm_type` 配置是否与实际设备一致。
* **GUI 无法启动**：确认使用带图形界面的终端（非纯 SSH），依赖 `pyqt5`、`pyqtgraph` 已安装。
