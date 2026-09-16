#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_o30_ros2_sdk',
            executable='linker_hand_o30_ros2_sdk',
            name='linker_hand_o30_ros2_sdk',
            output='screen',
            parameters=[{
                'hand_type': 'right', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O30", # O30 字母为大写
                'is_touch': True, # 配置Linker Hand灵巧手是否有压力传感器 True | False
                'is_rad': False,  # 是否开启弧度state发布
                'canfd_device': 0, #蓝色 or 黑色CANFD盒 配置CANFD设备编号 0 | 1 先插入的设备为0，后插入的设备为1。单CANFD设置为0即可
                'comm_type': 'libcanbus', # libcanbus | socketcan。 蓝色 or 黑色CANFD盒设置为:libcanbus。透明塑封 USB-CANFD 设备设置为:socketcan 通信后端 libcanbus(厂商私有库) | socketcan(内核原生 can0 + python-can, 透明塑封 USB-CANFD 设备)
                # 以下四项仅 comm_type='socketcan' 时生效：
                'channel': 'can0',   # 透明塑封 USB-CANFD 设备 socketcan 接口名
                'bitrate': 1000000,  # 仲裁段波特率 (须与灵巧手一致)
                'dbitrate': 5000000, # 数据段波特率
                'auto_setup': True,  # True 时自动 sudo ip link 拉起接口
                # ---- 上电安全 ----
                'auto_init_pose': True,  # True: 校验通过后自动摆到 init_pose；False: 只连接/校验/使能，不动
                'self_test_delay': 5.0,  # 上电自检等待秒数（摆位前）
                'strict_device_check': True,  # 产品型号/左右手/协议名称与配置不符时拒绝启动
                'ignore_joint_faults': False, # True 时执行器离线/异常也照样启动（危险，仅排障用）
                # ---- 控制输入 ----
                'joint_limit_min': [0] * 20,   # 各关节位置下限（越界命令截断到此，防止 &0xFF 回绕）
                'joint_limit_max': [255] * 20, # 各关节位置上限
                # ---- deadman（命令流看门狗，收到首条命令后才开始计时）----
                'cmd_timeout': 0.0,       # 秒；<=0 关闭。上层失联超过该时间即执行 deadman_action
                'deadman_action': 'hold', # hold(保持) | open(伸直五指) | disable(失能)
                # ---- 频率与诊断 ----
                'state_rate': 30.0,       # 状态发布/命令下发频率 Hz
                'touch_rate': 30.0,       # 触觉发布频率 Hz（独立线程；实测值见 info 的 touch_hz）
                'touch_fingers': [1, 2, 3, 4, 5], # 要读的传感器 1拇指~5小指(6手掌本机无)；少读几指才能上高频
                'touch_publish_matrix': True,     # False 时只发合力话题，省一半 JSON 序列化
                'info_rate': 1.0,         # 诊断信息发布频率 Hz
                'publish_velocity': True, # 状态里带实时速度（多一次总线读）
                'publish_effort': True,   # 状态里带实时电流（多一次总线读）
                'temp_warn': 90,          # 过温告警阈值 °C（设备自报 90）
            }],
        ),
    ])