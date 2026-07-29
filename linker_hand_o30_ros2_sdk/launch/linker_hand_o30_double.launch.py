#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_o30_ros2_sdk',
            executable='linker_hand_o30_ros2_sdk',
            name='linker_hand_o30_ros2_sdk_left',
            output='screen',
            parameters=[{
                'hand_type': 'left', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O30", # O30I 字母为大写
                'is_touch': False, # 配置Linker Hand灵巧手是否有压力传感器 True | False
                'canfd_device': 0, #蓝色 or 黑色CANFD盒 配置CANFD设备编号 0 | 1 先插入的设备为0，后插入的设备为1。单CANFD设置为0即可
                'comm_type': 'socketcan', # libcanbus | socketcan 蓝色 or 黑色CANFD盒设置为:libcanbus。透明塑封 USB-CANFD 设备设置为:socketcan 通信后端 libcanbus(厂商私有库) | socketcan(内核原生 can0 + python-can, 透明塑封 USB-CANFD 设备)
                # 以下四项仅 comm_type='socketcan' 时生效：
                'channel': 'can0',   # 透明塑封 USB-CANFD 设备 socketcan 接口名
                'bitrate': 1000000,  # 仲裁段波特率 (须与灵巧手一致)
                'dbitrate': 5000000, # 数据段波特率
                'auto_setup': True,  # True 时自动 sudo ip link 拉起接口
            }],
        ),

        Node(
            package='linker_hand_o30_ros2_sdk',
            executable='linker_hand_o30_ros2_sdk',
            name='linker_hand_o30_ros2_sdk_right',
            output='screen',
            parameters=[{
                'hand_type': 'right', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O30", # O30I 字母为大写
                'is_touch': False, # 配置Linker Hand灵巧手是否有压力传感器 True | False
                'canfd_device': 0, #蓝色 or 黑色CANFD盒 配置CANFD设备编号 0 | 1 先插入的设备为0，后插入的设备为1。单CANFD设置为0即可
                'comm_type': 'socketcan', # libcanbus | socketcan 蓝色 or 黑色CANFD盒设置为:libcanbus。透明塑封 USB-CANFD 设备设置为:socketcan 通信后端 libcanbus(厂商私有库) | socketcan(内核原生 can0 + python-can, 透明塑封 USB-CANFD 设备)
                # 以下四项仅 comm_type='socketcan' 时生效：
                'channel': 'can1',   # 透明塑封 USB-CANFD 设备 socketcan 接口名
                'bitrate': 1000000,  # 仲裁段波特率 (须与灵巧手一致)
                'dbitrate': 5000000, # 数据段波特率
                'auto_setup': True,  # True 时自动 sudo ip link 拉起接口
            }],
        ),
    ])
