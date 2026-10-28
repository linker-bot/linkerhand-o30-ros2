#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''web_control 启动文件 (通用灵巧手):
    ros2 launch web_control web_control.launch.py
    ros2 launch web_control web_control.launch.py hand:=right
    ros2 launch web_control web_control.launch.py hand:=right model:=O30

关节名 / 数量 / 映射均自适应 /cb_<hand>_hand_state 与 URDF, 无需额外配置。
端口无需指定: 节点从 8090 起自动探测, 被占用则自动顺延到下一个空闲端口。
URDF 根目录由本启动文件的 urdf_path 参数配置, 默认指向 web_control 包内置的
urdf 目录 (即"现在的目录"); 以后换型号 / 版本只需 urdf_path:=<绝对路径>。
'''
import os
import importlib.util

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_urdf_dir():
    '''默认 URDF 根目录 = web_control 包内置的 urdf 目录 (即"现在的目录")。

    用 importlib.util.find_spec 定位包目录 (不执行模块代码, 避免拉起 rclpy /
    sensor_msgs 等重依赖); 定位失败时返回空串, 由 web_control.py 回退到
    Path(__file__).parent / 'urdf'。以后换型号 / 版本时, 直接传
    urdf_path:=<绝对路径> 覆盖即可。
    '''
    # 将URDF放到`src\web_control\web_control\urdf\O30\left|right目录中
    try:
        spec = importlib.util.find_spec('web_control')
        if spec and spec.origin:
            pkg_dir = os.path.dirname(spec.origin)
            return os.path.join(pkg_dir, 'urdf')
    except (ImportError, ValueError, AttributeError):
        pass
    return ''


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hand', default_value='left',
                              description='手部类型 left | right (小写)'),
        DeclareLaunchArgument('model', default_value='auto',
                              description='URDF 型号子目录 (auto = 从信息话题匹配, 唯一时自动选用)'),
        DeclareLaunchArgument('urdf_path', default_value=_default_urdf_dir(), # /path/to/urdf
                              description=('URDF 根目录; 默认 = web_control 包内置 urdf 目录（现在的目录）; '
                                           '留空即此默认, 或填绝对路径随时更换, 例如 urdf_path:=/path/to/urdf')),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='监听地址'),
        Node(
            package='web_control',
            executable='web_control',
            name='web_control',
            output='screen',
            arguments=[
                '--hand', LaunchConfiguration('hand'),
                '--model', LaunchConfiguration('model'),
                '--urdf-path', LaunchConfiguration('urdf_path'),
                '--host', LaunchConfiguration('host'),
            ],
        ),
    ])
