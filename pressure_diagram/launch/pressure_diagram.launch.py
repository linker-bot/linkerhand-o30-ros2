#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # auto: 按当前存在的 /cb_*_hand_matrix_touch_mass 话题自动选左右手；
    # 也可显式写 left / right
    hand_type = LaunchConfiguration('hand_type')

    return LaunchDescription([
        DeclareLaunchArgument('hand_type', default_value='auto',
                              description='auto | left | right'),
        Node(
            package='pressure_diagram',
            executable='pressure_diagram',
            name='pressure_diagram_node',
            output='screen',
            parameters=[{
                'hand_type': hand_type,
            }],
        ),

    ])
