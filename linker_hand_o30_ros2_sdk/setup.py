#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup



package_name = 'linker_hand_o30_ros2_sdk'

setup(
    name=package_name,
    version='3.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='linkerhand',
    maintainer_email='linkerhand@todo.todo',
    description='LinkerHand O30 二十自由度灵巧手 ROS 2 驱动（CANFD / HOP 协议）',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'linker_hand_o30_ros2_sdk = linker_hand_o30_ros2_sdk.linker_hand_o30:main',
        ],
    },
)
