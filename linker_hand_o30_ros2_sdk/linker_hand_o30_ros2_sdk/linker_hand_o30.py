#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install

'''
from re import A
import rclpy,sys                                     # ROS2 Python接口库
import time
import numpy as np
from rclpy.node import Node                      # ROS2 节点类
from rclpy.clock import Clock
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState, PointCloud2, PointField
import time, json, threading
from tabulate import tabulate
from wcwidth import wcswidth
from .core.canfd.linker_hand_o30_control import LinkerHandO30Controller, NAMES_EN, NAMES_CN
from .utils.color_msg import ColorMsg

INIT_POSE = [33, 23, 96, 176, 212, 162, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
# 速度设置即便为0也不会停止，0映射电机速度8000,255映射电机速度12000
# INIT_VEL = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
INIT_VEL = [200] * 20
INIT_TORQUE = [200] * 20
class LinkerHand(Node):
    def __init__(self, name):
        super().__init__(name)
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'O30')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('canfd_device', 0)
        # 通信后端：'libcanbus'(默认, 厂商私有库) 或 'socketcan'(内核原生 can0 + python-can,
        # 用于透明塑封 USB-CANFD 设备)。后四个参数仅 socketcan 生效。
        self.declare_parameter('comm_type', 'libcanbus')
        self.declare_parameter('channel', 'can0')        # socketcan 接口名
        self.declare_parameter('bitrate', 1000000)       # 仲裁段波特率
        self.declare_parameter('dbitrate', 5000000)      # 数据段波特率
        self.declare_parameter('auto_setup', True)       # 自动 ip link 拉起接口

        # ros时间获取
        self.stamp_clock = Clock()
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.canfd_device = self.get_parameter('canfd_device').value
        self.comm_type = self.get_parameter('comm_type').value
        self.channel = self.get_parameter('channel').value
        self.bitrate = self.get_parameter('bitrate').value
        self.dbitrate = self.get_parameter('dbitrate').value
        self.auto_setup = self.get_parameter('auto_setup').value
        self.init_hand() # 初始化Linker Hand SDK
        # 有效关节数以控制器实测结果为准（self.linker_hand.num_joints）
        self.num_joints = self.linker_hand.num_joints
        self.last_pose = [-1] * self.num_joints
        self.last_vel = [-1] * self.num_joints
        self.last_effort = [-1] * self.num_joints
        time.sleep(0.5) # 等待SDK初始化完成
        self.init_topics() # 初始化ROS2话题
        self.run_thread = threading.Thread(target=self.run, daemon=True)
        self.run_thread.start()

    def init_hand(self):
        # 初始化Linker Hand SDK
        self.linker_hand = LinkerHandO30Controller(
            hand_type=self.hand_type, canfd_device=self.canfd_device,
            comm_type=self.comm_type, channel=self.channel,
            bitrate=self.bitrate, dbitrate=self.dbitrate,
            auto_setup=self.auto_setup)
        try:
            # 获取设备信息
            hand_info = self.linker_hand.get_device_info()
        except Exception as e:
            print(f"❌ 获取设备信息失败: {e}")
            hand_info = {}
        # 转换为列表格式
        table_data = [[key, value] for key, value in hand_info.items()]
        # 使用 plain 格式避免对齐问题
        print(tabulate(table_data, headers=[], tablefmt='plain', stralign='left'), flush=True)
        ColorMsg(msg=f"正在初始化 {hand_info['产品型号']}-{hand_info['左右手']}设备，请稍候...", color="yellow")
        time.sleep(5) # 等待设备自检
        # 设置初始速度为255
        self.linker_hand.set_target_velocity(INIT_VEL)
        time.sleep(0.1) # 等待设备响应
        # 初始化joint到初始位置
        self.linker_hand.set_target_position(INIT_POSE)
        time.sleep(0.1) # 等待设备响应
        if hand_info["设备唯一标识"] == None or hand_info["设备唯一标识"] == "":
            ColorMsg(msg="❌ 设备初始化失败，请检查硬件连接和配置参数是否正确", color="red")
            self.close()
        ColorMsg(msg=f"✅ {hand_info['产品型号']}-{hand_info['左右手']}设备初始化成功", color="green")
        self.linker_hand.set_target_torque(INIT_TORQUE)

    def init_topics(self):
        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        # 控制命令订阅
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        # joint state状态发布
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        # 判断是否存在触觉数据，如果存在则创建触觉数据发布器
        if self.is_touch == True:
            self.matrix_touch_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
            self.matrix_touch_mass_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
        # 其他信息发布
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        ColorMsg(msg=f"✅ Topics话题初始化完毕", color="green")

    def hand_control_cb(self, msg):
        self.last_pose = msg.position
        self.last_vel = msg.velocity
        self.last_effort = msg.effort
    
    def run(self):
        try:
            while rclpy.ok():
                if all(x == -1 for x in self.last_pose):
                    pass
                else:
                    self.linker_hand.set_target_position(self.last_pose)
                    self.last_pose = [-1] * self.num_joints
                    time.sleep(0.002)
                if all(x == -1 for x in self.last_vel):
                    pass
                else:
                    self.linker_hand.set_target_velocity(self.last_vel)
                    self.last_vel = [-1] * self.num_joints
                    time.sleep(0.002)
                hand_vel = self.linker_hand.get_current_velocity()
                hand_current = self.linker_hand.get_motor_current()
                # 获取手部状态
                hand_state = self.linker_hand.get_current_position()
                joint_state = self.joint_state_msg(hand_state,vel=hand_vel,effort=hand_current)
                self.hand_state_pub.publish(joint_state)
                
                
                time.sleep(0.008)  # 控制循环频率
        except Exception as e:
            print(f"❌ 发生错误: {e}")

    def joint_state_msg(self, pose,vel=[],effort=[]):
        joint_state = JointState()
        joint_state.header = Header()
        # 当前时间戳
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = NAMES_EN
        joint_state.position = [float(x) for x in pose]
        # 如果速度和力矩数据为空，则填充为0
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        if len(effort) > 1:
            joint_state.effort = [float(x) for x in effort]
        else:
            joint_state.effort = [0.0] * len(pose)
        return joint_state
    
    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}",flush=True)
        try:
            if data["params"]["hand_type"] == "left":
                hand = self.linker_hand
                hand_left = True
            elif data["params"]["hand_type"] == "right":
                hand = self.linker_hand
                hand_right = True
            else:
                print("Please specify the hand part to be set",flush=True)
                return
            self.cmd_lock = True
            # Set maximum torque
            if data["setting_cmd"] == "set_max_torque_limits": # Set maximum torque
                ColorMsg(msg=f"暂不支持扭矩设置", color="yellow")
                #torque = list(data["params"]["torque"])
                #hand.set_torque(torque=torque)
                
            if data["setting_cmd"] == "set_speed": # Set speed
                if isinstance(data["params"]["speed"], list) == True:
                    speed = data["params"]["speed"]
                    if len(speed) != self.num_joints:
                        speed = [speed[0]] * self.num_joints
                        print(f"设置速度{speed}",flush=True)
                        hand.set_target_velocity(speed)
                else:
                    ColorMsg(msg=f"Speed parameter error, speed must be a list", color="red")
            # if data["setting_cmd"] == "clear_faults": # Clear faults
            #     if hand_left == True and self.hand_joint == "L10" :
            #         ColorMsg(msg=f"L10 left hand cannot clear faults")
            #     elif hand_right == True and self.hand_joint == "L10" :
            #         ColorMsg(msg=f"L10 right hand cannot clear faults")
            #     else:
            #         hand.clear_faults()
            # if data["setting_cmd"] == "get_faults": # Get faults
            #     f = hand.get_fault()
            #     ColorMsg(msg=f"Get faults: {f}")
            # if data["setting_cmd"] == "electric_current": # Get current
            #     ColorMsg(msg=f"Get current: {hand.get_current()}")
            # if data["setting_cmd"] == "set_electric_current": # Set current
            #     if isinstance(data["params"]["current"], list) == True:
            #         hand.set_current(data["params"]["current"])
            
        except:
            print("命令参数错误")
            self.cmd_lock = False
        finally:
            self.cmd_lock = False
        
    def close(self):
        self.linker_hand.close()

        self.destroy_node()
        self.run_thread.join(timeout=2)  # 等待线程结束
        rclpy.shutdown()
# 自定义中文对齐
def chinese_len(text):
    return wcswidth(str(text))

def main(args=None):
    try:
        rclpy.init(args=args)
        node = LinkerHand("linker_hand_o30_ros2_sdk")
        
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
        node.close()
    except Exception as e:
        print(f"❌ 发生错误: {e}")
    finally:
        # node.close_can()         # 关闭 CAN 或其他硬件资源
        # node.destroy_node()      # 销毁 ROS 节点
        # rclpy.shutdown()         # 关闭 ROS
        ColorMsg(msg="程序已退出", color="yellow")