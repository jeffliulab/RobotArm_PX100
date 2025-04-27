#!/usr/bin/env python3

"""
PX100机械臂控制程序 v1.1
======================

本程序提供了PX100机械臂的综合控制系统，同时支持命令行和图形界面两种控制方式：
1. 命令行控制 - 通过命令行直接控制机械臂
2. 图形界面控制 - 通过按钮直观地控制机械臂
3. 监听模式 - 监听外部话题，接收来自其他源的控制命令

功能特点:
--------
- 支持基本机械臂操作：夹爪开关、关节移动
- 直观的图形用户界面，按钮控制
- 灵活的命令映射系统，便于修改和扩展
- 实时显示关节位置状态
- 与外部系统集成，支持远程控制
- 模块化设计，易于维护和扩展

使用方法:
--------
1. 启动PX100仿真环境:
   ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=px100

2. 运行本控制程序:
   ros2 run px100_test simple_control

3. 控制方式:
   - 使用GUI按钮直接控制机械臂
   - 在命令行输入命令控制机械臂
   - 输入'listen'启动监听模式，接收外部命令

开发者说明:
---------
本程序使用ROS2 Action接口与PX100机械臂通信，通过JointTrajectory消息控制关节移动。
如需修改映射关系或添加新功能，请参考配置部分的注释说明。

作者: AI Assistant
日期: 2025-04-26
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import threading
import time
import sys
import signal
import json
import tkinter as tk
from tkinter import ttk
import queue

# =====================================================================
# 配置部分 - 修改此部分可以自定义命令映射和GUI外观
# =====================================================================
# 外部话题名称 - 用于监听模式
EXTERNAL_TOPIC = 'GUI'

# Click状态映射
CLICK_MAPPINGS = {
    1: "close",  # click为1时对应close命令
    0: "open"    # click为0时对应open命令
}

# Motion状态映射
MOTION_MAPPINGS = {
    "lifting": "up",        # lifting对应up命令
    "dropping": "down",     # dropping对应down命令
    "stationary": None      # stationary不执行动作
}

# 夹爪控制参数配置
GRIPPER_PARAMS = {
    "open": {"left": 0.5, "right": 0.5},  # 打开夹爪时左右手指的角度(弧度)
    "close": {"left": 0.0, "right": 0.0}  # 关闭夹爪时左右手指的角度(弧度)
}

# 关节移动参数配置
JOINT_PARAMS = {
    "up": {"joint": 1, "delta": -0.1},    # 向上移动肩关节，delta为负(弧度)
    "down": {"joint": 1, "delta": 0.1},   # 向下移动肩关节，delta为正(弧度)
    "left": {"joint": 0, "delta": 0.1},   # 向左旋转腰部，delta为正(弧度)
    "right": {"joint": 0, "delta": -0.1}  # 向右旋转腰部，delta为负(弧度)
}

# GUI窗口配置
GUI_CONFIG = {
    "title": "PX100机械臂控制器",
    "width": 600,
    "height": 450,
    "bg_color": "#f0f0f0",
    "button_width": 10,
    "button_height": 2,
    "button_font": ("Arial", 12),
    "status_font": ("Arial", 10),
    "label_font": ("Arial", 12, "bold")
}

# 按钮颜色配置
BUTTON_COLORS = {
    "up": "#a3d2ca",      # 浅绿色
    "down": "#5eaaa8",    # 深绿色
    "left": "#f05945",    # 红色
    "right": "#eb5e0b",   # 橙色
    "open": "#bbdfc8",    # 浅蓝色
    "close": "#8ac4d0",   # 深蓝色
    "listen": "#ffd369",  # 黄色
    "manual": "#f9f7f7",  # 白色
    "status": "#4b6584",  # 深蓝色
    "exit": "#fc5c65",    # 红色
    "disabled": "#cccccc" # 灰色
}
# =====================================================================
# 配置结束
# =====================================================================

class PX100Controller(Node):
    """
    PX100机械臂控制器类
    
    该类提供了对PX100机械臂的控制功能，包括关节移动、夹爪控制等，
    并可以通过命令行、GUI或外部话题进行控制。
    """
    
    def __init__(self, use_gui=True):
        """
        初始化控制器
        
        参数:
            use_gui: 是否使用GUI界面
        """
        super().__init__('px100_controller')
        
        # 初始化关节位置字典
        self.joint_positions = {}
        self.joint_ready = False
        
        # 创建关节状态订阅者
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/px100/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 创建Action客户端
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/px100/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/px100/gripper_controller/follow_joint_trajectory'
        )
        
        # 创建外部命令订阅者
        self.external_subscriber = self.create_subscription(
            String,
            EXTERNAL_TOPIC,
            self.external_command_callback,
            10
        )
        
        # 控制状态
        self.action_in_progress = False  # 是否有动作正在执行
        self.initialized = False         # 是否已初始化
        self.shutdown_requested = False  # 是否请求关闭
        self.listen_mode = False         # 是否处于监听模式
        self.command_queue = queue.Queue() # 命令队列
        
        # 设置信号处理器以确保干净的退出
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 命令映射表，将命令字符串映射到对应的方法
        self.command_map = {
            "open": lambda: self.control_fingers(**GRIPPER_PARAMS["open"]),
            "close": lambda: self.control_fingers(**GRIPPER_PARAMS["close"]),
            "up": lambda: self.move_arm_joint(**JOINT_PARAMS["up"]),
            "down": lambda: self.move_arm_joint(**JOINT_PARAMS["down"]),
            "left": lambda: self.move_arm_joint(**JOINT_PARAMS["left"]),
            "right": lambda: self.move_arm_joint(**JOINT_PARAMS["right"]),
            "status": self.print_joint_positions,
            "help": self.print_help,
            "listen": self.enable_listen_mode,
            "manual": self.disable_listen_mode,
            "exit": lambda: setattr(self, 'shutdown_requested', True)
        }
        
        # GUI相关
        self.use_gui = use_gui
        self.gui_window = None
        self.status_var = None
        self.log_text = None
        
        if self.use_gui:
            # 创建GUI线程
            self.gui_thread = threading.Thread(target=self.create_gui)
            self.gui_thread.daemon = True
            self.gui_thread.start()
    
    def signal_handler(self, sig, frame):
        """
        处理CTRL+C信号，确保程序正常退出
        
        参数:
            sig: 信号类型
            frame: 当前帧
        """
        print("\n收到中断信号，正在退出...")
        self.shutdown_requested = True
        self.listen_mode = False
        rclpy.shutdown()
    
    def joint_state_callback(self, msg):
        """
        关节状态回调函数，处理从机械臂接收到的关节状态信息
        
        参数:
            msg: JointState消息，包含关节名称和位置
        """
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos
        
        if not self.joint_ready and len(self.joint_positions) > 0:
            self.joint_ready = True
            self.get_logger().info('成功获取关节状态!')
            
            # 更新GUI状态
            if self.use_gui and self.status_var:
                self.status_var.set('已连接到PX100机械臂')
    
    def external_command_callback(self, msg):
        """
        外部命令回调函数，处理从外部话题接收到的控制命令
        
        参数:
            msg: String消息，包含JSON格式的命令数据
        """
        if not self.listen_mode:
            # 如果不在监听模式，记录但忽略外部命令
            self.get_logger().info(f"收到外部命令，但当前不在监听模式: {msg.data}")
            self.log_message(f"收到外部命令，但当前不在监听模式")
            return
            
        try:
            # 解析JSON数据
            self.get_logger().info(f"收到外部消息: {msg.data}")
            data = json.loads(msg.data)
            
            # 提取命令数据
            click_state = data.get("click", 0)
            motion_state = data.get("motion", "stationary")
            channel = data.get("channel", "1")
            
            log_msg = f"处理外部命令: click={click_state}, motion={motion_state}, channel={channel}"
            self.get_logger().info(log_msg)
            self.log_message(log_msg)
            
            # 获取click对应的命令
            click_command = CLICK_MAPPINGS.get(click_state)
            if click_command:
                log_msg = f"执行click映射命令: {click_command}"
                self.get_logger().info(log_msg)
                self.log_message(log_msg)
                self.command_queue.put(click_command)
            
            # 获取motion对应的命令
            motion_command = MOTION_MAPPINGS.get(motion_state)
            if motion_command:
                log_msg = f"执行motion映射命令: {motion_command}"
                self.get_logger().info(log_msg)
                self.log_message(log_msg)
                self.command_queue.put(motion_command)
            
        except json.JSONDecodeError:
            error_msg = f"无法解析外部命令: {msg.data}"
            self.get_logger().error(error_msg)
            self.log_message(error_msg, is_error=True)
        except Exception as e:
            error_msg = f"处理外部命令时出错: {str(e)}"
            self.get_logger().error(error_msg)
            self.log_message(error_msg, is_error=True)
    
    def print_joint_positions(self):
        """打印当前所有关节的位置信息"""
        positions_info = ['当前机械臂位置:']
        
        # 打印手臂关节
        arm_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle']
        for joint in arm_joints:
            if joint in self.joint_positions:
                pos_str = f'  {joint}: {self.joint_positions[joint]:.4f} rad'
                positions_info.append(pos_str)
                self.get_logger().info(pos_str)
        
        # 打印夹爪位置
        positions_info.append('当前夹爪位置:')
        if 'left_finger' in self.joint_positions and 'right_finger' in self.joint_positions:
            left_pos = self.joint_positions['left_finger']
            right_pos = self.joint_positions['right_finger']
            positions_info.append(f'  left_finger: {left_pos:.4f} rad')
            positions_info.append(f'  right_finger: {right_pos:.4f} rad')
            self.get_logger().info(f'  left_finger: {left_pos:.4f} rad')
            self.get_logger().info(f'  right_finger: {right_pos:.4f} rad')
        
        if 'gripper' in self.joint_positions:
            gripper_pos = self.joint_positions['gripper']
            positions_info.append(f'  gripper: {gripper_pos:.4f} m')
            self.get_logger().info(f'  gripper: {gripper_pos:.4f} m')
        
        # 更新GUI日志
        if self.use_gui:
            self.log_message('\n'.join(positions_info))
        
        return True  # 返回成功状态
    
    def print_help(self):
        """打印帮助信息，显示可用的命令列表"""
        help_text = [
            "\n===== PX100控制命令 =====",
            "  open   - 打开夹爪",
            "  close  - 关闭夹爪",
            "  up     - 肩关节向上移动",
            "  down   - 肩关节向下移动",
            "  left   - 腰部向左旋转",
            "  right  - 腰部向右旋转",
            "  status - 显示当前位置",
            "  help   - 显示此帮助信息",
            "  listen - 启动监听模式，接收外部命令",
            "  manual - 返回手动控制模式",
            "  exit   - 退出程序",
            "========================="
        ]
        
        # 打印到控制台
        for line in help_text:
            print(line)
        
        if self.listen_mode:
            print("当前模式: 监听模式 (接收外部命令)")
            print("\n当前映射配置:")
            print("  Click映射:")
            for click, command in CLICK_MAPPINGS.items():
                if command:
                    print(f"    {click} -> {command}")
            print("  Motion映射:")
            for motion, command in MOTION_MAPPINGS.items():
                if command:
                    print(f"    {motion} -> {command}")
        else:
            print("当前模式: 手动控制模式")
        
        # 更新GUI日志
        if self.use_gui:
            self.log_message('\n'.join(help_text))
            if self.listen_mode:
                self.log_message("当前模式: 监听模式 (接收外部命令)")
            else:
                self.log_message("当前模式: 手动控制模式")
        
        return True  # 返回成功状态
    



    def enable_listen_mode(self):
        """启用监听模式，接收来自外部的控制命令"""
        if self.listen_mode:
            print("已经处于监听模式")
            return True
                
        self.listen_mode = True
        
        msg = "\n========= 监听模式已启动 =========\n"
        msg += f"正在监听话题: {EXTERNAL_TOPIC}\n"
        msg += "在此模式下，机械臂将接收来自外部的命令\n"
        msg += "点击'手动模式'按钮或输入'manual'命令返回手动控制模式"
        
        print(msg)
        
        # 更新GUI状态
        if self.use_gui:
            self.log_message(msg)
            self.update_gui_buttons()
        
        return True

    def disable_listen_mode(self):
        """禁用监听模式，返回手动控制模式"""
        if not self.listen_mode:
            print("已经处于手动控制模式")
            return True
                
        self.listen_mode = False
        
        msg = "\n已返回手动控制模式\n"
        msg += "点击'监听模式'按钮或输入'listen'命令重新启动监听模式"
        
        print(msg)
        
        # 更新GUI状态
        if self.use_gui:
            self.log_message(msg)
            self.update_gui_buttons()
        
        return True




    def initialize(self):
        """
        初始化控制器，连接到机械臂和控制器
        
        返回值:
            bool: 初始化是否成功
        """
        print("正在连接PX100机械臂...")
        self.log_message("正在连接PX100机械臂...")
        
        # 等待关节状态
        timeout = 10  # 10秒超时
        start_time = time.time()
        
        while not self.joint_ready:
            if time.time() - start_time > timeout:
                error_msg = "错误: 无法获取关节状态，请检查机械臂是否已启动"
                print(error_msg)
                self.log_message(error_msg, is_error=True)
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 等待Action服务器
        print("正在连接控制器...")
        self.log_message("正在连接控制器...")
        
        arm_connected = self.arm_client.wait_for_server(timeout_sec=5.0)
        gripper_connected = self.gripper_client.wait_for_server(timeout_sec=5.0)
        
        if not arm_connected:
            error_msg = "错误: 无法连接到机械臂控制器"
            print(error_msg)
            self.log_message(error_msg, is_error=True)
            return False
        
        if not gripper_connected:
            error_msg = "错误: 无法连接到夹爪控制器"
            print(error_msg)
            self.log_message(error_msg, is_error=True)
            return False
        
        success_msg = "成功连接到PX100机械臂!"
        print(success_msg)
        self.log_message(success_msg)
        self.print_joint_positions()
        self.initialized = True
        
        # 更新GUI状态
        if self.use_gui and self.status_var:
            self.status_var.set('已连接到PX100机械臂')
        
        return True
    
    def move_arm_joint(self, joint, delta):
        """
        移动指定的机械臂关节
        
        参数:
            joint: 关节索引 (0:waist, 1:shoulder, 2:elbow, 3:wrist_angle)
            delta: 关节角度变化量 (弧度)
            
        返回值:
            bool: 操作是否成功
        """
        if self.action_in_progress:
            busy_msg = "有动作正在执行，请稍后再试"
            print(busy_msg)
            self.log_message(busy_msg)
            return False
        
        self.action_in_progress = True
        
        # 创建 Action 目标
        goal_msg = FollowJointTrajectory.Goal()
        
        # 创建轨迹
        traj = JointTrajectory()
        joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']
        traj.joint_names = joint_names
        
        # 获取当前位置
        current_positions = [
            self.joint_positions.get('waist', 0.0),
            self.joint_positions.get('shoulder', 0.0),
            self.joint_positions.get('elbow', 0.0),
            self.joint_positions.get('wrist_angle', 0.0)
        ]
        
        # 计算目标位置
        target_positions = current_positions.copy()
        target_positions[joint] += delta  # 修改指定关节位置
        
        # 创建起始点
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0, 0.0, 0.0, 0.0]
        start_point.accelerations = [0.0, 0.0, 0.0, 0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1秒
        
        # 创建目标点
        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        target_point.velocities = [0.0, 0.0, 0.0, 0.0]
        target_point.accelerations = [0.0, 0.0, 0.0, 0.0]
        target_point.time_from_start = Duration(sec=1, nanosec=0)  # 1秒
        
        # 添加轨迹点
        traj.points.append(start_point)
        traj.points.append(target_point)
        goal_msg.trajectory = traj
        
        # 发送目标
        joint_name = joint_names[joint]
        msg = f'发送{joint_name}关节移动命令: {delta:+.2f} rad'
        print(msg)
        self.log_message(msg)
        
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.arm_goal_response_callback)
        
        # 等待动作完成
        while self.action_in_progress and not self.shutdown_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True
    
    def arm_goal_response_callback(self, future):
        """
        机械臂目标响应回调函数
        
        参数:
            future: 包含响应结果的Future对象
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            msg = '移动命令被拒绝!'
            print(msg)
            self.log_message(msg, is_error=True)
            self.action_in_progress = False
            return
        
        msg = '移动命令被接受! 执行中...'
        print(msg)
        self.log_message(msg)
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_goal_result_callback)
    
    def arm_goal_result_callback(self, future):
        """
        机械臂目标结果回调函数
        
        参数:
            future: 包含执行结果的Future对象
        """
        try:
            result = future.result().result
            msg = '移动完成!'
            print(msg)
            self.log_message(msg)
            
            # 打印新位置
            self.print_joint_positions()
        except Exception as e:
            error_msg = f'处理移动结果时出错: {str(e)}'
            print(error_msg)
            self.log_message(error_msg, is_error=True)
        finally:
            self.action_in_progress = False
            
            # 更新GUI状态
            if self.use_gui and self.status_var:
                self.status_var.set('就绪 - 可以执行命令')
    
    def control_fingers(self, left, right):
        """
        控制夹爪左右手指位置
        
        参数:
            left: 左手指位置 (弧度)
            right: 右手指位置 (弧度)
            
        返回值:
            bool: 操作是否成功
        """
        if self.action_in_progress:
            busy_msg = "有动作正在执行，请稍后再试"
            print(busy_msg)
            self.log_message(busy_msg)
            return False
        
        self.action_in_progress = True
        
        # 创建 Action 目标
        goal_msg = FollowJointTrajectory.Goal()
        
        # 使用手指关节来控制
        traj = JointTrajectory()
        traj.joint_names = ['left_finger', 'right_finger']
        
        # 获取当前位置
        current_left = self.joint_positions.get('left_finger', 0.0)
        current_right = self.joint_positions.get('right_finger', 0.0)
        
        # 创建起始点
        start_point = JointTrajectoryPoint()
        start_point.positions = [current_left, current_right]
        start_point.velocities = [0.0, 0.0]
        start_point.accelerations = [0.0, 0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1秒
        
        # 创建目标点
        target_point = JointTrajectoryPoint()
        target_point.positions = [left, right]
        target_point.velocities = [0.0, 0.0]
        target_point.accelerations = [0.0, 0.0]
        target_point.time_from_start = Duration(sec=1, nanosec=0)  # 1秒
        
        # 添加轨迹点
        traj.points.append(start_point)
        traj.points.append(target_point)
        goal_msg.trajectory = traj
        
        # 发送目标
        msg = f'发送夹爪控制命令: {"打开" if left > 0 else "关闭"}'
        print(msg)
        self.log_message(msg)
        
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self.gripper_goal_response_callback)
        
        # 等待动作完成
        while self.action_in_progress and not self.shutdown_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True
    
    def gripper_goal_response_callback(self, future):
        """
        夹爪目标响应回调函数
        
        参数:
            future: 包含响应结果的Future对象
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            msg = '夹爪命令被拒绝!'
            print(msg)
            self.log_message(msg, is_error=True)
            self.action_in_progress = False
            return
        
        msg = '夹爪命令被接受! 执行中...'
        print(msg)
        self.log_message(msg)
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_goal_result_callback)
    
    def gripper_goal_result_callback(self, future):
        """
        夹爪目标结果回调函数
        
        参数:
            future: 包含执行结果的Future对象
        """
        try:
            result = future.result().result
            msg = '夹爪操作完成!'
            print(msg)
            self.log_message(msg)
            
            # 打印新位置
            self.print_joint_positions()
        except Exception as e:
            error_msg = f'处理夹爪结果时出错: {str(e)}'
            print(error_msg)
            self.log_message(error_msg, is_error=True)
        finally:
            self.action_in_progress = False
            
            # 更新GUI状态
            if self.use_gui and self.status_var:
                self.status_var.set('就绪 - 可以执行命令')
            
    def execute_command(self, command):
        """
        执行指定的命令
        
        参数:
            command: 命令字符串
            
        返回值:
            bool: 命令是否成功执行
        """
        if command in self.command_map:
            try:
                return self.command_map[command]()
            except Exception as e:
                error_msg = f"执行命令 '{command}' 时出错: {str(e)}"
                print(error_msg)
                self.log_message(error_msg, is_error=True)
                return False
        else:
            unknown_msg = f"未知命令: '{command}'"
            print(unknown_msg)
            self.log_message(unknown_msg)
            print("输入 'help' 查看可用命令")
            return False
    
    def log_message(self, message, is_error=False):
        """
        记录消息到GUI日志
        
        参数:
            message: 消息内容
            is_error: 是否为错误消息
        """
        if not self.use_gui or not self.log_text:
            return
            
        # 在GUI线程中执行，避免线程冲突
        if self.gui_window:
            self.gui_window.after(0, self._append_log, message, is_error)
    
    def _append_log(self, message, is_error):
        """
        实际添加日志到文本框的方法（由log_message在GUI线程中调用）
        """
        if not self.log_text:
            return
            
        self.log_text.config(state=tk.NORMAL)
        
        # 添加时间戳
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        
        # 根据消息类型设置标签
        tag = "error" if is_error else "normal"
        
        # 插入新消息
        self.log_text.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.log_text.insert(tk.END, f"{message}\n", tag)
        
        # 滚动到底部
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    




    def create_gui(self):
        """创建GUI界面"""
        # 创建主窗口
        self.gui_window = tk.Tk()
        self.gui_window.title(GUI_CONFIG["title"])
        self.gui_window.geometry(f"{GUI_CONFIG['width']}x{GUI_CONFIG['height']}")
        self.gui_window.configure(bg=GUI_CONFIG["bg_color"])
        
        # 创建状态变量
        self.status_var = tk.StringVar(value="正在初始化...")
        
        # 创建顶部框架 - 状态显示
        top_frame = ttk.Frame(self.gui_window, padding=5)
        top_frame.pack(fill=tk.X)
        
        status_label = ttk.Label(top_frame, text="状态:", font=GUI_CONFIG["label_font"])
        status_label.pack(side=tk.LEFT, padx=5)
        
        status_value = ttk.Label(top_frame, textvariable=self.status_var, font=GUI_CONFIG["status_font"])
        status_value.pack(side=tk.LEFT, padx=5)
        
        # 创建模式控制框架（顶部区域）
        mode_control_frame = ttk.LabelFrame(self.gui_window, text="模式控制", padding=10)
        mode_control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 监听模式和手动模式按钮使用同一个变量，确保互斥
        self.mode_var = tk.StringVar(value="手动模式")  # 默认为手动模式
        
        # 使用水平框架包含单选按钮
        mode_buttons_frame = ttk.Frame(mode_control_frame)
        mode_buttons_frame.pack(fill=tk.X, expand=True)
        
        # 监听模式按钮（单选按钮）
        self.listen_button = ttk.Radiobutton(
            mode_buttons_frame,
            text="监听模式",
            variable=self.mode_var,
            value="监听模式",
            command=lambda: self.execute_command("listen")
        )
        self.listen_button.pack(side=tk.LEFT, padx=20, pady=5, expand=True)
        
        # 手动模式按钮（单选按钮）
        self.manual_button = ttk.Radiobutton(
            mode_buttons_frame,
            text="手动模式",
            variable=self.mode_var,
            value="手动模式",
            command=lambda: self.execute_command("manual")
        )
        self.manual_button.pack(side=tk.RIGHT, padx=20, pady=5, expand=True)
        
        # 创建中间框架 - 控制按钮
        mid_frame = ttk.LabelFrame(self.gui_window, text="机械臂控制", padding=10)
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 创建左侧框架 - 移动控制
        movement_frame = ttk.Frame(mid_frame)
        movement_frame.pack(side=tk.LEFT, padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        move_label = ttk.Label(movement_frame, text="移动控制", font=GUI_CONFIG["status_font"])
        move_label.pack(pady=5)
        
        # 使用网格布局组织方向按钮
        button_frame = ttk.Frame(movement_frame)
        button_frame.pack(pady=5)
        
        # 上按钮
        self.up_button = tk.Button(
            button_frame,
            text="上移",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["up"],
            command=lambda: self.execute_command("up")
        )
        self.up_button.grid(row=0, column=1, padx=5, pady=5)
        
        # 左按钮
        self.left_button = tk.Button(
            button_frame,
            text="左转",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["left"],
            command=lambda: self.execute_command("left")
        )
        self.left_button.grid(row=1, column=0, padx=5, pady=5)
        
        # 右按钮
        self.right_button = tk.Button(
            button_frame,
            text="右转",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["right"],
            command=lambda: self.execute_command("right")
        )
        self.right_button.grid(row=1, column=2, padx=5, pady=5)
        
        # 下按钮
        self.down_button = tk.Button(
            button_frame,
            text="下移",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["down"],
            command=lambda: self.execute_command("down")
        )
        self.down_button.grid(row=2, column=1, padx=5, pady=5)
        
        # 创建右侧框架 - 夹爪控制
        control_frame = ttk.Frame(mid_frame)
        control_frame.pack(side=tk.RIGHT, padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        # 夹爪控制子框架
        gripper_frame = ttk.LabelFrame(control_frame, text="夹爪控制", padding=10)
        gripper_frame.pack(fill=tk.X, pady=5)
        
        # 打开夹爪按钮
        self.open_button = tk.Button(
            gripper_frame,
            text="打开夹爪",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["open"],
            command=lambda: self.execute_command("open")
        )
        self.open_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # 关闭夹爪按钮
        self.close_button = tk.Button(
            gripper_frame,
            text="关闭夹爪",
            width=GUI_CONFIG["button_width"],
            height=GUI_CONFIG["button_height"],
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["close"],
            command=lambda: self.execute_command("close")
        )
        self.close_button.pack(side=tk.RIGHT, padx=5, pady=5)
        
        # 创建状态控制区域（底部独立区域）
        status_control_frame = ttk.LabelFrame(self.gui_window, text="状态控制", padding=10)
        status_control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 查看状态按钮
        self.status_button = tk.Button(
            status_control_frame,
            text="查看状态",
            width=GUI_CONFIG["button_width"],
            height=2,
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["status"],
            fg="white",
            command=lambda: self.execute_command("status")
        )
        self.status_button.pack(side=tk.LEFT, padx=20, pady=5, expand=True)
        
        # 退出按钮
        self.exit_button = tk.Button(
            status_control_frame,
            text="退出程序",
            width=GUI_CONFIG["button_width"],
            height=2,
            font=GUI_CONFIG["button_font"],
            bg=BUTTON_COLORS["exit"],
            fg="white",
            command=lambda: self.execute_command("exit")
        )
        self.exit_button.pack(side=tk.RIGHT, padx=20, pady=5, expand=True)
        
        # 创建底部框架 - 日志显示
        bottom_frame = ttk.LabelFrame(self.gui_window, text="日志信息", padding=10)
        bottom_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建滚动条
        scrollbar = ttk.Scrollbar(bottom_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 创建文本框用于显示日志
        self.log_text = tk.Text(
            bottom_frame, 
            height=10,
            yscrollcommand=scrollbar.set,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=("Courier", 9)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.log_text.yview)
        
        # 定义文本标签
        self.log_text.tag_configure("error", foreground="red")
        self.log_text.tag_configure("normal", foreground="black")
        self.log_text.tag_configure("timestamp", foreground="blue")
        
        # 初始日志
        self.log_message("PX100控制器已启动")
        self.log_message("等待连接到机械臂...")
        
        # 设置窗口关闭事件
        self.gui_window.protocol("WM_DELETE_WINDOW", self.on_gui_close)
        
        # 更新按钮状态
        self.update_gui_buttons()
        
        # 启动GUI主循环
        self.gui_window.mainloop()








    def update_gui_buttons(self):
        """更新GUI按钮状态"""
        if not self.use_gui:
            return
            
        # 获取需要在监听模式下禁用的按钮列表
        control_buttons = [
            self.up_button,
            self.down_button,
            self.left_button,
            self.right_button,
            self.open_button,
            self.close_button,
            self.status_button
        ]
        
        # 针对监听模式设置按钮状态
        if self.listen_mode:
            # 设置为监听模式
            self.mode_var.set("监听模式")
            # 禁用所有控制按钮
            for button in control_buttons:
                button.config(state=tk.DISABLED, bg=BUTTON_COLORS["disabled"])
            # 更新状态文本
            self.status_var.set('监听模式 - 等待外部命令')
        else:
            # 设置为手动模式
            self.mode_var.set("手动模式")
            # 启用所有控制按钮
            for button in control_buttons:
                # 根据按钮类型设置正确的颜色
                if button == self.up_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["up"])
                elif button == self.down_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["down"])
                elif button == self.left_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["left"])
                elif button == self.right_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["right"])
                elif button == self.open_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["open"])
                elif button == self.close_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["close"])
                elif button == self.status_button:
                    button.config(state=tk.NORMAL, bg=BUTTON_COLORS["status"])
            # 更新状态文本
            self.status_var.set('手动模式 - 可以执行命令')





    def on_gui_close(self):
        """处理GUI窗口关闭事件"""
        self.shutdown_requested = True
        self.gui_window.destroy()
        print("GUI已关闭，正在退出程序...")
        
        # 如果在单独的线程中，需要确保主线程也结束
        rclpy.shutdown()

def main(args=None):
    """主函数，初始化并运行控制器"""
    rclpy.init(args=args)
    
    # 默认使用GUI
    use_gui = True
    
    # 检查命令行参数，是否要禁用GUI
    if args and "--no-gui" in args:
        use_gui = False
    
    # 创建控制器
    controller = PX100Controller(use_gui=use_gui)
    
    # 初始化控制器
    if not controller.initialize():
        controller.destroy_node()
        rclpy.shutdown()
        return
    
    # 显示帮助信息
    controller.print_help()
    
    # 如果没有使用GUI，进入命令行主循环
    if not use_gui:
        # 命令行主循环
        try:
            while rclpy.ok() and not controller.shutdown_requested:
                # 处理命令队列中的命令
                while not controller.command_queue.empty():
                    command = controller.command_queue.get()
                    controller.execute_command(command)
                
                # 处理ROS回调
                rclpy.spin_once(controller, timeout_sec=0.1)
                
                # 提示用户输入
                command = input("\n请输入命令 > ").strip().lower()
                
                # 执行命令
                controller.execute_command(command)
        
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
        finally:
            controller.destroy_node()
            rclpy.shutdown()
    else:
        # 如果使用GUI，启动命令处理线程
        cmd_thread = threading.Thread(target=process_commands, args=(controller,))
        cmd_thread.daemon = True
        cmd_thread.start()
        
        # 主线程等待程序结束
        try:
            while rclpy.ok() and not controller.shutdown_requested:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n收到中断信号，正在退出...")
        finally:
            controller.destroy_node()
            rclpy.shutdown()

def process_commands(controller):
    """处理命令队列的线程函数"""
    try:
        while rclpy.ok() and not controller.shutdown_requested:
            # 处理命令队列中的命令
            while not controller.command_queue.empty():
                command = controller.command_queue.get()
                controller.execute_command(command)
            
            # 处理ROS回调
            rclpy.spin_once(controller, timeout_sec=0.1)
            time.sleep(0.01)  # 减少CPU使用率
    except Exception as e:
        print(f"命令处理线程出错: {str(e)}")

if __name__ == '__main__':
    main(sys.argv)