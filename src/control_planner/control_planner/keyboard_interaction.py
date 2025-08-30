#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
import os
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
import atexit
import time
import threading

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64

class KeyboardInteracter(Node):
    def __init__(self):
        super().__init__('keyboard_interaction')

        # 发布者
        self.pub = self.create_publisher(String, 'keyboard_info', 10)
        self.thrust_rpm_pub = self.create_publisher(Float64, 'thrust_rpm', 10)
        self.rudder_angle_pub = self.create_publisher(Float64, 'rudder_joint_angle', 10)
        
        self.cmd = String()
        self.latlon_tuple = String()
        self.psi_d_float = String()
        self.keyboardBindings = ["w","a","s","d","e","r","f","x",'o','p','u','h']
        
        # 模式控制
        self.continuous_mode = False  # 默认为阻塞模式
        
        # 按键状态跟踪（仅在连续模式下使用）
        self.key_states = {}  # 记录按键状态
        self.key_printed = {}  # 记录按键是否已打印
        self.active_rpm_key = None  # 当前激活的RPM控制键
        self.active_rudder_key = None  # 当前激活的舵角控制键
        self.last_key_time = {}  # 记录每个键最后一次检测到的时间
        self.key_timeout = 0.2  # 按键超时时间（秒），超过此时间未检测到则认为已释放
        
        # 控制参数
        self.rpm_increment = 500.0  # 每秒增加的RPM
        self.rudder_increment =120.0  # 每秒增加的舵角（度）
        self.max_rpm = 3000.0
        self.min_rpm = -1000.0  # 允许反转
        self.max_rudder = 60.0  # 最大舵角±90度
        self.control_update_rate = 0.1  # 控制更新频率(秒)
        
        # 当前值
        self.current_rpm = 0.0
        self.current_rudder = 0.0
        
        # 参数
        self.declare_parameter('walk_vel', 0.5)
        self.declare_parameter('run_vel', 1.0)
        self.declare_parameter('yaw_rate', 1.0)
        self.declare_parameter('yaw_rate_run', 1.5)
        self.walk_vel_ = self.get_parameter('walk_vel').value
        self.run_vel_ = self.get_parameter('run_vel').value
        self.yaw_rate_ = self.get_parameter('yaw_rate').value
        self.yaw_rate_run_ = self.get_parameter('yaw_rate_run').value

        self.max_tv = self.walk_vel_
        self.max_rv = self.yaw_rate_

        self.speed = 0
        self.turn = 0
        self.t_get = time.time()

        self.latlon_acknowledged = False
        self.is_shutting_down = False  # 添加关闭标志

        # Save terminal settings
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        atexit.register(self.reset_terminal)

        # Ack订阅
        self.create_subscription(String, 'acknowledge', self.acknowledge_callback, 10)
        
        # 创建定时器，定期更新控制值（仅在连续模式下激活）
        self.control_timer = None

    def get_key(self):
        """非阻塞方式获取按键"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            return key
        return None

    def check_key_release(self):
        """检查按键是否已释放（基于超时）"""
        current_time = time.time()
        
        # 检查RPM键
        if self.active_rpm_key and self.active_rpm_key in self.last_key_time:
            if current_time - self.last_key_time[self.active_rpm_key] > self.key_timeout:
                print(f"\rKey {self.active_rpm_key} released - RPM control stopped")
                # 先保存键名，再设置为None
                released_key = self.active_rpm_key
                self.active_rpm_key = None
                if released_key in self.key_printed:
                    self.key_printed[released_key] = False
                
        # 检查舵角键
        if self.active_rudder_key and self.active_rudder_key in self.last_key_time:
            if current_time - self.last_key_time[self.active_rudder_key] > self.key_timeout:
                print(f"\rKey {self.active_rudder_key} released - Rudder control stopped")
                # 先保存键名，再设置为None
                released_key = self.active_rudder_key
                self.active_rudder_key = None
                if released_key in self.key_printed:
                    self.key_printed[released_key] = False

    def update_controls(self):
        """定期更新并发布控制命令（连续模式）"""
        if not self.continuous_mode or self.is_shutting_down:
            return
            
        # 检查按键释放
        self.check_key_release()
            
        rpm_changed = False
        rudder_changed = False
        
        # RPM控制 - 只在按键激活时更新
        if self.active_rpm_key == 'w':  # 增加转速
            self.current_rpm = min(self.current_rpm + self.rpm_increment * self.control_update_rate, self.max_rpm)
            rpm_changed = True
        elif self.active_rpm_key == 's':  # 减少转速
            self.current_rpm = max(self.current_rpm - self.rpm_increment * self.control_update_rate, self.min_rpm)
            rpm_changed = True
            
        # 舵角控制 - 只在按键激活时更新
        # 修正：左舵增加舵角，使用min限制上限；右舵减少舵角，使用max限制下限
        if self.active_rudder_key == 'a':  # 左舵
            self.current_rudder = min(self.current_rudder + self.rudder_increment * self.control_update_rate, self.max_rudder)
            rudder_changed = True
        elif self.active_rudder_key == 'd':  # 右舵
            self.current_rudder = max(self.current_rudder - self.rudder_increment * self.control_update_rate, -self.max_rudder)
            rudder_changed = True
        
        # 发布更新的值
        if rpm_changed and not self.is_shutting_down:
            rpm_msg = Float64()
            rpm_msg.data = self.current_rpm
            self.thrust_rpm_pub.publish(rpm_msg)
            
        if rudder_changed and not self.is_shutting_down:
            rudder_msg = Float64()
            rudder_msg.data = self.current_rudder
            self.rudder_angle_pub.publish(rudder_msg)
            
        # 显示当前状态
        if rpm_changed or rudder_changed:
            status = f"\rRPM: {self.current_rpm:.0f} | Rudder: {self.current_rudder:.1f}°"
            print(f"{status:<50}", end='', flush=True)

    def keyboard_loop(self):
        print("Reading from keyboard")
        print("Use wasd keys to control the robot")
        print("Press m to toggle continuous mode")
        print("Press l to enter latitude and longitude")
        print("Press q to quit")

        receive_latlon = False
        receive_psi_d = False

        while rclpy.ok():
            try:
                if not (receive_latlon or receive_psi_d):
                    if self.continuous_mode:
                        # 非阻塞模式
                        key = self.get_key()
                        
                        if key:
                            # 更新按键时间
                            self.last_key_time[key] = time.time()
                            
                            # 清除当前行
                            print("\r" + " " * 80 + "\r", end='')
                            
                            if key in ['w', 's']:
                                # 只在首次按下或切换键时打印
                                if self.active_rpm_key != key or not self.key_printed.get(key, False):
                                    self.active_rpm_key = key
                                    self.key_printed[key] = True
                                    print(f"Key {key} pressed (Continuous Mode) - RPM control active")
                                    # 清除其他RPM键的打印状态
                                    other_key = 's' if key == 'w' else 'w'
                                    if other_key in self.key_printed:
                                        self.key_printed[other_key] = False
                                    
                            elif key in ['a', 'd']:
                                # 只在首次按下或切换键时打印
                                if self.active_rudder_key != key or not self.key_printed.get(key, False):
                                    self.active_rudder_key = key
                                    self.key_printed[key] = True
                                    print(f"Key {key} pressed (Continuous Mode) - Rudder control active")
                                    # 清除其他舵角键的打印状态
                                    other_key = 'd' if key == 'a' else 'a'
                                    if other_key in self.key_printed:
                                        self.key_printed[other_key] = False
                                    
                            elif key == ' ':  # 空格键释放所有按键
                                self.active_rpm_key = None
                                self.active_rudder_key = None
                                # 重置当前值
                                self.current_rpm = 0.0
                                self.current_rudder = 0.0
                                # 清除所有打印状态
                                self.key_printed.clear()
                                print("All keys released - Values reset to 0")
                            elif key == 'm':  # 切换模式
                                self.continuous_mode = False
                                self.active_rpm_key = None
                                self.active_rudder_key = None
                                self.key_printed.clear()
                                if self.control_timer:
                                    self.control_timer.cancel()
                                # 恢复终端设置准备进入阻塞模式
                                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
                                print("Switched to Blocking Mode")
                            elif key == 'l':
                                receive_latlon = True
                                self.active_rpm_key = None
                                self.active_rudder_key = None
                                self.key_printed.clear()
                            elif key == 'k':
                                receive_psi_d = True
                                self.active_rpm_key = None
                                self.active_rudder_key = None
                                self.key_printed.clear()
                            elif key == 'q':
                                print("Now keyboard_interaction.py is getting exit...")
                                self.is_shutting_down = True
                                break
                        
                        # 允许其他回调执行
                        rclpy.spin_once(self, timeout_sec=0.01)
                        
                    else:
                        # 阻塞模式（原始行为）
                        tty.setraw(self.fd)
                        ch = sys.stdin.read(1)
                        
                        if ch == '\x1b':
                            next_char = sys.stdin.read(2)
                            if next_char == '[A':
                                print("\n上箭头被按下")
                            elif next_char == '[B':
                                print("\n下箭头被按下")
                            elif next_char == '[C':
                                print("\n右箭头被按下")
                            elif next_char == '[D':
                                print("\n左箭头被按下")
                            else:
                                print("\n未知的方向键")
                            ch = next_char.replace('[','')
                        else:
                            print(f"\n普通字符: {ch}")
                        
                        if ch in self.keyboardBindings:
                            self.cmd.data = ch
                            self.publish_keyboard_info(self.cmd)
                        elif ch == 'm':  # 切换到连续模式
                            self.continuous_mode = True
                            # 设置终端为非阻塞模式
                            tty.setcbreak(self.fd)
                            # 启动定时器
                            if not self.control_timer:
                                self.control_timer = self.create_timer(self.control_update_rate, self.update_controls)
                            print("\nSwitched to Continuous Mode")
                            print("W/S: Increase/Decrease RPM | A/D: Left/Right Rudder | Space: Release all")
                            print("Note: Release detection based on timeout - hold key for continuous control")
                        elif ch == 'l':
                            receive_latlon = True
                        elif ch == 'k':
                            receive_psi_d = True
                        elif ch == 'q':
                            print("\nNow keyboard_interaction.py is getting exit...")
                            self.is_shutting_down = True
                            break
                        else:
                            print("\nInput error and meaningless. You should reinput.")
                            
                elif receive_latlon:
                    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
                    
                    latlon_input = input("\nEnter latitude and longitude in the format (lat, lon):(q for exiting)\n")
                    if latlon_input == 'q':
                        receive_latlon = False
                        print("Exited latitude/longitude input mode.")
                    else:
                        match = re.match(r'(\d+(\.\d+)?),\s*(\d+(\.\d+)?)', latlon_input)
                        if match:
                            lat = float(match.group(1))
                            lon = float(match.group(3))
                            if 29 <= lat <= 32 and 119 <= lon <= 122:
                                print(f"Received valid latitude and longitude: ({lat}, {lon})")
                                self.latlon_tuple.data = latlon_input
                                self.publish_keyboard_info(self.latlon_tuple)
                                # 等待ack
                                wait_count = 0
                                self.latlon_acknowledged = False
                                while not self.latlon_acknowledged and rclpy.ok() and wait_count < 20:
                                    print("Waiting for acknowledgement ... ")
                                    rclpy.spin_once(self, timeout_sec=0.5)
                                    wait_count += 1
                                if self.latlon_acknowledged:
                                    print("Successfully sent.")
                                else:
                                    print("No acknowledgement received.")
                            else:
                                print(f"Received : {lat}, {lon}")
                                print("Latitude or longitude out of range, please re-enter.")
                        else:
                            print("Invalid input format, please re-enter in the format 'lat, lon'.")
                            
                elif receive_psi_d:
                    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
                    
                    psi_d_input = input("\nEnter psi_d (degree):(q for exiting)\n")
                    if psi_d_input == 'q':
                        receive_psi_d = False
                        print("Exited psi_d input mode.")
                    else:
                        print(f"Received valid psi_d input: {psi_d_input}")
                        self.psi_d_float.data = psi_d_input
                        self.publish_keyboard_info(self.psi_d_float)

            finally:
                if not self.continuous_mode:
                    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def acknowledge_callback(self, msg):
        if msg.data:
            self.latlon_acknowledged = True

    def publish_keyboard_info(self, data):
        if not self.is_shutting_down:
            self.pub.publish(data)

    def reset_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        # 只在未关闭时发送归零命令
        if not self.is_shutting_down:
            try:
                if hasattr(self, 'thrust_rpm_pub'):
                    rpm_msg = Float64()
                    rpm_msg.data = 0.0
                    self.thrust_rpm_pub.publish(rpm_msg)
                if hasattr(self, 'rudder_angle_pub'):
                    rudder_msg = Float64()
                    rudder_msg.data = 0.0
                    self.rudder_angle_pub.publish(rudder_msg)
            except:
                pass  # 忽略已销毁的发布器错误

def main(args=None):
    rclpy.init(args=args)
    interacter = KeyboardInteracter()
    try:
        interacter.keyboard_loop()
    finally:
        interacter.is_shutting_down = True
        interacter.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()