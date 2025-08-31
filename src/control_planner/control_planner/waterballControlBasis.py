#!/usr/bin/env python3
# coding: utf-8
# V9_2 modification compared with V9_1: abort the use of setup_transform function which was written in class WaterballControlBasis. We use static transform publisher in launch file to do the same thing.


## A Control basis neccessary for Waterball control no matter for real or simulation.
## Including transform setup 
# ========= Python标准库与通用第三方库 =========
import numpy as np
import time
import collections
import random
import queue
import matplotlib.pyplot as plt

# ========= ROS2核心依赖 =========
import rclpy
from rclpy.time import Time

# ========= ROS2消息类型 =========
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped, Twist, Point, TransformStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix, Imu, JointState


# 自定义消息
from pid_interfaces.msg import Command         # 你的自定义Command消息

# 导航Action相关（如用nav2）
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose        # 适用于ROS2 Nav2导航
from action_msgs.msg import GoalStatusArray  # 如果你有对应的GoalStatusArray，替换包名
from std_msgs.msg import Header

# ========= ROS2 TF库 =========
import tf_transformations                         # 纯Python欧拉角与四元数变换
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformListener,
    Buffer
)

# ========= 项目本地工具和算法 =========
from control_planner import usvParam as P
from control_planner.curveGenerator import curve_x, curve_y
from control_planner.lonlat2coor import geodetic_to_polar, polar_to_cartesian, latlon2en
from control_planner.LOSguidance import classical_LOSGuider, adaptive_LOSGuider
from control_planner.courseController import CourseController, transitionJudge, slow_transition
from control_planner.MFACControl import MFAC_Controller
from control_planner.PIDControl import PIDControl
from control_planner.loc_PIDControl import pid_loc, PID_LocTypeDef
from control_planner.distanceController import Distance_Controller
from control_planner.trans_GCJ02ToWGS84 import GCJ02ToWGS84
from control_planner.object_follow import calculate_local_coordinates, convert_to_global_coordinates




# Function definitions
def calculate_distance(position1,position2): # calculate distance between [e1,n1] and [e2,n2], and get accordingly vector
    # position2 is the target, position1 is the starting point.
    de = position2[0] - position1[0]
    dn = position2[1] - position1[1]
    distance = (de**2+dn**2)**0.5+1e-5

    # Compute the direction vector of the tangent
    if distance < 0.1:
        factor = 10/distance
        tangent_vector = np.array([de, dn])*factor # Multiplying 100 is in case the norm could be too tiny.
    else:
        factor = 1
        tangent_vector = np.array([de, dn])
    # Compute the direction vector of the y axis
    y_axis_vector = np.array([0, 1])

    # Compute the cosine of the angle between the tangent and the y axis
    cosine_angle = np.dot(tangent_vector, y_axis_vector) / (distance*factor)

    # Compute the angle in degrees,relative to y axis.
    angle = np.arccos(cosine_angle) * 180 / np.pi
    if de >0:
        pass
    else:
        angle = - angle
    return distance,angle
def sigmoid(x, a, b, c, d):
    x = np.abs(x)  # 取绝对值
    return d - a / (1 + np.exp(-b * (x - c)))

def angle_diff(angle1, angle2):
    # compute anle diff.
    psi_diff = angle1-angle2
    if psi_diff < -180:
        psi_diff += 360
    if psi_diff > 180:
        psi_diff -= 360
    return psi_diff

class WaterballControlBasis:
    def __init__(self, node, Ball):
        # node: 传入ROS2 Node对象
        self.node = node
        self.Ball = Ball

        self.pub_path = node.create_publisher(Path, "/topic_msgs_path", 10)
        self.msg_path = Path()
        self.last_publish_time = node.get_clock().now()

        self.pub_expect_path = node.create_publisher(Path, "/topic_expect_path", 10)
        self.twist_msg = Twist()

        self.pub_orient = node.create_publisher(Odometry, "/topic_orient", 1)
        self.pub_cmd_vel = node.create_publisher(Twist, "cmd_vel", 1)
        self.pub_v_w_force = node.create_publisher(Twist, "cmd_vel", 1)
        self.msg = Command()
        self.pub_state = node.create_publisher(Command, "/topic_state", 1)

        # 添加 joint_state 发布器
        self.pub_joint_state = node.create_publisher(JointState, '/joint_states', 10)
        # 设置关节位置（90度）
        self.joint_position = 0 * 3.14/180
        # 创建定时器，10Hz频率
        # self.joint_state_timer = node.create_timer(0.1, self.publish_joint_state)

        # tf2 broadcaster，ROS2要求要先new出来，不能每次new
        self.static_broadcaster = StaticTransformBroadcaster(node)
        self.tf_broadcaster = TransformBroadcaster(node)

        self.static_transformStamped = TransformStamped()
        # self.setup_transform()

        self.t_range = P.t_range
        self.n_points = P.n_points

    def publish_joint_state(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['base_spherical_center_high_joint']
        msg.position = [self.joint_position]
        self.pub_joint_state.publish(msg)

    def setup_transform(self):
        # static initial frame transform: world(ENU)->NED
        self.static_transformStamped.header.stamp = self.node.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = "world"
        self.static_transformStamped.child_frame_id = "NED"
        self.static_transformStamped.transform.translation.x = 0.0
        self.static_transformStamped.transform.translation.y = 0.0
        self.static_transformStamped.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(np.pi, 0, np.pi/2)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]
        self.static_broadcaster.sendTransform(self.static_transformStamped)

    def generate_path(self):
        for i in range(self.n_points):
            t_l = np.linspace(self.t_range[0], self.t_range[-1], self.n_points)
            x = curve_x(t_l[i])
            y = curve_y(t_l[i])
            self.create_pose(x, y)

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = y
        pose.pose.position.y = x
        self.msg_epath.poses.append(pose)

    def visualize_expected_path(self, e_list, n_list):
        msg_epath = Path()
        msg_epath.header.frame_id = "NED"
        msg_epath.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(len(e_list)):
            pose = PoseStamped()
            pose.pose.position.x = float(n_list[i])
            pose.pose.position.y = float(e_list[i])
            pose.header.stamp = self.node.get_clock().now().to_msg()
            msg_epath.poses.append(pose)
        self.pub_expect_path.publish(msg_epath)

    def visualize_actual_path(self, n, e, MAX_PATH_LENGTH=10000):
        msg_pose = PoseStamped()
        msg_pose.pose.position.x, msg_pose.pose.position.y = float(n), float(e)
        msg_pose.header.stamp = self.node.get_clock().now().to_msg()
        if len(self.msg_path.poses) >= MAX_PATH_LENGTH:
            self.msg_path.poses.pop(0)
        self.msg_path.poses.append(msg_pose)
        self.msg_path.header.frame_id = "NED"
        self.msg_path.header.stamp = self.node.get_clock().now().to_msg()
        # 比如每1秒发布一次
        if (self.node.get_clock().now() - self.last_publish_time).nanoseconds > 1e9:
            self.pub_path.publish(self.msg_path)
            self.last_publish_time = self.node.get_clock().now()

    def publish_cmd_vel(self, vx_d, angular_v_d):
        self.twist_msg.linear.x = vx_d
        self.twist_msg.angular.z = angular_v_d
        self.pub_cmd_vel.publish(self.twist_msg)

    def publish_v_w_force(self,mode):
        value_gradient = self.Ball.PID_LOC.Ek - self.Ball.PID_LOC.Ek1
        if self.Ball.model != 'Rudder':
            w_force = pid_loc(self.Ball.angular_v_d*100, self.Ball.angular_v*100, value_gradient, self.Ball.PID_LOC)
            self.Ball.w_force = float(w_force)
            self.Ball.v_force = float(self.Ball.vx_d)
            if mode != 'Sim':
                self.twist_msg.linear.x = self.Ball.v_force
                self.twist_msg.angular.z = self.Ball.w_force
                self.pub_v_w_force.publish(self.twist_msg)
        else:
            rudder_angle = pid_loc(self.Ball.angular_v_d*100, self.Ball.angular_v*100, value_gradient, self.Ball.PID_LOC)
            # self.Ball.rudder_angle = float(rudder_angle)
            # self.Ball.thrust_n = float(self.Ball.vx_d)
            if mode != 'Sim':
                self.twist_msg.linear.x = self.Ball.thrust_n
                self.twist_msg.angular.z = self.Ball.rudder_angle
                self.pub_v_w_force.publish(self.twist_msg)
    def state_Pub(self):
        self.msg.e = float(self.Ball.e)
        self.msg.n          = float(self.Ball.n)
        self.msg.psi_d      = float(self.Ball.psi_d)
        self.msg.psi        = float(self.Ball.psi)
        self.msg.vx         = float(self.Ball.vx)
        self.msg.vx_d       = float(self.Ball.vx_d)
        self.msg.angular_v_d = float(self.Ball.angular_v_d)
        self.msg.angular_v   = float(self.Ball.angular_v)
        self.msg.t_l       = float(self.Ball.T_l)
        self.msg.t_r       = float(self.Ball.T_r)

        self.pub_state.publish(self.msg)

    def baselink_NED_transform(self, n, e, psi):
        trans_msg = TransformStamped()
        trans_msg.header.stamp = self.node.get_clock().now().to_msg()
        trans_msg.header.frame_id = "NED"
        trans_msg.child_frame_id = "base_link"
        trans_msg.transform.translation.x = float(n)
        trans_msg.transform.translation.y = float(e)
        trans_msg.transform.translation.z = float(0)
        quat = tf_transformations.quaternion_from_euler(np.pi, 0, psi * np.pi / 180)
        trans_msg.transform.rotation.x = quat[0]
        trans_msg.transform.rotation.y = quat[1]
        trans_msg.transform.rotation.z = quat[2]
        trans_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(trans_msg)
        # self.node.get_logger().info("Now in baselink_NED_transform =====")


class Waterball_50:
    def __init__(self, node):
        # node: 传入你的rclpy Node对象
        self.node = node

        self.az, self.distance = 0, 0
        self.n_list = [0]
        self.e_list = [0]
        self.n0, self.e0 = 0, 0
        self.n1, self.e1 = self.n0, self.e0 = 0, 0
        self.n, self.e = 0, 0
        self.anchor_position = []
        self.goal_pose = [0, 0, 0]
        self.object_pose = [0, 0, 0]
        self.steady_state = False
        self.object_angle, self.object_distance = 0, 0
        self.x_global_NED, self.y_global_NED = 0, 0
        self.x_global_map, self.y_global_map = 0, 0
        self.x_local, self.y_local = 0, 0

        self.stay_point = [0, 0]
        self.stay_psi = 90

        self.psi_d = 0.0
        self.psi_d_pre = self.psi_d
        self.psi = 45
        self.psi0, self.psi_d0 = 0.0, 0.0
        self.lon0, self.lat0 = 0, 0
        self.lon, self.lat = self.lon0, self.lat0

        self.angular_v_d, self.vx_d = 0.0, 0.0
        self.angular_v = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.angular_v_d0 = 0.0
        self.v_force, self.w_force = 0.0, 0.0
        self.rudder_angle = 0.0 # 角度制
        self.thrust_n = 0.0 #转速
        self.omega_l, self.omega_r = 0.0, 0.0
        self.T_r, self.T_l = 0.0, 0.0

        self.PID_LOC = PID_LocTypeDef(Kp=0.07, Ki=0.004, Kd=0.15, MaxSum=20, MaxResult=250)

        self.mode = "Default"
        self.last_mode = "Default"
        self.model = "Thrust"  # 'Rudder' or 'Thrust'

        self.count = 0
        self.flag = 0
        self.i = 0
        # ROS2时间建议用 rclpy.time 或 time.time
        self.t_start = node.get_clock().now().nanoseconds / 1e9
        self.t_get = time.time()

        self.DirectionKey_dic = {"Right": 0, "Up": 0}
        self.turn_direction_char = ''

        self.forward_flag = False
        self.stay_flag = False
        self.coursecontrol_flag = False
        self.pre_stage = False

        self.distance_range = 0.5

        self.classical_guider = classical_LOSGuider()
        self.adaptive_guider = adaptive_LOSGuider()

        # 业务控制基类，需传入node
        self.Basis = WaterballControlBasis(node, self)
        # self.Basis.baselink_NED_transform(self.n, self.e, self.psi)

        self.Sub = WaterballSubscriber(node,self)
        self.Mode = Waterball_BasicModes(node,self)
        self.Psi_d_Generator = Waterball_Psi_d_Generator(self)

        self.coursecontroller = CourseController()
        self.MFACcontroller = MFAC_Controller(self)
        self.distancecontroller = Distance_Controller(1, 0, 0.1)

        self.psi_diff_history = collections.deque(maxlen=300)

        self.straight_time_get = 0

        self.default_stay_flag = False
        self.mb_status = 0
        self.mb_status_flag = False
        self.test = 0
        self.pursue_flag = False

        # ROS2 tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)
        # try:
        #     # ROS2不再用waitForTransform，需异步/定期检查是否有tf
        #     pass  # 你可以实现自己的“等待tf”逻辑，如周期尝试self.tf_buffer.lookup_transform(...)
        # except Exception as e:
        #     print(f"Exception0:{e}")


        timeout = 4.0  # 超时时间（秒）
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < timeout:
            try:
                self.Basis.baselink_NED_transform(self.n, self.e, self.psi)
                trans = self.tf_buffer.lookup_transform("base_link", "NED", rclpy.time.Time())
                self.node.get_logger().info("Transform found successfully.")
                break  # 找到了就跳出循环
            except Exception as e:
                time.sleep(0.1)  # 没找到就等一会儿再试
        else:
            print("Exception0: Transform not found within timeout")
            trans = None

    def set_orient0(self, mode='Sim'):
        if mode == 'Real':
            print("=============== Setting orient to north...======================")
            while (np.abs(self.psi) > 10) and rclpy.ok():
                self.coursecontroller.set_pid(2, 1, 0.0)
                self.angular_v_d = self.coursecontroller.update(0, self.psi)
                self.send_control(self.angular_v_d, 0)
                print("\n psi:%f" % (self.psi))
            print("========================Setting orient to north FINISH ==========================")
        else:
            pass

    def get_mode(self):
        return self.mode

    def set_position(self, e, n):
        self.e = e
        self.n = n

    def get_position(self):
        return [self.e, self.n]

    def set_psi(self, psi):
        self.psi = psi

    def set_psi_dot(self, psi_dot):
        self.angular_v = psi_dot

    def set_vx(self, vx):
        self.vx = vx

    def send_control(self, angular_v_d, vx_d):
        self.angular_v_d = angular_v_d
        self.vx_d = vx_d

    def steady_state_judge(self):
        psi_diff = np.abs(angle_diff(self.psi, self.psi_d))
        self.psi_diff_history.append(psi_diff)
        self.steady_state = all(np.abs(psi) < 6 for psi in self.psi_diff_history)
        return self.steady_state

    def distance_control(self, psi_diff):
        de = self.stay_point[0] - self.e
        dn = self.stay_point[1] - self.n
        distance = (de ** 2 + dn ** 2) ** 0.5
        dt = 0.1
        # self.vx_d = self.distancecontroller.update(error=distance, dt=dt)
        if np.abs(psi_diff) <= 5:
            self.count += 1
            if self.count == round(300):
                self.vx_d = 2
                self.count = 0
            else:
                self.vx_d = 1.5
        elif np.abs(psi_diff) <= 30:
            self.count = 0
            self.vx_d = 1
        elif np.abs(psi_diff) <= 60:
            self.count = 0
            self.vx_d = 0.8
        elif np.abs(psi_diff) <= 90:
            self.count = 0
            self.vx_d = 0.5
        else:
            self.count = 0
            self.vx_d = 0
        return self.vx_d

    def check_mode(self):
        if self.mode != 'StraightForward' and self.last_mode == 'StraightForward':
            self.forward_flag = False
        if self.mode != "Stabilization" and self.last_mode == "Stabilization":
            self.stay_flag = False
        if self.mode != "Coursecontrol" and self.last_mode == "Coursecontrol":
            self.coursecontrol_flag = False
        if self.mode != "DirectionKey" and self.last_mode == "DirectionKey":
            self.DirectionKey_dic = {"Right": 0, "Up": 0}
        if self.mode != "TurnAround" and self.last_mode == "TurnAround":
            self.coursecontrol_flag = False
        if self.mode != "Purse" and self.last_mode == "Purse":
            self.pursue_flag = False
        if self.mode != "Default" and self.last_mode == "Default":
            self.default_stay_flag = False
        self.last_mode = self.mode

    def move(self):
        psi_diff = np.abs(angle_diff(self.psi, self.psi_d))
        if self.mode == "StraightForward":
            straight_flag = True if time.time() - self.straight_time_get >= 2.5 else False
            if not straight_flag:
                self.vx_d = 0.5
            else:
                self.coursecontroller.set_pid(1.5, 0, 0)
                self.angular_v_d = self.coursecontroller.update(self.psi_d, self.psi)
                self.vx_d = 0.5  # or use self.distance_control(psi_diff)

        if self.mode == "Stabilization":
            if False:
                self.angular_v_d = 0.0
                self.vx_d = 0.0
            else:
                self.pre_stage = True
            de = self.stay_point[0] - self.e
            dn = self.stay_point[1] - self.n
            distance = (de ** 2 + dn ** 2) ** 0.5
            if np.abs(psi_diff) <= 15:
                self.count += 1
                if self.count == round(300):
                    self.vx_d = 1
                    self.count = 0
                else:
                    self.vx_d = 0.8
            else:
                self.count = 0
                self.vx_d = 0
            if distance < 1:
                self.vx_d = 0
                self.angular_v_d = 0
                self.pre_stage = False

        if self.mode == "Follow":
            self.vx_d = self.distance_control(psi_diff)
            i = self.i
            dis2goal = np.sqrt((self.e - self.e_list[i]) ** 2 + (self.n - self.n_list[i]) ** 2)
            if dis2goal < self.distance_range:
                self.vx_d = 0.3
        if self.mode == "Default":
            self.angular_v_d = 0.0
            self.vx_d = 0.0

        ## compute tau and force. dont' use at present for real.
        # u_queue = self.MFACcontroller.update(self.angular_v_d,self.angular_v)
        # w_force = u_queue[0]
        # v_force = self.vx_d
        
        # return w_force,v_force
        
        # return self.angular_v_d,self.vx_d



class WaterballSubscriber:
    def __init__(self, node, Ball):
        self.node = node
        self.Ball = Ball

        self.pub = node.create_publisher(String, 'acknowledge', 1)
        self.ack_msg = String()

        # 所有需要的订阅在这里注册
        self.node.create_subscription(String, "keyboard_info", self.keyboard_callback, 10)
        self.node.create_subscription(Imu, "imu", self.angular_v_callback, 10)
        self.node.create_subscription(Float64, "yaw", self.yaw_callback, 10)
        self.node.create_subscription(NavSatFix, "fix", self.gps_callback, 10)
        self.node.create_subscription(Point, "object_info", self.object_info_callback, 10)
        self.node.create_subscription(Twist, "cmd_vel", self.mb_av_callback, 10)
        self.node.create_subscription(PoseStamped, "/goal_pose", self.mb_goal_callback, 10)
        self.node.create_subscription(GoalStatusArray, "/move_base/status", self.mb_status_callback, 10)
        self.node.create_subscription(Float64, "rudder_joint_angle", self.rudder_joint_angle_callback, 10)
        self.node.create_subscription(Float64, "thrust_rpm", self.thrust_rpm_callback, 10)
        self.node.create_subscription(Point, "usvState_from_matlab", self.usvState_callback, 10)
        self.node.create_subscription(Point, "usvVel_from_matlab", self.usvVel_callback, 10)
        # 其它需要的订阅也在此注册

    def state_update(self, mode='Sim'):
        # 只需运行模式判断和后处理即可，订阅不用再每次注册
        self.Ball.check_mode()
        if mode == 'Real':
            # 执行实际模式下的额外逻辑（如果有）
            e, n = latlon2en(self.Ball.lat0, self.Ball.lon0, self.Ball.lat, self.Ball.lon)
            self.Ball.e, self.Ball.n = e, n
        else:
            pass

    def get_lon0lat0(self, mode='Sim'):
        if mode == 'Real':
            self.node.get_logger().info("=============== Getting lat0 and lon0...======================")
            while (self.Ball.lat0 < 20 or self.Ball.lon0 < 120) and rclpy.ok():
                # 订阅消息时直接等回调更新 Ball.lat0 Ball.lon0
                time.sleep(0.1)
            self.node.get_logger().info(f"\n lat0:{self.Ball.lat0}, lon0:{self.Ball.lon0}")
            self.node.get_logger().info("========================Get lat0 and lon0 END ==========================")
        else:
            self.node.get_logger().info("mode == sim, get_lon0lat0 pass")

    # 各类订阅回调
    def thrust_rpm_callback(self, data):
        # 处理舵机转速信息
        self.Ball.thrust_n = data.data
        # print("thrust_rpm_callback: ", self.Ball.thrust_n)
        # self.node.get_logger().info(f"thrust_rpm_callback: {self.Ball.thrust_n}")
    def rudder_joint_angle_callback(self, data):
        # 处理舵机角度信息
        self.Ball.rudder_angle = data.data
    def mb_status_callback(self, data):
        if len(data.status_list) > 0:
            self.Ball.mb_status = data.status_list[-1].status

    def mb_goal_callback(self, data):
        self.Ball.goal_pose[0] = data.pose.position.x  # e
        self.Ball.goal_pose[1] = data.pose.position.y  # n
        quaternion = (
            data.pose.orientation.x, data.pose.orientation.y,
            data.pose.orientation.z, data.pose.orientation.w
        )
        # euler_angle = tf_transformations.euler_from_quaternion(quaternion)
        self.Ball.goal_pose[2] = 0  # psi, 如有需要请加上四元数转欧拉角

    def mb_av_callback(self, data):
        
        self.Ball.angular_v_d = (-data.angular.z * 180 / 3.14) / 10
        self.Ball.vx_d = data.linear.x*3 # temp multipy 3
        # self.node.get_logger().info(f"=============cmd_vel get point===== vx_d: {self.Ball.vx_d}, angular_v_d: {self.Ball.angular_v_d}")
        self.Ball.send_control(self.Ball.angular_v_d, self.Ball.vx_d)

    def angular_v_callback(self, data):
        self.Ball.angular_v = - data.angular_velocity.z * 180 / np.pi

    def yaw_callback(self, data):
        psi = (-data.data + 180) % 360 - 180
        self.Ball.psi = psi

    def gps_callback(self, data):
        if data.longitude > 100 and data.latitude > 20:
            self.Ball.lon = data.longitude
            self.Ball.lat = data.latitude

    def object_info_callback(self, data):
        self.Ball.object_angle = data.x
        self.Ball.object_distance = data.y
    
    def usvState_callback(self, data):
        self.Ball.n = data.x
        self.Ball.e = data.y
        self.Ball.psi = data.z #deg
        # print("usvState_callback: ", self.Ball.e, self.Ball.n, self.Ball.psi)

    def usvVel_callback(self, data):
        self.Ball.vx = data.x
        self.Ball.angular_v = data.y * 180 / np.pi
        # print("usvVel_callback: ", self.Ball.vx, self.Ball.angular_v)

    def keyboard_callback(self, data):
        # 一个按键按下以后的0.2秒内的其他按键都不会被输出。
        if time.time() - self.Ball.t_get > 0.2:
            self.Ball.t_get = time.time()
            flag = True
        else:
            flag = False

        if len(data.data) == 1:
            self.Ball.keyboard_char = data.data
            if self.Ball.keyboard_char in "adwsop":
                self.Ball.mode = "DirectionKey"
                if flag:
                    if data.data == 'a':
                        self.Ball.DirectionKey_dic["Right"] -= 1
                    elif data.data == 'd':
                        self.Ball.DirectionKey_dic["Right"] += 1
                    elif data.data == 'w':
                        self.Ball.DirectionKey_dic["Up"] += 0.5
                    elif data.data == 's':
                        self.Ball.DirectionKey_dic["Up"] -= 0.5
                    elif data.data == "o":
                        self.Ball.DirectionKey_dic["Right"] = 0
                    elif data.data == "p":
                        self.Ball.DirectionKey_dic["Up"] = 0
            elif self.Ball.keyboard_char == "u":
                self.Ball.mode = "StraightForward"
                self.Ball.straight_time_get = time.time()
            elif self.Ball.keyboard_char in ['A', 'B', 'C', 'D']:
                self.Ball.turn_direction_char = self.Ball.keyboard_char
                self.Ball.mode = "TurnAround"
                self.Ball.coursecontrol_flag = False
            elif self.Ball.keyboard_char == "e":
                self.Ball.mode = "Stabilization"
            elif self.Ball.keyboard_char == "f":
                self.Ball.mode = "Follow"
            elif self.Ball.keyboard_char == "x":
                self.Ball.mode = "Coursecontrol"
            elif self.Ball.keyboard_char == "h":
                self.Ball.mode = "Pursue"
            else:
                self.Ball.mode = "Default"
        elif len(data.data) > 1:
            try:
                self.Ball.psi_d = int(data.data)
            except:
                split_string = data.data.split(',')
                as_tuple = (float(split_string[0]), float(split_string[1]))
                # as_tuple = GCJ02ToWGS84(as_tuple)  # coordinate transform if needed
                newgoal_e, newgoal_n = latlon2en(self.Ball.lat0, self.Ball.lon0, as_tuple[0], as_tuple[1])
                dis2goal = np.sqrt(
                    np.power(newgoal_e - self.Ball.e_list[-1], 2) + np.power(newgoal_n - self.Ball.n_list[-1], 2))
                if dis2goal > 2:
                    print(f"Received latitude and longitude: {data.data}, transferred to e&n is: ({newgoal_e},{newgoal_n})")
                    self.Ball.e_list.append(newgoal_e)
                    self.Ball.n_list.append(newgoal_n)
                # Send acknowledgment
                self.ack_msg.data = f"Received: {data.data}. Com success."
                self.pub.publish(self.ack_msg)
        else:
            pass


    
class Waterball_Psi_d_Generator:
    def __init__(self, Ball):
        self.Ball = Ball

    def go_straight_forward(self):
        if not self.Ball.forward_flag:
            self.Ball.n0, self.Ball.e0 = self.Ball.n, self.Ball.e
            self.Ball.forward_flag = True
            self.Ball.n1 = self.Ball.n0 + 100 * np.cos(self.Ball.psi * np.pi / 180)
            self.Ball.e1 = self.Ball.e0 + 100 * np.sin(self.Ball.psi * np.pi / 180)
            self.Ball.n_list.append(self.Ball.n1)
            self.Ball.e_list.append(self.Ball.e1)
        if self.Ball.psi_d0 == 0.0:
            self.Ball.psi_d0 = self.Ball.psi_d
        self.Ball.psi_d = self.Ball.classical_guider.update(
            self.Ball.e0, self.Ball.n0, self.Ball.e1, self.Ball.n1, self.Ball.e, self.Ball.n
        )
        return self.Ball.psi_d

    def line_follow(self):
        i = self.Ball.i
        dis2goal = np.sqrt((self.Ball.e - self.Ball.e_list[i]) ** 2 + (self.Ball.n - self.Ball.n_list[i]) ** 2)
        if dis2goal < 2:
            if i < len(self.Ball.e_list) - 1:
                self.Ball.e0, self.Ball.n0 = self.Ball.e_list[i], self.Ball.n_list[i]
                self.Ball.e1, self.Ball.n1 = self.Ball.e_list[i + 1], self.Ball.n_list[i + 1]
                print("******************************************************************************+")
                print(f"dis2goal is {dis2goal}..")
                print("******************************************************************************+")
                self.Ball.i += 1
            else:
                print("=============Already arrive at final target position=============")
                self.Ball.mode = "Stabilization"

        self.Ball.psi_d = self.Ball.classical_guider.update(
            self.Ball.e0, self.Ball.n0, self.Ball.e1, self.Ball.n1, self.Ball.e, self.Ball.n
        )
        return self.Ball.psi_d

    def psi_d_update(self):
        if self.Ball.mode == "StraightForward":
            self.Ball.psi_d = self.go_straight_forward()
        elif self.Ball.mode == "Stabilization":
            # 这里假设 self.stay_here() 也是纯算法，不依赖 ROS
            self.Ball.psi_d = self.stay_here()
        elif self.Ball.mode == "Left":
            if self.Ball.psi0 == 0.0:
                self.Ball.psi0 = self.Ball.psi
                self.Ball.psi_d = self.Ball.psi0 - 20
            if np.abs(angle_diff(self.Ball.psi_d, self.Ball.psi)) < 10:
                self.Ball.psi0 = 0.0
                self.Ball.mode = "Default"
        elif self.Ball.mode == "Right":
            if self.Ball.psi0 == 0.0:
                self.Ball.psi0 = self.Ball.psi
                self.Ball.psi_d = self.Ball.psi0 + 20
            if np.abs(angle_diff(self.Ball.psi_d, self.Ball.psi)) < 10:
                self.Ball.psi0 = 0.0
                self.Ball.mode = "Default"
        elif self.Ball.mode == "Backward":
            if self.Ball.psi0 == 0.0:
                self.Ball.psi0 = self.Ball.psi
                self.Ball.psi_d = self.Ball.psi0 + 180
            if np.abs(angle_diff(self.Ball.psi_d, self.Ball.psi)) < 10:
                self.Ball.psi0 = 0.0
                self.Ball.mode = "Default"
        elif self.Ball.mode == "Follow":
            self.Ball.psi_d = self.line_follow()
        elif self.Ball.mode == "Default":
            self.Ball.psi0 = 0.0
            self.Ball.psi_d = 0

        # 如果有 transition 相关算法/平滑，需要用 time.time() 代替 rospy.get_time()
        # if self.Ball.psi_d0 == 0.0:
        #     self.Ball.psi_d0 = self.Ball.psi_d
        # t = time.time()
        # self.Ball.flag, self.Ball.t_start = transitionJudge(self.Ball.psi_d0, self.Ball.psi_d, self.Ball.flag, self.Ball.t_start)
        # self.Ball.psi_d = slow_transition(self.Ball.flag, self.Ball.psi_d, t, self.Ball.t_start)
        # if t - self.Ball.t_start > 6:
        return self.Ball.psi_d

    # 如果你还需要 stay_here，请补充如下
    def stay_here(self):
        # 示例：静止点保持，指向当前 stay_point
        de = self.Ball.stay_point[0] - self.Ball.e
        dn = self.Ball.stay_point[1] - self.Ball.n
        # 利用 atan2 求目标角度
        psi_target = np.arctan2(dn, de) * 180 / np.pi
        return psi_target
    


class Waterball_BasicModes:
    def __init__(self, node, Ball):
        self.node = node
        self.Ball = Ball
        self.t1 = 0
        self.t2 = 0
        self.is_orienting = False     

        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = "map"
        self.goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.goal_msg.pose.position.x = 0.0
        self.goal_msg.pose.position.y = 0.0
        self.goal_msg.pose.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.goal_msg.pose.orientation.x = quat[0]
        self.goal_msg.pose.orientation.y = quat[1]
        self.goal_msg.pose.orientation.z = quat[2]
        self.goal_msg.pose.orientation.w = quat[3] 

        self.pub_goal = node.create_publisher(PoseStamped, "/move_base_simple/goal", 1)
        # ROS2 action client
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def feedback_cb(self, feedback):
        self.node.get_logger().info(f"Currently at pose: {feedback.current_pose}")

    def mode_judge(self):
        if self.Ball.mode not in ["StraightForward", "Follow"]:
            if self.Ball.mode == "DirectionKey":
                a_v_d = self.Ball.DirectionKey_dic["Right"]
                l_v_d = self.Ball.DirectionKey_dic["Up"]
                self.Ball.send_control(a_v_d, l_v_d)
            elif self.Ball.mode == "Stabilization":
                self.stay_here()
            elif self.Ball.mode == "Pursue":
                self.pursue()
            elif self.Ball.mode == "TurnAround":
                self.turn_around(self.Ball.turn_direction_char)
            elif self.Ball.mode == "Coursecontrol":
                self.coursecontrol()
            elif self.Ball.mode == "Default":
                self.default_stay()
        elif self.Ball.mode in ["StraightForward", "Follow"]:
            self.Ball.Psi_d_Generator.psi_d_update()
            self.Ball.move()
            self.Ball.send_control(self.Ball.angular_v_d, self.Ball.vx_d)
        if np.abs(angle_diff(self.Ball.psi_d, self.Ball.psi_d_pre)) > 10:
            self.Ball.psi_diff_history.clear()
        self.Ball.psi_d_pre = self.Ball.psi_d

    def default_stay(self):      
        self.goal_msg.pose.position.x = 2.0
        self.goal_msg.pose.position.y = 0.0
        if not self.Ball.default_stay_flag:
            self.pub_goal.publish(self.goal_msg)
            self.Ball.default_stay_flag = True
        # self.Ball.Sub.mb_subscriber()  # ROS2已在__init__中注册，无需反复注册

    def turn_around(self, turn_direction_char):
        if turn_direction_char == 'D': # left 90
            turn_deg = -90
        elif turn_direction_char == 'C': # right 90
            turn_deg = 90
        elif turn_direction_char == 'B': # right 120
            turn_deg = 120
        else: # default left 250
            turn_deg = 250
        if not self.Ball.coursecontrol_flag:
            self.Ball.psi_d = (self.Ball.psi + turn_deg + 180) % 360 - 180
            self.Ball.coursecontrol_flag = True        
        self.Ball.coursecontroller.set_pid(0.1, 0.01, 0.0)
        self.Ball.angular_v_d = self.Ball.coursecontroller.update(self.Ball.psi_d, self.Ball.psi)
        self.Ball.send_control(self.Ball.angular_v_d, 0)
        if self.Ball.steady_state_judge():
            self.Ball.coursecontroller.limit = 6
        else:
            self.Ball.coursecontroller.limit = 10

    def coursecontrol(self, psi_d=0):
        if not self.Ball.coursecontrol_flag:
            self.Ball.psi_d = psi_d
            self.Ball.coursecontrol_flag = True
        self.Ball.coursecontroller.set_pid(2, 1, 0.4)
        self.Ball.angular_v_d = self.Ball.coursecontroller.update(self.Ball.psi_d, self.Ball.psi)
        self.Ball.send_control(self.Ball.angular_v_d, 0)
        if self.Ball.steady_state_judge():
            self.Ball.send_control(0, 0)

    def pursue(self):
        # 订阅初始化已在 ROS2 __init__ 完成，无需多次注册
        if not self.Ball.pursue_flag:
            random.seed(1)
            self.Ball.object_angle = random.uniform(-np.pi/2, np.pi/2)
            self.Ball.object_distance = random.uniform(5, 10)
            self.Ball.x_local, self.Ball.y_local = calculate_local_coordinates(self.Ball.object_angle, self.Ball.object_distance)
            self.Ball.x_global_NED, self.Ball.y_global_NED, self.Ball.x_global_map, self.Ball.y_global_map = convert_to_global_coordinates(
                self.Ball.x_local, self.Ball.y_local, self.Ball.listener
            )
            if self.Ball.x_global_NED is None:
                self.Ball.x_global_NED, self.Ball.y_global_NED = 100, 100
                self.Ball.x_global_map, self.Ball.y_global_map = 100, 100
            self.Ball.object_pose = [self.Ball.x_global_NED, self.Ball.y_global_NED, 0]
            self.goal_msg.pose.position.x = self.Ball.x_global_map
            self.goal_msg.pose.position.y = self.Ball.y_global_map
            self.pub_goal.publish(self.goal_msg)
            self.Ball.pursue_flag = True
        de = self.Ball.e - self.Ball.x_global_NED
        dn = self.Ball.n - self.Ball.y_global_NED
        self.Ball.distance = np.hypot(de, dn) + 1e-5
        if self.Ball.mb_status == 3:
            self.Ball.mb_status_flag = True
        if self.Ball.distance > 2 and self.Ball.mb_status_flag:
            self.Ball.mb_status_flag = False
            self.Ball.mb_status = 0

    def stay_here(self): 
        if not self.Ball.stay_flag:
            self.Ball.stay_point[0] = self.Ball.e
            self.Ball.stay_point[1] = self.Ball.n
            self.Ball.anchor_position = [self.Ball.stay_point[0], self.Ball.stay_point[1]]
            self.Ball.stay_flag = True
            self.t1 = 0

        de = self.Ball.e - self.Ball.goal_pose[0]
        dn = self.Ball.n - self.Ball.goal_pose[1]
        self.Ball.distance = np.hypot(de, dn) + 1e-5
        if self.Ball.mb_status == 3:
            self.Ball.mb_status_flag = True
        if self.Ball.distance > 5 and self.Ball.mb_status_flag:
            self.Ball.mb_status_flag = False
            self.Ball.mb_status = 0
            self.goal_msg.pose.position.x = self.Ball.goal_pose[0]
            self.goal_msg.pose.position.y = self.Ball.goal_pose[1]
            self.pub_goal.publish(self.goal_msg)







