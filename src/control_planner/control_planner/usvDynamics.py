#! /usr/bin/env python3

from control_planner import usvParam as P
from control_planner.courseController import courseLimitation

import tf_transformations  # ROS2没有自带tf.transformations, 需装第三方库
import rclpy
from rclpy.node import Node
from pid_interfaces.msg import Command

import numpy as np 
import random
import tf2_ros
import time
from geometry_msgs.msg import PointStamped, Vector3Stamped


class usvDynamics:
    def __init__(self, listener, ball_instance, alpha=0):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],      # initial angle
            [P.thetadot0], # initial angular rate
            [P.vel_u0], #initial velocity on x direction
            [P.vel_v0]
        ])  


        # Mass of the arm, kg
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))
        # Damping coefficient, Ns
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  
        self.b_f = P.b_f * (1.+alpha*(2.*np.random.rand()-1.))  
        self.b_r = P.b_r * (1.+alpha*(2.*np.random.rand()-1.)) 

        # the gravity constant is well known, so we don't change it.
        self.g = P.g

        # sample rate at which the dynamics are propagated
        self.Ts = P.Ts  
        self.torque_limit = P.tau_max

        #转动惯量
        self.I_z = P.I_z

        #propeller to center 
        self.r = P.r

        self.thrust_F = 0.0 #推进器推力
        self.T_list = [0.0,0.0]

        self.ball_instance = ball_instance # 水球机器人实例，用于获取各个机器人的属性
        self.force_X = 0.0 #X方向合力
        self.force_Y = 0.0
        self.f_vec = [0.0,0.0,0.0]
        self.tau = 0.0 # 主动力矩
        self.Tau = 0.0 #合力矩
        self.model = "Thrust"  # 默认驱动模式为差速驱动



        self.listener = listener
    def add_wind(self, data):
        try:
            # 1. 查找最新的transform，target='base_link', source='NED'
            # 注意：ROS2的tf2中，lookup_transform(target, source, time)
            # 返回geometry_msgs.msg.TransformStamped对象
            trans = self.listener.lookup_transform(
                'base_link', 'NED', rclpy.time.Time())

            # 2. 获取平移和四元数
            trans_vec = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            quat = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]

            # 3. wind_force_map（4维） → 旋转，只取XYZ分量（不要加第四个0）
            wind_force_map = [data[0], data[1], 0]

            # 4. 使用tf_transformations进行四元数旋转
            v_rotated = tf_transformations.quaternion_matrix(quat)[:3, :3].dot(wind_force_map)

            # 5. 累加平移，获得boat系下的力
            wind_force_boat = [
                v_rotated[0] + trans_vec[0],
                v_rotated[1] + trans_vec[1],
                v_rotated[2] + trans_vec[2]
            ]

            return wind_force_boat
        except Exception as e:
            print(f"Exception1:{e}")
            return [0, 0, 0]
    
    def rudder2tau(self):
        """
        计算无人船基于舵角和线速度的偏航力矩
        
        使用公式: M_z = K1 * V^2 * δ + K2 * V * δ^2
        其中:
        - M_z: 偏航力矩 (N·m)
        - V: 线速度 (m/s)
        - δ: 舵角 (rad)
        - K1, K2: 系数
        """
        # 获取舵角和线速度
        delta = self.ball_instance.rudder_angle * 3.14/180  # 舵角 (假设最终单位为弧度)
        V = self.ball_instance.vx               # 线速度
        
        # 定义系数 (这些值需要根据实际船舶参数调整)
        K1 = 5    # 一阶系数
        K2 = 3    # 二阶系数
        
        # 计算偏航力矩
        # M_z = K1 * V^2 * δ + K2 * V * δ^2
        M_z = K1 * V**2 * delta + K2 * V * delta**2
        
        # 可选：添加更高阶项
        # K3 = 0.1  # 三阶系数
        # M_z += K3 * V**3 * delta
        
        # 可选：考虑速度很低时的特殊处理
        # 避免在速度接近零时出现不稳定
        if abs(V) < 0.1:  # 速度阈值 (m/s)
            M_z *= abs(V) / 0.1  # 线性衰减
        
        # 将计算得到的力矩赋值给tau
        self.tau = -M_z #角度和力矩的作用效果相反，所以加个负号     

    
    def generate_force_tau(self):
        """
        根据不同的驱动模式计算推力和力矩
        输入：
            T_u: 推进器输入 [T_l, T_r]
            model: 驱动模式 "Rudder" 或 "Thrust"
        输出：
            force_X: X方向(前进方向)的力
            force_Y: Y方向(侧向)的力  
            tau: 转动力矩
        """
        
        thetadot = self.state.item(1)
        vel_u = self.state.item(2)
        vel_v = self.state.item(3)         
        
        if self.model == "Rudder":
            # 舵-桨模式 
            # 这里假设使用单推进器+舵角控制
            # rudder_angle = ...  # 舵角需要从某处获取
            force_X = self.thrust_F - self.b_f * vel_u 
            force_Y = 0  # 待实现
            # print(f"self.tau: {self.tau}")
            # print(f"self.r: {self.r}")
            # print(f"self.b: {self.b}")
            # print(f"thetadot: {thetadot}")
            Tau = self.tau * self.r - self.b * thetadot      
            
        elif self.model == "Thrust":
            # 差速驱动模式
            # X方向力：两个推进器推力之和减去前向阻力
            T_u = self.T_list  # 获取推进器输入
            T_l = T_u[0]
            T_r = T_u[1]
            force_X = (T_r + T_l) - self.b_f * vel_u
            
            # Y方向力：只有侧向阻力
            force_Y = -self.b_f * vel_v
            
            # 转动力矩：推力差产生的力矩减去旋转阻尼
            Tau = (T_l - T_r) * self.r - self.b * thetadot
            
        else:
            # 默认情况
            pass

        self.force_X = force_X
        self.force_Y = force_Y
        self.f_vec = [force_X, force_Y, 0]  # 机器人坐标系下的力向量
        self.Tau = Tau
        
    def update(self,model):
        self.model = model
        self.generate_force_tau()  # 计算推力和力矩
        # This is the external method that takes the input u at time
        # t and returns the output n at time t.
        # saturate the input torque
        # T_u = self.saturate(T_u, self.torque_limit) #T_u is a list after editing
        
        self.rk4_step()  # propagate the state by one time sample
        self.ball_instance.psi,self.ball_instance.angular_v,self.ball_instance.vx,self.ball_instance.vy = self.h()  # return the corresponding output
        #print("thetadot is:%f"%self.state.item(1))
        #print("vel_v is:%f"%vel_v)
        #print("vel_u is:%f"%vel_u)
        # print(f"self.state:\n{self.state}")


    def f(self,state):
        # Return edot = f(e,u), the system state update equations
        # re-label states for readability
        wind_force_boat = [0,0] #self.add_wind(wind_u)
        

        thetadot = self.state.item(1)
        vel_u = self.state.item(2)
        vel_v = self.state.item(3)        


        
        # 计算角加速度
        thetaddot = self.Tau/self.I_z + 0.1*vel_u*vel_v
        
        # 计算线加速度
        vel_udot = self.force_X / self.m + 0.01 * vel_v * thetadot
        vel_vdot = 0.0 #self.force_Y / self.m - 0.01 * vel_u * thetadot
        
        edot = np.array([[thetadot],
                        [thetaddot],
                        [vel_udot],
                        [vel_vdot]
                        ])
        
        return edot

    def h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state.item(0)
        #n = np.array([[theta]])
        psi = courseLimitation(theta)
        psi_dot = self.state.item(1)
        vel_u = self.state.item(2)
        vel_v = self.state.item(3)
        
        return psi,psi_dot/10,vel_u,vel_v

    def rk4_step(self):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state)
        F2 = self.f(self.state + self.Ts / 2 * F1)
        F3 = self.f(self.state + self.Ts / 2 * F2)
        F4 = self.f(self.state + self.Ts * F3)
        self.state = self.state + self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        # self.state = np.array([[self.state.item(0)],[self.angular_v],[self.state.item(2)],[self.state.item(3)]])

    def saturate(self, u, limit):
        for i in range(len(u)):
            T = u[i]
            if abs(T) > limit:
                T = limit*np.sign(T)
                u[i] = T
        return u

class usvKinematics:
    def __init__(self, alpha=0):
        # Initial state conditions
        self.state = np.array([
            [P.e0],     #e  
            [P.n0]    #n
        ]) 

        # sample rate at which the dynamics are propagated
        self.Ts = P.Ts  

        #propeller to center , m
        self.r = P.r * (1.+alpha*(2.*np.random.rand()-1.))


    def update(self, psi, vel_u,vel_v):
        # This is the external method that takes the input u at time
        # t and returns the output n at time t.
        # saturate the input torque
        u = [psi,vel_u,vel_v]
        self.rk4_step(u)  # propagate the state by one time sample
        e,n = self.h()  # return the corresponding output

        return e,n

    def f(self, state, u):
        # Return edot = f(e,u), the system state update equations
        # re-label states for readability
        """
        psi = 0 is along with y positive axis. The angle direction is clockwise.
        """
        #计算力矩
        e = state.item(0)
        n = state.item(1)
        psi = u[0]
        vel_u = u[1]
        vel_v = u[2]
        
        vel_total = np.sqrt(vel_u**2+vel_v**2)
        psi_w = courseLimitation(psi + np.arctan2(vel_v,vel_u)*180/np.pi)
        edot = vel_total*np.sin(psi_w*np.pi/180)
        ndot = vel_total*np.cos(psi_w*np.pi/180)
        statedot = np.array([[edot],
                         [ndot]])

        
        return statedot

    def h(self):
        # return the output equations
        # could also use input u if needed
        e = self.state.item(0)
        n = self.state.item(1)
        #n = np.array([[theta]])
        p = [e,n]
        
        return p

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)


def updateCallback(data):
    global T_list
    T_list = [data.T_l,data.T_r]


# if __name__ == "__main__":
#     usv = usvDynamics()
#     usv_kine = usvKinematics()
#     T_l = 0
#     T_r = 0
#     psi = 0
#     e = 0
#     n = 0
#     T_list = [T_l,T_r]
#     msg = Command()
#     rospy.init_node("usvDynamics")
#     pub1 = rospy.Publisher("/topic_IMU",Command,queue_size=10)
#     pub2 = rospy.Publisher("/topic_GPS",Command,queue_size=10)
#     r = rospy.Rate(100)
#     while not rospy.is_shutdown():
#         rospy.Subscriber("/thrustTopic",Command,updateCallback)
#         print("T_l is %f, T_r is: %f"%(T_list[0],T_list[1]))
#         psi,v = usv.update(T_list)
#         print("msg.v is :%f"%(v))
#         msg.v = v
#         p = usv_kine.update(psi, v)
#         msg.psi = psi
        
#         msg.e = p[0]
#         msg.n = p[1]
               
#         pub1.publish(msg)
#         pub2.publish(msg)
#         r.sleep()
    