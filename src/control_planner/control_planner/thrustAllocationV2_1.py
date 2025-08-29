#! /usr/bin/env python3
from asyncore import loop
from tkinter import E

import rclpy
import numpy as np

from std_msgs.msg import String
import time

from pid_interfaces.msg import Command
from control_planner import usvParam as P0
from control_planner.waterballControlBasisV9_2 import Waterball_50



def exponential_response_curve(initial_rpm, target_rpm, T=0.025):
    """
    Generate an exponential response curve based on initial and target RPM.

    Parameters:
    - initial_rpm: Initial RPM value.
    - target_rpm: Target RPM value.
    - T: Time constant for the exponential curve. Default is 0.025s.

    Returns:
    - Function representing the response curve.
    """
    B = initial_rpm - target_rpm
    A = target_rpm
    
    def response_curve(t):
        return A + B * np.exp(-t/T)
    
    return response_curve

def thrust_alloc(tau,force,mode=1): # tau&force unit is not normal and should be transferred.
    if mode == 1:
        if abs(tau)<P0.tau_cut and abs(force)<P0.f_cut: #to avoid meaningless oscillation
            T_l = 0
            T_r = 0

        else:#得到左右推力T_l,T_r
            T_l = (force + tau/P0.r)/2
            T_r = (force - tau/P0.r)/2
        return T_l, T_r
    
    if mode == 2: # assume that f_tau itself would not exceed limit.
        f_tau = tau/(2*P0.r) #f_tau is the force on the left.
        if np.abs(f_tau)>P0.T_n_max:
            f_tau = P0.T_n_max * np.sign(f_tau)
        f_force = force/2 
        pwm_l = f_tau + f_force
        pwm_r = -f_tau + f_force
        return pwm_l,pwm_r





def pwm2thrust(current_thrust,cmd_pwm,sim_t,k=10):
    initial_thrust = current_thrust 
    target_thrust = cmd_pwm * k
    curve = exponential_response_curve(initial_thrust, target_thrust)
    response = curve(sim_t)
    return response

def pwm2rpm(current_rpm,cmd_pwm,sim_t,k=500):
    initial_rpm = current_rpm 
    target_rpm = cmd_pwm * k
    curve = exponential_response_curve(initial_rpm, target_rpm)
    response = curve(sim_t)
    return response
def thrust2rpm(thrust): #将推力转化为转速
    # parameters come from curve fitting
    if thrust > 0:
        A = -0.055023
        B = 24.061722
        C = 482.60205        
    else:
        A = -0.164427
        B = 43.918084
        C = 485.87280
    rpm = A*thrust**2+B*thrust+C


    return rpm


def rpm2thrust(rpm): #将转速转化为推力
    # parameters come from curve fitting
    if rpm > 0:
        A = -0.055023
        B = 24.061722
        C = 482.60205        
    else:
        A = -0.164427
        B = 43.918084
        C = 485.87280
    # thrust = (-B + np.sqrt(B**2 - 4*A*(C-rpm))) / (2*A)
    thrust = 0.1*rpm/3

    return thrust

# 输入：pwm，当前推力，转动惯量和B都是关键字参数
# 函数体内：
# tau_drive = pwm*k1
# dtheta/dt = 当前推力*k2
# alpha= ....
# 下一时刻新的dtheta/dt = 上一时刻的+ alpha*dt
# 新推力=(dtheta/dt)/k2
# 输出：新推力
# J \frac{d^2 \theta}{dt^2} = \tau_{\text{drive}} - B \frac{d\theta}{dt}
def thrust_cal(pwm,cur_omega,J=0.025,B=0.1,dt=0.01):
    k1 = 100
    k2 = 400
    tau_drive = pwm*k1
    alpha = (tau_drive - B * cur_omega) / J # the alpha of thrust.
    new_omega = cur_omega + alpha * dt  # using the previous acceleration
    new_thrust =new_omega/k2 *9.8
    return new_thrust,new_omega





# if __name__ == '__main__':
#     rospy.init_node("thrustAllocation")
#     r = rospy.Rate(100)
#     loop_counter = 0
#     Ball = Waterball_50()
#     cur_thrust = 0
#     cur_omega = 0
#     pwm = 0
#     while not rospy.is_shutdown():
#         mode = Ball.Sub.keyboard_subscribe()
#         if mode == "DirectionKey":
#             pwm = Ball.DirectionKey_dic["Right"]
#             new_thrust,new_omega = thrust_cal(pwm,cur_omega)
#             cur_thrust = new_thrust
#             cur_omega = new_omega
#             Ball.angular_v = cur_thrust
#             Ball.Basis.state_Pub() #topic name:/topic_state
#         loop_counter += 1
#         if loop_counter>10:
#             print(f"pwm:{pwm}")
#             print(f"cur_thrust:{cur_thrust}")
#             print(f"cur_omega:{cur_omega}")
#             loop_counter = 0
#         r.sleep()




