#! /usr/bin/env python3
import numpy as np
from control_planner import usvParam as P0
from control_planner import PIDParam as P
from control_planner.PIDControl import PIDControl
from control_planner.SplaneControl import SplaneControl
from control_planner.courseController import courseLimitation

from pid_interfaces.msg import Command

import rclpy
from rclpy.node import Node


class velocityController:    #速度控制器
    def __init__(self):
        # Instantiates the PD object
        #self.thetaCtrl = PIDControl(P.kp, P.ki, P.kd, P0.tau_max, P.beta, P.Ts)
        self.thetaCtrl = SplaneControl(P.kp, P.ki, P.kd, P0.f_max, P.beta, P.Ts)
        self.limit = P0.f_max

    def update(self, v_r, y,psi,alpha,judge=100):
        v = y

        # compute feedback linearized torque tau_fl
        #tau_fl = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)

        # compute the linearized torque using PD
        e = courseLimitation(psi-alpha)
        f_tilde = self.thetaCtrl.PID(v_r, v, True) #S plane control

        # compute total torque
        #tau = tau_fl + tau_tilde
        f = f_tilde
        f = self.saturate(f)

        return f

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)

        return u
    
    def force_publisher(self, force):
        print("Now force is publishing...")
        
        pub = rospy.Publisher("/force_topic",Command,queue_size=10)

        msg = Command()
        
        msg.force = force

        pub.publish(msg)
        print("msg.force is :%f"%(msg.force))


def signal_callback(data):
    #print("data.force is:%f"%(data.force))
    controller.force_publisher(data.force)

if __name__ == '__main__':
    # instantiate controller, and reference classes
    controller = velocityController()
    # 订阅参考输入信息
    #subscribe reference
    rospy.init_node("velocitycontroller",anonymous=True)
    rospy.Subscriber("/signals",Command,signal_callback)
    #reference = signalGenerator(amplitude=30*np.pi/180.0,frequency=0.05)
    #订阅IMU信息
    #subscriber IMU information
    #输入到PID控制器中，输出值为力矩
    #force = controller.update(r, y) 
    #publish电压信号
    #publish force
    
    rospy.spin()
