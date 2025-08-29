#! /usr/bin/env python3
from distutils.log import error
import numpy as np
import rclpy
from control_planner import usvParam as P0
from control_planner import PIDParam as P
from control_planner.PIDControl import PIDControl
from control_planner.SplaneControl import SplaneControl
from control_planner.loc_PIDControl import pid_loc, PID_LocTypeDef

from pid_interfaces.msg import Command

class CourseController:    #航向控制器
    def __init__(self):
        # Instantiates the PD object
        self.thetaCtrl = PIDControl(2, 1.0, 0.4, 200, P.beta, P.Ts)
        # self.thetaCtrl = SplaneControl(P.kp, P.ki, P.kd, 180, 0.33, P0.cmdW_max,P0.cmdW_max)
        self.limit = P0.cmdW_max
        # self.PID_LOC = PID_LocTypeDef(Kp=0.3, Ki=0.001, Kd=0, MaxSum=20, MaxResult=250)

    def set_pid(self, p, i, d):
        """设置新的PID参数"""
        self.thetaCtrl = PIDControl(p, i, d, 200, P.beta, P.Ts)


    def update(self, theta_r, y,mode=2):
        if mode == 1:
            value_gradient = self.PID_LOC.Ek-self.PID_LOC.Ek1
            angular_v_d = pid_loc(theta_r, y, value_gradient, self.PID_LOC) #Using loc PID algorithm
            return angular_v_d
        elif mode == 2 :
            #theta = y.item(0)
            theta = y  
            error = theta_r - theta #theta_r - theta is between [-360,360]
            # consider period. For example, 1 degree (theta_r) - 359 degree（theta）should be 2 degree rather than -358 degree.
            # Through process below, error can be limited within [-180,180] 
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            # print("error",error)
            # cmdW_tilde = self.thetaCtrl.PID(theta_r, theta, True)
            # cmdW_tilde = self.thetaCtrl.PD(error) #S plane control
            cmdW_tilde = self.thetaCtrl.PID(error) #pid plane control
            # compute total torque
            #tau = tau_fl + tau_tilde
            cmdW = cmdW_tilde
            if np.abs(error)>150:
                gear = 3
            elif np.abs(error)> 90:
                gear = 2
            else:
                gear = 1
            if self.limit == 6:
                gear = 0
            cmdW = self.saturate(cmdW,gear)
            
            return cmdW
        else:
            pass

    def saturate(self, u, gear):
        if gear == 3:
            self.limit = 20
        elif gear ==2:
            self.limit = 10
        elif gear == 1:
            self.limit = 8
        else:
            self.limit = 6
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u




def transitionJudge(psi_d0,psi_d,flag,t_start,threshold=30): #degree
    if psi_d - psi_d0 > threshold:
        return 1,rospy.get_time() #desired angle increase suddenly
    elif psi_d - psi_d0 < -threshold:
        return 2,rospy.get_time() #desired angle decrease suddenly
    else:
        return flag,t_start #normal

def slow_transition(flag,psi_d,t,t_start,t_dur=6,k=0.5): #psi:degree, k is used for adjusting the beginning of the psi_d
    #course transition function
    if flag == 0:
        psi_d_new = psi_d
    elif flag == 1:
        if t_start<= t <= t_dur + t_start:
            T_b = 0.5 *(1-np.cos((t-t_start)/t_dur*np.pi))
            psi_d_new = k*psi_d + (1-k)*T_b*psi_d
        else:
            T_b = 1
            psi_d_new = psi_d
    else:
        if t_start<= t <= t_dur + t_start:
            T_s = 0.5 *(1+np.cos((t-t_start)/t_dur*np.pi))
            psi_d_new = k*psi_d + (k-1)*T_s*psi_d    
        else:
            T_s = 0
            psi_d_new = psi_d
    return psi_d_new 

def courseLimitation(course): #degree to degree interval [-180,180)
    course = course * np.pi/180
    course = course - 2*np.pi*np.floor((course+np.pi)/(2*np.pi))
    course_deg = course*180/np.pi
    return course_deg


def referenceCallback(data):
    global psi_d
    psi_d = data.psi_d
    #print("sig_reference is :%f"%(sig_reference))
    #print("now in referenceCallback function")

def IMUCallback(data):
    # 航向角
    global psi
    psi = data.psi
    


if __name__ == '__main__':
    psi_d = 0
    psi = 0
    # instantiate controller, and reference classes
    controller = courseController()
    
    rospy.init_node("coursecontroller",anonymous=True)
    r = rospy.Rate(300)
    pub = rospy.Publisher("/tau_topic",Command,queue_size=10)
    msg = Command()
    while not rospy.is_shutdown():
        # 订阅参考输入信息
        #reference = signalGenerator(amplitude=30*np.pi/180.0,frequency=0.05)
        rospy.Subscriber("/psi_topic",Command,callback=referenceCallback)

        #订阅IMU信息
        rospy.Subscriber("/topic_IMU",Command,callback=IMUCallback) #订阅的话题名字需要问松哥  

            #输入到PID控制器中，输出值为力矩
        print("psi_d is: %f, psi is: %f"%(psi_d, psi))
        tau = controller.update(psi_d, psi) 
        #publish电压信号
        controller.tau_publisher(tau,msg,pub)
        r.sleep()
    

    


