#! /usr/bin/env python3


import numpy as np
import rclpy
from control_planner import usvParam as P
from pid_interfaces.msg import Command

class windGenerator:
    def __init__(self, theta0=0.0, force0=0.0):
        self.theta = theta0 #global coordinate system
        self.force = force0
        self.wind_u = [0,0]
    def theta_update(self, t, mode=1,amplitude=0.0,y_offset=0.0): #
        if mode == 1:
            return self.step(t,amplitude, y_offset)
        elif mode == 2:
            return self.random(t,amplitude, y_offset)
        else:
            return self.theta

    def force_update(self,t,mode=1, amplitude=0.0,y_offset=0.0):
        if mode == 1:
            return self.step(t,amplitude, y_offset)
        elif mode == 2:
            return self.random(t,amplitude, y_offset)
        else:
            return self.force
    
    def update(self,t):
        self.theta = 30*3.14/180 #self.theta_update(t)
        self.force = 1 #self.force_update(t)
        
        self.wind_u[0] = self.force*np.cos(self.theta) #x in map
        self.wind_u[1] = self.force*np.sin(self.theta)
        return self.wind_u

    def step(self, t, amplitude, y_offset):
        if t >= 0.0:
            out = amplitude + y_offset
        else:
            out = y_offset
        return out

    def random(self,t, amplitude, y_offset):
        out = np.random.normal(y_offset, amplitude)
        return out


def signal_publisher(signal_list,msg,pub):

    msg.reference = signal_list[0]
    msg.force = signal_list[1]
    print("msg.force is : ----------------------- %f"%msg.force)
    msg.ref_x = signal_list[2]
    msg.ref_y = signal_list[3]

    pub.publish(msg)
    #print("msg.ref_x is :%f"%(msg.ref_x))
    #print("msg.ref_y is :%f"%(msg.ref_y))
    #time.sleep(1) #time delay

