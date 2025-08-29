#! /usr/bin/env python3
import rospy
from pid_course_control.msg import Command
import usvParam as P

from dataPlotter import dataPlotter

import matplotlib.pyplot as plt
import numpy as np

def IMUCallback(data):
    global psi
    psi = data.psi

def refCallback(data):
    #global sig_ref
    #sig_ref = data.reference
    global ref_x,ref_y
    ref_x,ref_y = data.ref_x,data.ref_y

def thrustCallback(data):
    global T_l,T_r
    T_l = data.T_l
    T_r = data.T_r

def GPSCallback(data):
    global x,y
    x,y = data.x,data.y

def LOSCallback(data):
    global psi_d
    psi_d = data.psi_d

if __name__=="__main__":
    print("usvplot.py is running...")
    dataPlot = dataPlotter()
    x,y,ref_x,ref_y = 0,0,0,0
    psi = 0
    psi_d = 0
    sig_ref = 0
    T_l = 0
    T_r = 0
    T_list = [T_l,T_r]
    msg = Command()
    
    rospy.init_node("plot_node",anonymous=True)
    r = rospy.Rate(100)
    pub = rospy.Publisher("/plot_topic",Command,queue_size=10)

    t = P.t_start  # time starts at t_start
    while not rospy.is_shutdown():
        #t_next_plot = t + P.t_plot
        #while t < t_next_plot: 
        rospy.Subscriber("/topic_IMU",Command,IMUCallback)
        rospy.Subscriber("/signals",Command,refCallback)
        rospy.Subscriber("/thrustTopic",Command,thrustCallback)
        rospy.Subscriber("/topic_GPS",Command,GPSCallback)
        rospy.Subscriber("/psi_topic",Command,LOSCallback)
        msg.psi = psi
        msg.psi_d = psi_d
        msg.reference = sig_ref
        msg.T_l = T_l
        msg.T_r = T_r
        pub.publish(msg)
        T_list = [T_l,T_r]
        t = t + 0.1#P.Ts
        y_list = [ref_y,y]
            
        # update animation and data plots
        dataPlot.update(t, psi_d, psi, T_list)

        # the pause causes the figure to display during simulation
        plt.pause(0.01)  
        #r.sleep()

    plt.close()        
    print("plot close")
        
