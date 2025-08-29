#! /usr/bin/env python3
import rospy
from pid_course_control.msg import Command
import usvParam as P


import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation


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

def init_func():
    plt.xlim(0,100)
    plt.ylim(0,50)

def update_plot(i):
    global x,y
    #t_next_plot = t + P.t_plot
    #while t < t_next_plot: 
    #rospy.Subscriber("/topic_IMU",Command,IMUCallback)
    rospy.Subscriber("/signals",Command,refCallback)
    rospy.Subscriber("/topic_GPS",Command,GPSCallback)

    #msg.psi = psi
    #pub.publish(msg)
    r.sleep()
    ax.scatter(x,y,marker='o', color='r')
    

    #self.text_pt.set_text("x=%.2f, y=%.2f"%(self.x_list[i],self.y_list[i]))


if __name__== "__main__":
    print("usvplot.py is running...")
    
    x,y,ref_x,ref_y = 0,0,0,0
    psi = 0
    sig_ref = 0
    x_list = []
    y_list = []

    msg = Command()
    
    rospy.init_node("plot_node2",anonymous=True)
    r = rospy.Rate(1)
    #pub = rospy.Publisher("/usvplot2_topic",Command,queue_size=10)

    t = P.t_start  # time starts at t_start
    


    #print(x_list[0:10])

    #x_list = np.linspace(0,2*np.pi,250)
    #y_list = np.sin(x_list)
    #print(y_list[0:10])
    fig = plt.figure(tight_layout=True)
    ax = plt.subplot(1,1,1)
    #point_ani, = plt.plot(x_list[0],y_list[0],"ro")
    ani = animation.FuncAnimation(fig,update_plot, init_func=init_func, interval=100)
    plt.show()    


      

        
