#! /usr/bin/env python3
import sys
from urllib.parse import uses_query
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import rospy


#from dataPlotter import dataPlotter


from pid_course_control.msg import Command
from MFACControl import MFAC_Controller
import MFACParam as P
import random

# instantiate arm, controller, and reference classes


# instantiate the simulation plots and animation
#dataPlot = dataPlotter()






rospy.init_node("sim_MFAC")

pub = rospy.Publisher("/topic_msgs",Command,queue_size=10)

r = rospy.Rate(100)
t_start = rospy.get_time()


y_queue = P.y_queue
u_queue = P.u_queue
phi_queue = [P.phi_0,0.0]

y_new = 0.0
y_dnew = 0.0
y_dnew_temp = 0.0


phi_new = 0.0


k = 3
controller = MFAC_Controller()
while not rospy.is_shutdown():  # main simulation loop
    t = rospy.get_time()
    # # calculate y_dnew
    # if k <= 300:
    #     y_dnew = 0.5*np.power(-1,round(k/500))
    # elif k<=700:
    #     y_dnew = 0.5*np.sin(k*np.pi/100) + 0.3*np.cos(k*np.pi/50)
    # else:
    #     y_dnew = 0.5*np.power(-1,round(k/500))
    
    
    if k % 50 == 0:
        y_dnew_temp = random.uniform(-1,1) 
    elif k % 25 == 0:
        y_dnew = y_dnew_temp * (random.uniform(0.1,0.5)*np.sin(k*np.pi/100) + random.uniform(0.1,0.5)*np.cos(k*np.pi/50))
    else:
        y_dnew =0.5*np.sin(k*np.pi/100) + 0.3*np.cos(k*np.pi/50)


    u_queue = controller.update(y_dnew,y_new)
    
    #system state update
    a = round(k/500)
    if k <= 500:
        y_new = a*y_queue[0]/(1+ y_queue[0]**2) + u_queue[0]**3
    else:
        y_new = (y_queue[0]*y_queue[1]*y_queue[2]*u_queue[1]*(y_queue[2]-1)+a*u_queue[0])/(1+y_queue[1]**2+y_queue[2]**2)
    
    if np.abs(y_new)>100000:
        y_new = np.sign(y_new)*10000

    y_queue[1:3] = y_queue[0:2] 
    y_queue[0] = y_new

    

    msg = Command()
    print(y_new,y_dnew)
    msg.vx = y_new
    msg.vx_d = y_dnew
    # msg.k = k
    pub.publish(msg)

    k += 1
    r.sleep()

print("Done.")
