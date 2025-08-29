#! /usr/bin/env python3

##########DEPENDENCIES#############
import time
import socket
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
# import exceptions
import math
import argparse


import sys
sys.path.append('..')  # add parent directory
import numpy as np
from courseController import courseController,courseChangeJudge,courseTransition
from velocityController import velocityController
from LOSguidance import LOSGuider
from lonlat2coor import geodetic_to_polar, polar_to_cartesian

#########################VARIABLES###############################
loop_counter=0

guider = LOSGuider()
coursecontroller = courseController()
velocitycontroller = velocityController()
# lon and lat
lat0 =  30.301687579 # original point's lat
lon0 = 120.082279139  # original point's lon
lat, lon = lat0, lon0
lat1, lon1 = lat0, lon0 ##lat1 and lon1 is actual lat and lon 
#Local frame
n0,e0 = 0,0 # north and east of path beginning point relative to original point
n1,e1 = -3,-10 # north and east of path ending point relative to original point
#e = e0#vehicle.location.local_frame.east #actual e
#n = n0#vehicle.location.local_frame.north #actual n


psi_d = 19 #yaw * 180/np.pi #degree
psi_d0 = 0 #yaw * 180/np.pi #last moment of psi_d, used for courseTransition,degree
psi = 0

vx_d = 0
vel_u = 0 #P.vel_u0 #0

flag = 0
count = 0
time_clock = 0
t, t_start = 0,0


#########FUNCTIONS#################
def yaw_callback(data):
    global psi
    # note that the original data is between [-360,360]
    #quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    #euler_angle = transformations.euler_from_quaternion(quaternion)
    # print(quaternion)
    psi = data.data
    if data.data > 180:
        psi = data.data - 360
    elif data.data < -180:
        psi = data.data + 360
    psi = -psi # convert to the psi definition that LOS algorithm uses.
    #print(psi)

def gps_callback(data):
    global lat,lon
    if data.longitude>120 and data.latitude>30:
        lon = data.longitude
        lat = data.latitude


##########MAIN EXECUTABLE###########
rospy.init_node("waterball_Realcontrol")

try:
    #while (lat0<30 or lon0<120) and not rospy.is_shutdown():
    #    rospy.Subscriber("fix",NavSatFix,gps_callback)
    #    lat0, lon0 = lat, lon
    #print("\n lat0:%f, lon0:%f"%(lat0,lon0))      
    while not rospy.is_shutdown() and (time_clock<=30000): # 5mins stop automatically.
        rospy.Subscriber("yaw",Float64,yaw_callback)
        rospy.Subscriber("fix",NavSatFix,gps_callback)
        lat1, lon1 = lat, lon #lat1 and lon1 is actual lat and lon 
        az,distance = geodetic_to_polar(lat0,lon0,lat1,lon1)
        e,n = polar_to_cartesian(az,distance)
        
        ## using guider planner.
        psi_d0 = psi_d
        psi_d = guider.update(e0,n0,e1,n1,e,n) 
        alpha_k0 = guider.alpha_k0_cal(e0,n0,e1,n1)

        # apply transition function
        flag,t_start = courseChangeJudge(psi_d0,psi_d,flag,t_start)
        psi_d = courseTransition(flag,psi_d,t,t_start)

        # course control and velocity control
        angular_v_d = coursecontroller.update(psi_d, psi)

        # Decouple thrust and revolve. 
        
        if np.abs(psi-psi_d)<=30:
            count += 1
            #print("I am in!")
            if count==round(3):
                # vx_d = velocitycontroller.update(vel_u_d,vel_u,psi,alpha_k0,)
                #print("I am in and cound==3!")
                angular_v_d = 0
                vx_d = 2
                count = 0
            else:
                vx_d = 1

        elif count!=0:
            count = 0
            vx_d = 0.5
        else:
            vx_d = 0


        twist_msg = Twist()
        pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        twist_msg.linear.y = vx_d
        twist_msg.angular.z = -angular_v_d ## convert to the psi definition that LOS algorithm uses.
        pub.publish(twist_msg)

        # e = vehicle.location.local_frame.east #actual e
        # n = vehicle.location.local_frame.north #actual n

        #print("Moving...")
        #print("vel_u:%f,\n v_x_d:%f,\n v_y_d:%f,\n e:%f,\n n:%f,\n psi_d:%f,\n psi:%f"%(vel_u,vx_d,angular_v_d,e,n,psi_d,psi))
        if loop_counter > 1:
            print("\n vx_d:%f, angular_v_d:%f,\n psi_d:%f, psi:%f"%(vx_d,angular_v_d,psi_d,psi))
            #print("lat1:%f,  lon1:%f"%(lat1,lon1))
            print("e:%f, n:%f, az:%f,  distance:%f"%(e,n,az,distance))
            loop_counter = 0
        time.sleep(0.01)        

        loop_counter += 1
        time_clock += 1
except KeyboardInterrupt:
    pass


# # 退出之前，清除vehicle对象  
print("Close vehicle object")


