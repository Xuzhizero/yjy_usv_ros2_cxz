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
# lon and lat
lat0 = 30.27528365 # original point's lat
lon0 = 120.122144428 # original point's lon

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

    #print(psi)

def gps_callback(data):
    global lat,lon
    if data.longitude>120 and data.latitude>30:
        lon = data.longitude
        lat = data.latitude

##########MAIN EXECUTABLE###########
rospy.init_node("origin_mark")

while (lat0>30 and lon0>120) and not rospy.is_shutdown():
    rospy.Subscriber("fix",NavSatFix,gps_callback)
    lat0, lon0 = lat, lon
print("\n lat0:%f, lon0:%f"%(lat0,lon0)) 
