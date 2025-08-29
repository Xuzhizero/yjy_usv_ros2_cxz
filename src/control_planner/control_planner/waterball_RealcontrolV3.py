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
from scipy import optimize
from math import sqrt
from courseController import courseController,courseChangeJudge,courseTransition
from velocityController import velocityController
from LOSguidance import LOSGuider
from lonlat2coor import geodetic_to_polar, polar_to_cartesian
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
# Because of transformations
import tf
import tf_conversions

import tf2_ros
import geometry_msgs.msg

#########################VARIABLES###############################
loop_counter=0

guider = LOSGuider()
coursecontroller = courseController()
velocitycontroller = velocityController()
# lon and lat
lat0 =  0 #30.301833552 # original point's lat
lon0 = 0 #120.081971055  # original point's lon
lat, lon = lat0, lon0
lat1, lon1 = lat0, lon0 ##lat1 and lon1 is actual lat and lon 
#Local frame
n0,e0 = 0,0 # north and east of path beginning point relative to original point
n1,e1 = 0,-15 # north and east of path ending point relative to original point
n_list = [n1,10,10]
e_list = [e1,0,-15]
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

def curve_x(t):
    return (4*10**(-4)*(t-50)**3+6*10**(-4)*(t-50)**2-2.4*10**(-3)*(t-50)+10)*1/5# x(t)方程
    # return -2+t*(80+2)
    return 10*np.cos(t)
def curve_y(t):
    return t-50+10# y(t)方程
    # return -1+t*(-20+1)
    # return 10*np.sin(t)

def min_distance_to_curve(t_range, x1, y1):
    def distance(t):
        return sqrt((curve_x(t) - x1)**2 + (curve_y(t) - y1)**2)

    res = optimize.minimize_scalar(distance, bounds=t_range, method='bounded')
    return res.fun, res.x  # 返回最小距离以及对应的t值   
def psi_cal(e,n,t_min,dt):
    x1, y1 = e,n
    x2, y2 = curve_x(t_min+dt),curve_y(t_min+dt)
    x, y = x2-x1,y2-y1
    # print("========================")
    # print("x,y:",x,y)
    # print("================")
    norm = np.sqrt(x ** 2 + y ** 2)
    cos_theta = y / norm
    theta_rad = np.arccos(cos_theta) # in radians
    theta_deg = np.degrees(theta_rad) # convert to degrees
    if x<0:
        theta_deg = -theta_deg

    return theta_deg
##########MAIN EXECUTABLE###########
rospy.init_node("waterball_RealcontrolV2")

#pub_pose = rospy.Publisher("/topic_msgs_pose",PoseStamped,queue_size=10)
pub_path = rospy.Publisher("/topic_msgs_path",Path,queue_size=10)
msg_path = Path() # msg_path is used for visualizing actual path on rviz.

pub_expect_path = rospy.Publisher("/topic_expect_path",Path,queue_size=10)
msg_epath = Path() # msg_epath is used for visualizing expected path on rviz.

pub_orient = rospy.Publisher("/topic_orient",Odometry,queue_size=1)
#static initial frame transform: world(ENU)->NED
broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped = geometry_msgs.msg.TransformStamped()

static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "world"
static_transformStamped.child_frame_id = "NED"

static_transformStamped.transform.translation.x = 0.0
static_transformStamped.transform.translation.y = 0.0
static_transformStamped.transform.translation.z = 0.0

quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi/2)
static_transformStamped.transform.rotation.x = quat[0]
static_transformStamped.transform.rotation.y = quat[1]
static_transformStamped.transform.rotation.z = quat[2]
static_transformStamped.transform.rotation.w = quat[3]

broadcaster.sendTransform(static_transformStamped)

t_range = (0, 120)  # t的范围.t is not time here, just a parameter.
dt = 0
# 生成路径
n_points = 100
for i in range(n_points):
    t_l = np.linspace(t_range[0],t_range[-1],n_points)
    # theta = 2*math.pi * i / n_points  # 在圆上均匀采样
    x = curve_x(t_l[i])
    y = curve_y(t_l[i])
    # x = curve_x(theta)
    # y = curve_y(theta)

    # 创建位姿，并加入到Path中
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = y
    pose.pose.position.y = x
    # pose.pose.orientation.w = 1.0  # 注意我们只设置了w分量，这意味着旋转为0
    msg_epath.poses.append(pose)
try:
    while (lat0<30 or lon0<120) and not rospy.is_shutdown():
        rospy.Subscriber("fix",NavSatFix,gps_callback)
        lat0, lon0 = lat, lon
    print("\n lat0:%f, lon0:%f"%(lat0,lon0))      
    i = 0    
    while not rospy.is_shutdown() and (time_clock<=30000): # 5mins stop automatically.
        rospy.Subscriber("yaw",Float64,yaw_callback)
        rospy.Subscriber("fix",NavSatFix,gps_callback)
        lat1, lon1 = lat, lon #lat1 and lon1 is actual lat and lon 
        az,distance = geodetic_to_polar(lat0,lon0,lat1,lon1)
        e,n = polar_to_cartesian(az,distance)
        dis2goal = np.sqrt(np.power(e-e_list[i],2)+np.power(n-n_list[i],2))
        if dis2goal<3:
            e0,n0 = e_list[i],n_list[i]
            i += 1
            if i == len(e_list):
                print("already arrive at final target position. Now return back to original point")
                i = 0
        ## using guider planner.
        min_dist, t_min = min_distance_to_curve(t_range, e, n) #t is not time here, just a parameter. min_dist is y_e
        print("t_min:",t_min)
        psi_d = psi_cal(e,n,t_min,dt)    
        print("========================")
        print("psi_d:",psi_d)
        print("================")
        # r_LOS = guider.r_LOS_cal(min_dist)

        psi_d0 = psi_d
        # psi_d = guider.update(e0,n0,e_list[i],n_list[i],e,n) 
        # alpha_k0 = guider.alpha_k0_cal(e0,n0,e_list[i],n_list[i])

        # apply transition function
        #flag,t_start = courseChangeJudge(psi_d0,psi_d,flag,t_start)
        #psi_d = courseTransition(flag,psi_d,t,t_start)

        # course control and velocity control
        angular_v_d = coursecontroller.update(psi_d, psi)

        # Decouple thrust and revolve. 
        psi_diff = psi-psi_d
        if psi_diff < -180:
            psi_diff += 360
        if psi_diff > 180:
            psi_diff -= 360

        if np.abs(psi_diff)<=10:
            count += 1
            #print("I am in!")
            if count==round(3):
                # vx_d = velocitycontroller.update(vel_u_d,vel_u,psi,alpha_k0,)
                #print("I am in and cound==3!")
                angular_v_d = 0
                vx_d = 3
                count = 0
            else:
                vx_d = 2

        elif np.abs(psi-psi_d)<=30 and count!=0:
            count = 0
            vx_d = 1.5
        elif np.abs(psi-psi_d)<=50 and count!=0:
            count = 0
            vx_d = 1            
        else:
            vx_d = 0

        ## used for visualization
        # visualize expected path
        # msg_epose0, msg_epose1 = PoseStamped(),PoseStamped()
        # msg_epose0.pose.position.x, msg_epose0.pose.position.y = n0,e0
        # msg_epose1.pose.position.x, msg_epose1.pose.position.y = n1,e1
        msg_epath.header.frame_id = "NED"
        # msg_epath.poses = [msg_epose0,msg_epose1]
        pub_expect_path.publish(msg_epath)   
        # visualize actual path
        msg_pose = PoseStamped()
        msg_pose.pose.position.x, msg_pose.pose.position.y = n,e
        msg_path.poses.append(msg_pose)
        msg_path.header.frame_id = "NED"
        pub_path.publish(msg_path)

        twist_msg = Twist()
        pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        twist_msg.linear.y = vx_d
        twist_msg.angular.z = -angular_v_d ## convert to the psi definition that LOS algorithm uses.
        pub.publish(twist_msg)
        br = tf2_ros.TransformBroadcaster()
        trans_msg = geometry_msgs.msg.TransformStamped()

        trans_msg.header.stamp = rospy.Time.now()
        trans_msg.header.frame_id = "NED"
        trans_msg.child_frame_id = "base_link"
        trans_msg.transform.translation.x = n
        trans_msg.transform.translation.y = e
        trans_msg.transform.translation.z = 0.0
        quat = tf_conversions.transformations.quaternion_from_euler(np.pi, 0, psi*np.pi/180) #只是为了好看
        trans_msg.transform.rotation.x = quat[0]
        trans_msg.transform.rotation.y = quat[1]
        trans_msg.transform.rotation.z = quat[2]
        trans_msg.transform.rotation.w = quat[3]

        br.sendTransform(trans_msg)
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


