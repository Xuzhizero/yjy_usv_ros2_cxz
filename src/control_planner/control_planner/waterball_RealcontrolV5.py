#!/usr/bin/env python3
# coding: utf-8
####
#Especially for going forward.
####
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import argparse
import sys
import numpy as np
from courseController import courseController, courseChangeJudge, courseTransition
from velocityController import velocityController
from LOSguidance import classical_LOSGuider,adaptive_LOSGuider
from lonlat2coor import geodetic_to_polar, polar_to_cartesian

from pid_course_control.scripts.waterballControlBasisV6 import WaterballControlBasis,WaterballSubscriber

# Add the parent directory to the path
sys.path.append('..')


# Main function
def main_loop():
    flag, count = 0, 0
    loop_counter = 0
    time_clock, t, t_start = 0, 0, 0
    lat0, lon0, lat, lon = 0, 0, 0, 0
    
    n_list = [n1, 10, 10]
    e_list = [e1, 0, -15]
    psi_d, psi_d0, psi = 19, 0, 0
    vx_d, vel_u = 0, 0
    try:
        while (lat0<30 or lon0<120) and not rospy.is_shutdown():
            lat0, lon0 = WaterballSub.gps_subscribe()
            psi = WaterballSub.yaw_subscribe()
        print("\n lat0:%f, lon0:%f"%(lat0,lon0))   
        n1, e1 = np.cos(psi), np.sin(psi)
        n0, e0 = 0, 0
        i = 0    
        while not rospy.is_shutdown() and (time_clock<=30000): # 5mins stop automatically.
            psi = WaterballSub.yaw_subscribe()
            lat, lon = WaterballSub.gps_subscribe()
            lat1, lon1 = lat, lon #lat1 and lon1 is actual lat and lon 
            az,distance = geodetic_to_polar(lat0,lon0,lat1,lon1)
            e,n = polar_to_cartesian(az,distance)
            ## using guider planner.  
            psi_d0 = psi_d
            psi_d = guider.update(e0,n0,e1,n1,e,n) 
            # alpha_k0 = guider.alpha_k0_cal(e0,n0,e_list[i],n_list[i]) #used for velocity control

            # apply transition function
            #flag,t_start = courseChangeJudge(psi_d0,psi_d,flag,t_start)
            #psi_d = courseTransition(flag,psi_d,t,t_start)

            # course control and velocity control
            angular_v_d = coursecontroller.update(psi_d, psi)

            # Decouple thrust and revolve. 
            if np.abs(psi-psi_d)<=10:
                count += 1
                if count==round(3):
                    # vx_d = velocitycontroller.update(vel_u_d,vel_u,psi,alpha_k0,)
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

            ## used for publish and later visualization
            Basis.visualize_expected_path()
            Basis.visualize_actual_path(n, e)
            Basis.publish_cmd_vel(vx_d, angular_v_d)
            Basis.baselink_NED_transform(n, e, psi)

            
            ## used for printing neccessary info.
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
    print("Close waterball object")

if __name__ == "__main__":
    rospy.init_node("waterball_RealcontrolV5")
    # Define global variables
    guider = classical_LOSGuider()
    coursecontroller = courseController()
    velocitycontroller = velocityController()
    Basis = WaterballControlBasis()
    WaterballSub = WaterballSubscriber()

    main_loop()

