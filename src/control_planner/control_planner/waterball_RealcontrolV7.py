#!/usr/bin/env python3
# coding: utf-8
####
#Especially for interactive mode
####
import time
import rospy


import sys
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


from waterballControlBasisV7 import WaterballSubscriber,Waterball_Psi_d_Generator,Waterball_50,angle_diff

# Add the parent directory to the path
sys.path.append('..')


# Main function
def main_loop():
    flag, count = 0, 0
    loop_counter = 0
    time_clock, t, t_start = 0, 0, 0
    vx_d, vel_u = 0, 0
    i = 0
    try: 
        while not rospy.is_shutdown() and (time_clock<=30000): # 5mins stop automatically.
            #Compute psi_d.
            Sub.state_update()
            psi_d = Psi_d_Generator.psi_d_update()

            # course control and velocity control
            w_force, v_force = Ball.move()
            


            ## used for publish and later visualization
            Ball.Basis.visualize_expected_path(Ball.e_list,Ball.n_list)
            Ball.Basis.visualize_actual_path(Ball.n, Ball.e)
            Ball.Basis.publish_cmd_vel(v_force, w_force)
            Ball.Basis.baselink_NED_transform(Ball.n, Ball.e, Ball.psi)
            # publish states
            Ball.state_Pub() #topic name:/topic_state
            
            ## used for printing neccessary info.
            if loop_counter > 100:
                print("\n v_force:%f, w_force:%f,\n psi_d:%f, psi:%f"%(v_force,w_force,psi_d,Ball.psi))
                #print("lat1:%f,  lon1:%f"%(lat1,lon1))
                print("e:%f, n:%f, e1:%f,  n1:%f"%(Ball.e,Ball.n,Ball.e1,Ball.n1))
                print("mode:",Ball.mode)
                loop_counter = 0
            time.sleep(0.01)        

            loop_counter += 1
            time_clock += 1
    except KeyboardInterrupt:
        pass



if __name__ == "__main__":
    rospy.init_node("waterball_RealcontrolV7")
    # Basic class init

    Ball = Waterball_50()
    Sub = WaterballSubscriber(Ball)
    Sub.get_lon0lat0()
    Psi_d_Generator = Waterball_Psi_d_Generator(Ball)

    # main_loop
    main_loop()
    # # 退出之前，清除vehicle对象  
    print("Close waterball object")

