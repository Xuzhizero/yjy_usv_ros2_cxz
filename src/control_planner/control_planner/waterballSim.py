#! /usr/bin/env python3
from cmath import tau
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
from math import sqrt
from scipy import optimize

import usvParam as P
from usvDynamics import usvDynamics,usvKinematics

from signalGenerator import signalGenerator
from LOSguidance import adaptive_LOSGuider, classical_LOSGuider
from thrustAllocation import thrustAllocation
#from dataPlotter import dataPlotter
from msgDetect import msgDetect

from pid_course_control.msg import Command

from windGenerator import windGenerator

from pid_course_control.scripts.waterballControlBasisV6 import WaterballSubscriber,Waterball_Psi_d_Generator,angle_diff,Waterball_50

import time

def main_loop():
    r = rospy.Rate(1/P.Ts)#100
    t_start = rospy.get_time()
    t = P.t_start  # time starts at t_start
    flag, count = 0, 0
    loop_counter = 0
    time_clock, t, t_start = 0, 0, 0
    # instantiate the simulation plots and animation
    #dataPlot = dataPlotter()

    flag = 0

    e0,n0 = 0,0 # -2,-1 #initial east, north
    e1, n1 = 0,0 #80,-20 #final east, north
    e = P.e0 #actual e
    n = P.n0 #actual n

    # psi_d = 120
    psi_d0 = 0 #last moment of psi_d, used for courseTransition
    psi = P.theta0 #0

    vel_u_d = 1
    vel_u = 0#P.vel_u0 #0

    force_d = 0
    time.sleep(3)
    print("==============================Node starts==============================")
    while not rospy.is_shutdown():  # main simulation loop
        t += P.Ts#rospy.get_time()
        # Psi_d_Generator.state_update()
        Sub.state_update()
        wind_u = wind.update(t)
        # psi_d0 = psi_d

        # psi_d = guider.update(e0,n0,e1,n1,e,n) # using guider planner.
        
        # alpha_k0 = guider.alpha_k0_cal(e0,n0,e1,n1)

        # course control and velocity control
        n_noise = 0#noise.random()  # simulate sensor noise       

        psi_d = Psi_d_Generator.psi_d_update()
        tau_d, force_d = Ball.move()

        # allocate thrust
        T_l, T_r = thrustAllocation(tau_d,force_d,mode=2)
        #T_l, T_r = 10,-10
        d_noise_l = 0 #disturbance.random()  # input disturbance on left propeller
        d_noise_r = 0 #disturbance.random()  # input disturbance
        T_list = [T_l, T_r]
        # T_list = [0,0]
        
        psi,psi_dot,vel_u,vel_v = usv.update(T_list,wind_u,Ball.angular_v_d) # kinetics update
        Ball.set_psi(psi)
        Ball.set_psi_dot(psi_dot)
        Ball.set_vx(vel_u)
        #vel_v = 0
        e,n = usv_kine.update(psi, vel_u,vel_v) # kinematics update
        Ball.set_position(e,n)
        
        
        if loop_counter > 100:
            print("force_d:",force_d)
            print("tau_d,",tau_d)
            print("vel_u:%f,vel_v:%f\n e:%f,n:%f\n,psi_d:%f\n,psi:%f"%(vel_u,vel_v,e,n,psi_d,psi))
            #print("lat1:%f,  lon1:%f"%(lat1,lon1))
            print("mode:",Ball.mode)
            # print(f"Ball.e_list[-1]:{Ball.e_list[-1]},Ball.n_list[-1]:{Ball.n_list[-1]}")
            loop_counter = 0   

        ## used for publish and later visualization
        Ball.Basis.visualize_expected_path(Ball.e_list,Ball.n_list)
        Ball.Basis.visualize_actual_path(n, e)
        Ball.Basis.baselink_NED_transform(n, e, psi)

        # publish states
        Ball.state_Pub() #topic name:/topic_state


        r.sleep()
        loop_counter += 1

if __name__ == "__main__":
    rospy.init_node("waterballSim")
    # Define global variables
    usv = usvDynamics()
    usv_kine = usvKinematics()
    guider = classical_LOSGuider()
    # velocitycontroller = velocityController()

    wind = windGenerator()

    psi_reference = signalGenerator(amplitude=30*np.pi/180.0,frequency=0.05)
    force_reference = signalGenerator(amplitude=50,frequency=0.05)
    ref_x_reference = signalGenerator(amplitude=10,frequency=0.05)
    ref_y_reference = signalGenerator(amplitude=10,frequency=0.05)

    disturbance = signalGenerator(amplitude=5)
    noise = signalGenerator(amplitude=5)
    Ball = Waterball_50()
    Sub = WaterballSubscriber(Ball)

    Psi_d_Generator = Waterball_Psi_d_Generator(Ball)

    main_loop()
    print("Done.")
