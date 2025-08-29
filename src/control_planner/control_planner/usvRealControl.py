##########DEPENDENCIES#############
from dronekit import connect, VehicleMode,LocationLocal,APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil

from rover_velocity_based_movement import connectMyCopter,arm

import sys
sys.path.append('..')  # add parent directory
import numpy as np
from courseController import courseController,courseChangeJudge,courseTransition
from velocityController import velocityController
from LOSguidance import LOSGuider

#########FUNCTIONS#################

def reverse(direction):
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, #command
		0, #confirmation
		direction, #Param 1, 0 for forward 1 for backward.
		0,  #Param 2, yaw speed deg/s
		0, #Param 3, Direction -1 ccw, 1 cw
		0, # Param 4, relative offset 1, absolute angle 0
		0,0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()




def backup(): ##rough function to easily reverse without needing to use a GPS navigation based movement
	vehicle.mode = VehicleMode("MANUAL")
	while vehicle.mode!='MANUAL':
		print("Waiting for drone to enter MANUAL flight mode")
		time.sleep(1)
	vehicle.channels.overrides = {'2':1400}
	time.sleep(1)
	vehicle.channels.overrides = {'2':1500}

	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)

def send_local_ned_velocity(vx, vy, vz, vehicle):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	time.sleep(0.5)
	#vehicle.flush()

##########MAIN EXECUTABLE###########
vehicle = connectMyCopter()
arm(vehicle)
print("vehicle battery:%s"%vehicle.battery)
loop_counter=0

guider = LOSGuider()
coursecontroller = courseController()
velocitycontroller = velocityController()
#Local frame
n0,e0 = 10,-100#27.8693348,121.1059648#initial east, north
n1,e1 = -10,-100#27.8691016,121.1057721 #final east, north
e = vehicle.location.local_frame.east #actual e
n = vehicle.location.local_frame.north #actual n


psi_d = vehicle.attitude.yaw * 180/np.pi #degree
psi_d0 = vehicle.attitude.yaw * 180/np.pi #last moment of psi_d, used for courseTransition,degree
psi = vehicle.attitude.yaw * 180/np.pi #0

vel_u_d = 0.5
vel_u = vehicle.velocity[0] #P.vel_u0 #0

force_d = 0

flag = 0
count = 0
try:
    while True and loop_counter<500:
        psi_d0 = psi_d
        psi_d = guider.update(e0,n0,e1,n1,e,n) # using guider planner.
        alpha_k0 = guider.alpha_k0_cal(e0,n0,e1,n1)

        # apply transition function
        #flag,t_start = courseChangeJudge(psi_d0,psi_d,flag,t_start)
        #psi_d = courseTransition(flag,psi_d,t,t_start)

        # course control and velocity control
        xxxx = coursecontroller.update(psi_d, psi)
        angular_v_d = np.sign(xxxx)*10 # course control 
        print("coursecontroller output:%f"%xxxx)

        # Decouple thrust and revolve. 
        if np.abs(psi-psi_d)<=30:
            count += 1
            print("I am in!")
            if count==round(3):
                # vx_d = velocitycontroller.update(vel_u_d,vel_u,psi,alpha_k0,)
                print("I am in and cound==3!")
                angular_v_d = 0
                vx_d = 100
                count = 0
            else:
                vx_d = 100

        elif count!=0:
            count = 0
            vx_d = 100
        else:
            vx_d = 100

        send_local_ned_velocity(vx_d,angular_v_d,0,vehicle)
        e = vehicle.location.local_frame.east #actual e
        n = vehicle.location.local_frame.north #actual n

        psi = vehicle.attitude.yaw * 180/np.pi
        vel_u = vehicle.velocity[0]
        print("Moving...")
        print("vel_u:%f,\n v_x_d:%f,\n v_y_d:%f,\n e:%f,\n n:%f,\n psi_d:%f,\n psi:%f"%(vel_u,vx_d,angular_v_d,e,n,psi_d,psi))
        #print(type(vel_u),type(vx_d),type(angular_v_d),type(e),type(psi))
        time.sleep(1)        

        loop_counter += 1
except KeyboardInterrupt:
    pass


# # 退出之前，清除vehicle对象  
print("Close vehicle object")
vehicle.close()


