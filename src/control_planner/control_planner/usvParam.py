#! /usr/bin/env python3
# Inverted Pendulum Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the arm known to the controller
m = 25     # Mass of the arm, kg
g = 9.8       # Gravity, m/s**2
b = 0.5      # Damping coefficient, Nms
b_f = 30  # forward drag
b_r = 0.01 #Y_r

# parameters for animation
length = 1    # length of arm in animation
width = 0.3   # width of arm in animation

# Initial Conditions
theta0 = 0.0  # ,degree
thetadot0 = 0.0         # ,rads/s
e0 = 0 #,m
n0 = 0 #,m
vel_u0 = 0 #m/s
vel_v0 = 0

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 0.05  # End time of simulation
Ts = 0.02  # sample time for simulation
t_plot = 0.033  # the plotting and animation is updated at this rate

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

# saturation limits
tau_max = 100.0                # Max torque, N-m
f_max = 300

cmdW_max = 20 #max degree per second sent to MCU

psi_max = 10000.0 #max psi_d, degree

r = 0.25 #distance between wheel and center

I_z =0.25 #转动惯量I_z

tau_cut = 4 #tau_total cut off
f_cut = 4  #force_total cut off

R_min = 1 # R_min used for calculating R_LOS

T_p_max = 150 #N posive direction max force
T_n_max = 70 #N negative direction max force

t_range = (0, np.pi*2) #(0, 120) 
n_points = 5000