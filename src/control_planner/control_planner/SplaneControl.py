#! /usr/bin/env python3
from distutils.log import error
import numpy as np
from control_planner import usvParam as P
from control_planner import PIDParam

class SplaneControl:
    def __init__(self, kp, ki, kd, error_range, e_dot_limit, u_max, limit):
        self.kp = kp                 # Proportional control gain
        self.ki = ki                 # Integral control gain
        self.kd = kd                 # Derivative control gain
       
        self.beta = PIDParam.beta             # gain for dirty derivative
        self.Ts = PIDParam.Ts               # sample rate

        self.error_range = error_range
        self.e_dot_limit = e_dot_limit
        self.u_max = u_max
        self.limit = limit           # The output will saturate at this limit

        self.y_dot = 0.0             # estimated derivative of y
        self.y_d1 = 0.0              # Signal y delayed by one sample
        self.error = 0.0
        self.error_dot = 0.0         # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator
        self.f_dl = 0.0
        self.u_f = 0.0

        self.T_p_max = P.T_p_max #N posive direction max force
        self.T_n_max = P.T_n_max #N negative direction max force
        

    def normalize_error(self, error, error_range):
        error = error/error_range 
        return error

    def limit_e_dot(self, e_dot, e_dot_limit):
        e_dot = self.beta * e_dot + (1 - self.beta) * ((self.error - self.error_d1) / self.Ts)
        if e_dot > e_dot_limit:
            e_dot = e_dot_limit
        elif e_dot < -e_dot_limit:
            e_dot = -e_dot_limit
        return e_dot

    def scale_output(self, u, u_max):
        if u >= 0:
            u_unsat = u * u_max
        else:
            u_unsat = u * u_max
        return u_unsat

    def PD(self, error):
        '''
            PD control,
        '''
        # Pre-processing
        self.error = self.normalize_error(error, self.error_range)
        self.error_dot = self.limit_e_dot(self.error_dot, self.e_dot_limit)

        # PD control
        u_unsat = -self.kp*self.error - self.kd*self.error_dot
        f = 2/(1+np.exp(u_unsat)) - 1 
        u_unsat = self.scale_output(f, self.u_max)
        
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        
        # update delayed variables
        self.error_d1 = self.error
        return u_sat

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
