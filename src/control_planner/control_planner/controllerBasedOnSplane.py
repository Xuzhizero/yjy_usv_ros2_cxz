#! /usr/bin/env python3
import numpy as np
import usvParam as P

class SplaneControl:
    def __init__(self, kp, ki, kd, limit, beta, Ts):
        self.kp = kp                 # Proportional control gain
        self.ki = ki                 # Integral control gain
        self.kd = kd                 # Derivative control gain
        self.limit = limit           # The output will saturate at this limit
        self.beta = beta             # gain for dirty derivative
        self.Ts = Ts                 # sample rate

        self.y_dot = 0.0             # estimated derivative of y
        self.y_d1 = 0.0              # Signal y delayed by one sample
        self.error_dot = 0.0         # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator
        self.f_dl = 0.0
        self.u_f = 0.0

        self.T_p_max = P.T_p_max #N posive direction max force
        self.T_n_max = P.T_n_max #N negative direction max force
        self.cmdW_max = P.cmdW_max

    def PID(self, y_r, y, flag=True):
        '''
            PID control,

            if flag==True, then returns
                u = kp*error + ki*integral(error) + kd*error_dot.
            else returns 
                u = kp*error + ki*integral(error) - kd*y_dot.

            error_dot and y_dot are computed numerically using a dirty derivative
            integral(error) is computed numerically using trapezoidal approximation
        '''

        # Compute the current error
        error = y_r - y
        # integrate error
        self.integrator = self.integrator + (self.Ts/2)*(error+self.error_d1)

        # PID Control
        if flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot + (1 - self.beta) * ((error - self.error_d1) / self.Ts)
            # PID control
            u_temp = -self.kp*error - self.kd*self.error_dot
            f = 2/(1+np.exp(u_temp)) - 1            
            self.u_f = self.u_f + self.ki*(self.Ts/2)*(f + self.f_dl)
            if self.u_f >= 0:
                u_unsat = self.u_f * self.cmdW_max
            else:
                u_unsat = self.u_f * self.cmdW_max
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
                             + (1 - self.beta) * ((y - self.y_d1) / self.Ts)
            # PID control
            u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts / self.ki * (u_sat - u_unsat)
        # update delayed variables
        self.f_d1 = f
        self.y_d1 = y
        self.error_d1 = error
        return u_sat
    
    def PD(self, error,y, flag=True):
        '''
            PD control,
            
            if flag==True, then returns
                u = kp*error + kd*error_dot.
            else returns 
                u = kp*error - kd*y_dot.
            
            error_dot and y_dot are computed numerically using a dirty derivative
        '''

        # Compute the current error
        # PD Control
        # normalization
        error = error/180 # error can be limited within [-1,1] through this process.
        if flag is True:
            # differentiate error
            self.error_dot = self.beta * self.error_dot \
                             + (1 - self.beta) * ((error - self.error_d1) / self.Ts)
            # error_dot limitation
            if self.error_dot > 3:
                self.error_dot = 3
            elif self.error_dot < -3:
                self.error_dot = -3
            self.error_dot = self.error_dot/3 # error_dot can be limited within [-1,1] through this process.
            # PD control
            u_unsat = -self.kp*error - self.kd*self.error_dot
            #print("u_unsat:%f"%(u_unsat) )
            f = 2/(1+np.exp(u_unsat)) - 1 #when u_unsat is 0, f is 0,  
            if f >= 0:
                u_unsat = f * self.cmdW_max
            else:
                u_unsat = f * self.cmdW_max
            #print("error:%f,error_dot:%f, u_unsat:%f, f: %f"%(error,self.error_dot,u_unsat,f))
        else:
            # differentiate y
            self.y_dot = self.beta * self.y_dot \
                             + (1 - self.beta) * ((y - self.y_d1) / self.Ts)
            # PD control
            u_unsat = self.kp*error - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        # update delayed variables
        self.error_d1 = error
        self.y_d1 = y
        return u_sat

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







