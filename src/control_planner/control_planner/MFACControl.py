#! /usr/bin/env python3
from urllib.parse import uses_query
import numpy as np
from control_planner import MFACParam as P

class MFAC_Controller:
    def __init__(self,Ball):
        self.Ball = Ball
        self.lam = P.lambda_param
        self.mu = P.mu
        self.eta = P.eta
        self.epsilon = P.epsilon
        self.rho = P.rho

        self.phi_0 = P.phi_0
        self.phi_real = 0.0
        self.phi_queue =  [self.phi_0,0.0]
        self.u_queue = P.u_queue
        self.y_queue = P.y_queue

        self.limit = 200
        self.u_max = 20
        
        self.loop_count = 0

    def scale_output(self, u, u_max):
        if u >= 0:
            u_unsat = u * self.u_max
        else:
            u_unsat = u * self.u_max
        return u_unsat

    def update(self, y_dnew,y): #y_dnew is desired y, y is present actual y
        # y, y_dnew : degree per 0.1 seconds
        y_dnew = y_dnew/27
        y = y/27       

        #update y
        self.y_queue[1] = self.y_queue[0] # self.y_queue[0] is y at last stage
        self.y_queue[0] = y # y is at present stage
 
        # 计算 Δu(k-1) 和 Δr(k)
        delta_u_k_1 = self.u_queue[0]-self.u_queue[1] # u_queue[0] is u at last stage, self.u_queue[1] is u at the stage before last stage
        delta_r_k = self.y_queue[0]-self.y_queue[1] # y_quere[0] is y at this state.


        self.phi_real = self.phi_real if abs(delta_u_k_1) < 0.001 else delta_r_k / delta_u_k_1
        
        # PPD 更新
        phi_new = self.phi_queue[0] + self.eta * delta_u_k_1/(self.mu + delta_u_k_1**2)*(delta_r_k-self.phi_queue[0]*delta_u_k_1)
        # PPD 重置机制
        if np.abs(phi_new)<=self.epsilon or np.abs(delta_u_k_1)<= self.epsilon or np.sign(phi_new)!=np.sign(self.phi_0):
            phi_new = self.phi_0

        self.phi_queue[1] = self.phi_queue[0] #phi_queue[0] is phi at last stage
        self.phi_queue[0] = phi_new # phi at this stage

        # 计算 u(k)
        if (self.Ball.angular_v_d==0 and self.Ball.vx_d==0):
            u_new = 0
            self.y_queue[1], self.y_queue[0] = 0.0, 0.0
            self.u_queue[1], self.u_queue[0] = 0.0, 0.0
        else:
            u_new = self.u_queue[0] + self.rho*self.phi_queue[0]/(self.lam+self.phi_queue[0]**2)*(y_dnew-y)
        self.u_queue[1] = self.u_queue[0]
        self.u_queue[0] = u_new
        # u = self.scale_output(self.u_queue[0],self.u_max)
        # u = self.saturate(self.u_queue[0])
        if u_new>0:
            if u_new>1:
                w_force = (u_new+4)*10
            else:
                w_force = u_new * 10
        elif u_new < 0:
            if u_new < -1:
                w_force = (u_new-1)*10
            else:
                w_force = u_new * 10

        else:
            w_force = 0.0
            
        if np.abs(w_force) > 250:
            w_force = np.sign(w_force)*250

        if self.loop_count > 10:
            print("from MFACControl:")
            print(f"phi_queue[0]:{self.phi_queue[0]}")
            print(f"u_queue[0]:{self.u_queue[0]}")
            print(f"w_force:{w_force}")
            self.loop_count = 0
        self.loop_count += 1
        return w_force,self.u_queue,self.phi_queue,self.phi_real


    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        if abs(u) < 33:
            u = np.sign(u)*33
        return u







