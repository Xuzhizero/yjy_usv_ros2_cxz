#! /usr/bin/env python3
import numpy as np
from scipy import optimize
from math import sqrt
import rclpy
from control_planner import usvParam as P0
from control_planner import PIDParam as P
from control_planner.PIDControl import PIDControl
from control_planner.SplaneControl import SplaneControl
from control_planner.curveGenerator import curve_x, curve_y

from pid_interfaces.msg import Command


class classical_LOSGuider:    
    def __init__(self):
        # Instantiates the PD object
        self.thetaCtrl = PIDControl(P.kp, P.ki, P.kd,
                                    P0.tau_max, P.beta, P.Ts)
        self.limit = P0.psi_max #degree
        self.R_min = P0.R_min

    def alpha_k0_cal(self,e0,n0,e1,n1):
        if n1 == n0:
            return np.sign(e1-e0)*90
        if e1 == e0:
            return 90.1 - np.sign(n1-n0)*90 #0 or 180 degree
        alpha_k0 = np.arctan(np.abs(e1-e0)/np.abs(n1-n0)) * 180/np.pi #degrees
        if (e1-e0)<0 and (n1-n0)>0:
            alpha_k0 = -alpha_k0
        elif (e1-e0)<0 and (n1-n0)<0:
            alpha_k0 = alpha_k0 - 180
        elif (e1-e0)>0 and (n1-n0)<0:
            alpha_k0 = 180 - alpha_k0
        return alpha_k0
    
    def y_e_cal(self,e0,n0,e1,n1,e,n): #calculatae Y-3
        if e1-e0 != 0:
            A = (n1-n0)/(e1-e0)
            B = -1
            C = n0-e0*(n1-n0)/(e1-e0)
            y_e = np.abs(A*e+B*n+C)/np.sqrt(A**2+B**2)
        else:
            y_e = np.abs(e0-e)
        temp = (n0-n1)*e + (e1-e0)*n + e0*n1 - e1*n0 #judge (e,n) is on the left of line or on the right. if temp>0, on the left and y_e<0
        return -np.sign(temp)*y_e
     
    def r_LOS_cal(self,y_e): #if y_e > R_min, r_LOS is y_e, if y_e < R_min, r_LOS is R_min
        r_LOS = 0.5*(1-np.sign(np.abs(y_e)-self.R_min))*(self.R_min-np.abs(y_e))+np.abs(y_e)
        return r_LOS

    def psi_LOS_cal(self,alpha_k0,y_e, R): #calculate psi_LOS, degree
        #dx = np.abs(ref_x-e)
        #dy = np.abs(ref_y-n)
        #psi = np.arctan(dy/dx) * 180/np.pi #angle between point and target, degree
        psi_LOS = alpha_k0+np.arcsin(-y_e/R) * 180/np.pi
        return psi_LOS
    
    def update(self,e0,n0,e1,n1,e,n): 
        """
        description: Given start point, end point, and present position, return new psi_d that water ball should track.
        parameters: start point (e0,n0), end point (e1,n1), and present position (e,n)
        return variable: psi_d
        notes:
        """
        #theta = n.item(0) 
        theta = n  
        # compute feedback linearized torque tau_fl
        #tau_fl = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)

        # compute the linearized torque using PD
        alpha_k0 = self.alpha_k0_cal(e0,n0,e1,n1)
        y_e = self.y_e_cal(e0,n0,e1,n1,e,n)

        r_LOS = self.r_LOS_cal(y_e)
        psi_tilde = self.psi_LOS_cal(alpha_k0,y_e,r_LOS)
        # print("here is in ----------------------------------------------------------------------- ")
        # print("e0:%f,n0:%F,e1:%F,n1:%F,e:%F,n:%F"%(e0,n0,e1,n1,e,n))
        # print("alpha_k0:%f,y_e:%f,r_LOS:%f,psi_tilde:%f"%(alpha_k0,y_e,r_LOS,psi_tilde))
        # print("okk--=====================================")

        # compute total torque
        #tau = tau_fl + tau_tilde
        psi = psi_tilde
        psi = (psi+180)%360-180
        psi_LOS = self.saturate(psi)
        #psi_LOS = courseLimitation(psi_LOS)
        return psi_LOS

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)

        return u

    def min_distance_to_curve(self,t_range, x1, y1):
        def distance(t):
            return sqrt((curve_x(t) - x1)**2 + (curve_y(t) - y1)**2)

        res = optimize.minimize_scalar(distance, bounds=t_range, method='bounded')
        return res.fun, res.x  # 返回最小距离以及对应的t值   
    def psi_cal(self,e,n,t_min,dt):
        x1, y1 = e,n
        x2, y2 = curve_x(t_min+dt),curve_y(t_min+dt)
        x, y = x2-x1,y2-y1
        norm = np.sqrt(x ** 2 + y ** 2)
        cos_theta = y / norm
        theta_rad = np.arccos(cos_theta) # in radians
        theta_deg = np.degrees(theta_rad) # convert to degrees
        if x<0:
            theta_deg = -theta_deg

        return theta_deg


class adaptive_LOSGuider:    
    def __init__(self):
        # Instantiates the PD object
        self.d_ahead_Ctrl = SplaneControl(4 ,P.ki, 4,1,3,2,20) #kp, ki, kd, error_range, e_dot_limit, u_max, limit
        self.limit = P0.psi_max #degree
        self.R_min = P0.R_min
        self.dt = 3

    def min_distance_to_curve(self,t_range, x1, y1):
        def distance(t):
            return sqrt((curve_x(t) - x1)**2 + (curve_y(t) - y1)**2)

        res = optimize.minimize_scalar(distance, bounds=t_range, method='bounded')
        return res.fun, res.x  # 返回最小距离以及对应的t值   
    def psi_LOS_cal(self,e,n,t_min,dt):
        x1, y1 = e,n
        x2, y2 = curve_x(t_min+dt),curve_y(t_min+dt)
        x, y = x2-x1,y2-y1
        norm = np.sqrt(x ** 2 + y ** 2)
        cos_theta = y / norm
        theta_rad = np.arccos(cos_theta) # in radians
        theta_deg = np.degrees(theta_rad) # convert to degrees
        if x<0:
            theta_deg = -theta_deg

        return theta_deg

    def fast_compute_tangent_with_y_axis(self,x, y, t1, dt=1e-3):
        """
        Fast but less accurate method to compute the angle with y-axis of the tangent to the curve at point t1.
        x, y are functions of a single variable t.
        dt is the small change in t to compute finite differences.
        """
        # Compute approximate derivatives
        dx_dt = (x(t1+dt) - x(t1-dt)) / (2*dt)
        dy_dt = (y(t1+dt) - y(t1-dt)) / (2*dt)

        # Compute the direction vector of the tangent
        tangent_vector = np.array([dx_dt, dy_dt])

        # Compute the direction vector of the y axis
        y_axis_vector = np.array([0, 1])

        # Compute the cosine of the angle between the tangent and the y axis
        cosine_angle = np.dot(tangent_vector, y_axis_vector) / np.linalg.norm(tangent_vector)

        # Compute the angle in degrees
        angle = np.arccos(cosine_angle) * 180 / np.pi
        if dx_dt>0:
            pass
        else:
            angle = - angle
        return angle
    # This function calculates the look-ahead distance based on the minimum distance and the curvature
    def compute_d(self,e_psi, A=7, B=0.1, C=40.0):
        """
        Compute lookahead distance d (meters) based on minimum distance to the path and curvature r
        A, B, C are parameters for the sigmoid function
        """     
        
        sigmoid_val = A / (1 + np.exp(-B * (e_psi - C)))
        if e_psi < 5:
            sigmoid_val += 2
        elif e_psi < 10:
            sigmoid_val += 1
        return sigmoid_val

    # This function calculates the parameter t corresponding to the end of the arc of length d
    def calculate_t2(self,x, y, t1, d, dt=0.01):
        t2 = t1
        accumulated_distance = 0.0

        while accumulated_distance < d:
            t2 += dt
            dx = x(t2) - x(t2-dt)
            dy = y(t2) - y(t2-dt)
            accumulated_distance += np.sqrt(dx**2 + dy**2)

        return t2, t2 - t1

    def update(self,psi, t_range,e,n): 
        """
        Description: Given start point, end point, and present position, return new psi_d that water ball should track.
        Parameters: start point (e0,n0), end point (e1,n1), and present position (e,n)
        Return variable: psi_d
        """

        min_dist, t_min = self.min_distance_to_curve(t_range, e, n) #t is not time here, just a parameter. min_dist is y_e
        # print("min_dist:",min_dist)
        if min_dist > 8:
            self.dt = 0
            psi_tilde = self.psi_LOS_cal(e,n,t_min,self.dt)  
        elif min_dist > 2:
            angle = self.fast_compute_tangent_with_y_axis(curve_x,curve_y,t_min) # compute tangent angle to y axis.
            e_psi = np.abs(psi - angle) #
            d_ahead = self.compute_d(e_psi) # compute look ahead distance
            t2, self.dt = self.calculate_t2(curve_x,curve_y,t_min,d_ahead) # 
            psi_tilde = self.psi_LOS_cal(e,n,t_min,self.dt)  
        else:
            angle = self.fast_compute_tangent_with_y_axis(curve_x,curve_y,t_min) # compute tangent angle to y axis.
            e_psi = np.abs(psi - angle) #
            print("angle,e_psi:",angle,e_psi)
            d_ahead = self.compute_d(e_psi) # compute look ahead distance            
            # d_ahead_offset = self.d_ahead_Ctrl.PD(-min_dist)
            # print("d_ahead_offset:", d_ahead_offset)
            
            # d_ahead = d_ahead + d_ahead_offset
            print("d_ahead:", d_ahead)
            
            t2, self.dt = self.calculate_t2(curve_x,curve_y,t_min,d_ahead) # 
            psi_tilde = self.psi_LOS_cal(e,n,t_min,self.dt)  
        # print("here is in ----------------------------------------------------------------------- ")
        # print("t_min:",t_min)
        # print("e0:%f,n0:%F,e1:%F,n1:%F,e:%F,n:%F"%(e0,n0,e1,n1,e,n))
        # print("alpha_k0:%f,y_e:%f,r_LOS:%f,psi_tilde:%f"%(alpha_k0,y_e,r_LOS,psi_tilde))
        # print("okk--=====================================")

        psi = psi_tilde
        psi = (psi+180)%360-180
        psi_LOS = self.saturate(psi)
        #psi_LOS = courseLimitation(psi_LOS)
        return psi_LOS

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)

        return u