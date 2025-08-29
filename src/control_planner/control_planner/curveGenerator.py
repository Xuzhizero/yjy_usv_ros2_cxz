#!/usr/bin/env python3
# coding: utf-8

## Generate curve with the form of parameterization. The parameter is t.
import numpy as np

def curve_x(t):
    # return (4*10**(-4)*(t-50)**3+6*10**(-4)*(t-50)**2-2.4*10**(-3)*(t-50)+10)*1/5# x(t)方程
    # return -2+t*(80+2)
    return 10*np.cos(t)+2
def curve_y(t):
    # return t-50+10# y(t)方程
    # return -1+t*(-20+1)
    return 10*np.sin(t)