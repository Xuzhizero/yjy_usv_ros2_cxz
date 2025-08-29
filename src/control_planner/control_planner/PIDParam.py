# Single link arm Parameter File
import numpy as np
# import control as cnt
from control_planner import usvParam as P


Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
tau_max = P.tau_max  # limit on control signal

#  tuning parameters
#tr = 0.8 # part (a)
tr = 0.6  # tuned for fastest possible rise time before saturation.
zeta = 0.90
ki = 10  # integrator gain 0.25

# desired natural frequency
wn = 0.5*np.pi/(tr*np.sqrt(1-zeta**2))
alpha1 = 2.0*zeta*wn
alpha0 = wn**2

# compute PD gains
kp = 1.3 #alpha0*(P.m*P.ell**2)/3.0
kd = 0.2 #(P.m*P.ell**2)/3.0*(alpha1-3.0*P.b/(P.m*P.ell**2))
#kp = 2
#kd = 1

print('kp: ', kp)
print('ki: ', ki)
print('kd: ', kd)



