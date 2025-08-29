import numpy as np
class PID_LocTypeDef:
    def __init__(self, Kp, Ki, Kd, MaxSum, MaxResult):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.MaxSum = MaxSum
        self.MaxResult = MaxResult
        self.Ek = 0
        self.Ek1 = 0
        self.LocSum = 0


def pid_loc(set_value, actual_value, value_gradient, pid):
    pid.Ek = set_value - actual_value
    pid.LocSum += pid.Ki * pid.Ek * 0.01
    
    # Clamp the LocSum to between -MaxSum and MaxSum
    pid.LocSum = min(max(pid.LocSum, -pid.MaxSum), pid.MaxSum)
    
    derivative = pid.Kd * value_gradient
    
    # Clamp the derivative to between -MaxResult/3 and MaxResult/3
    derivative = min(max(derivative, -pid.MaxResult/3), pid.MaxResult/3)
    
    pid_loc = pid.Kp * pid.Ek + pid.LocSum + derivative
    pid.Ek1 = pid.Ek
    
    # Clamp the PIDLoc to between -MaxResult and MaxResult
    pid_loc = min(max(pid_loc, -pid.MaxResult), pid.MaxResult)
    
    return pid_loc


# # Example usage:
# pid = PID_LocTypeDef(Kp=1.0, Ki=0.1, Kd=0.01, MaxSum=100, MaxResult=10)
# set_value = 50
# actual_value = 40
# value_gradient = 1.5
# pid_result = pid_loc(set_value, actual_value, value_gradient, pid)
# print(pid_result)
