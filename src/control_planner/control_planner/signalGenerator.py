#! /usr/bin/env python3

import numpy as np
import rclpy
from control_planner import usvParam as P
from pid_interfaces.msg import Command

class signalGenerator:
    def __init__(self, amplitude=1.0, frequency=0.001, y_offset=0):
        self.amplitude = amplitude  # signal amplitude
        self.frequency = frequency  # signal frequency
        self.y_offset = y_offset  # signal y-offset

    def square(self, t):
        if t % (1.0/self.frequency) <= 0.5/self.frequency:
            out = self.amplitude + self.y_offset
        else:
            out = - self.amplitude + self.y_offset
        return out

    def sawtooth(self, t):
        tmp = t % (0.5/self.frequency)
        out = 4 * self.amplitude * self.frequency*tmp \
              - self.amplitude + self.y_offset
        return out

    def step(self, t):
        if t >= 0.0:
            out = self.amplitude + self.y_offset
        else:
            out = self.y_offset
        return out

    def random(self,y_offset=0,amplitude=0):
        out = np.random.normal(y_offset, amplitude)
        return out

    def sin(self, t):
        out = self.amplitude * np.sin(2*np.pi*self.frequency*t) \
              + self.y_offset
        return out

def signal_publisher(signal_list,msg,pub):

    msg.reference = signal_list[0]
    msg.force = signal_list[1]
    #print("msg.force is : ----------------------- %f"%msg.force)
    msg.ref_x = signal_list[2]
    msg.ref_y = signal_list[3]

    pub.publish(msg)
    #print("msg.ref_x is :%f"%(msg.ref_x))
    #print("msg.ref_y is :%f"%(msg.ref_y))
    #time.sleep(1) #time delay

if __name__ == '__main__':
    print("Now signal is publishing...")
    rospy.init_node("signal_publisher",anonymous=True)
    pub = rospy.Publisher("/signals",Command,queue_size=10)
    msg = Command()    
    #publish 参考信号
    psi_reference = signalGenerator(amplitude=30*np.pi/180.0,frequency=0.05)
    force_reference = signalGenerator(amplitude=50,frequency=0.05)
    ref_x_reference = signalGenerator(amplitude=10,frequency=0.05)
    ref_y_reference = signalGenerator(amplitude=10,frequency=0.05)
    r = rospy.Rate(50) #10hz
    t = P.t_start
    while not rospy.is_shutdown():
        signal_r  = psi_reference.step(t) * 180.0 / np.pi #reference 
        #print("signal_r is :%f"%(signal_r))
        signal_f  = 0*force_reference.step(t) #reference force
        ref_x = ref_x_reference.step(t)
        ref_y = ref_y_reference.step(t)

        signal_list = [signal_r,signal_f,ref_x,ref_y] #顺序切记不要弄错
        signal_publisher(signal_list,msg,pub)
        print("signal_f is: %f"%signal_f)
        t = t + P.Ts
        r.sleep()