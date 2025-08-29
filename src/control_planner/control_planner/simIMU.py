#! /usr/bin/env python3
import rospy
from msg import Command
#模拟IMU发布信息
if __name__ == "__main__":
    pub = rospy.Publisher("/topic_IMU",Command,queue_size=10)
    msg = Command()
    r = rospy.Rate(100)
    msg.psi = 0
    t = P.t_start
    while (t<P.t_end ):
        pub.publish(msg) 
        r.sleep()
        print(t)
        t+=P.Ts
    print("loop gets end")    