#! /usr/bin/env python3

import rospy
from pid_course_control.msg import Command

def referenceCallback(data):
    global ref_x, ref_y, signal_f
    ref_x = data.ref_x
    ref_y = data.ref_y
    signal_f = data.force
    #print("sig_reference is :%f"%(sig_reference))
    #print("now in referenceCallback function")

def GPSCallback(data):
    # 航向角
    global x,y
    x = data.x
    y = data.y

def updateCallback(data):
    global T_l, T_r
    T_l = data.T_l
    T_r = data.T_r

def reference_psiCallback(data):
    global psi_d
    psi_d = data.psi_d
    #print("sig_reference is :%f"%(sig_reference))
    #print("now in referenceCallback function")

def IMUCallback(data):
    global psi,v
    psi = data.psi
    v = data.v

def forceCallback(data):
    global force
    force = data.force

def tauCallback(data):
    global tau
    tau = data.tau

def msgDetect(pub,msg,e,n,psi,psi_d,tau,force,v,v_d,T_l,T_r):
    msg.x = e
    msg.y = n
    msg.T_l = T_l
    msg.T_r = T_r
    msg.psi_d = psi_d
    msg.psi = psi
    msg.tau = tau
    msg.force = force
    msg.v = v
    msg.v_d = v_d
        
    pub.publish(msg)

if __name__ == "__main__":
    ref_x,ref_y =0,0
    x,y = 0,0
    T_l,T_r = 0,0
    psi_d, psi = 0,0
    tau, force = 0,0
    v = 0
    signal_f = 0

    rospy.init_node("msgDetect")
    msg = Command()
    pub = rospy.Publisher("/topic_allmsgs",Command,queue_size=10)
    r = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        rospy.Subscriber("/signals",Command,callback=referenceCallback) #get ref_x,ref_y,signal_f
        rospy.Subscriber("/topic_GPS",Command,callback=GPSCallback) # get x,y
        rospy.Subscriber("/thrustTopic",Command,updateCallback) #get T_l, T_r
        rospy.Subscriber("/psi_topic",Command,callback=reference_psiCallback) #get psi_d
        rospy.Subscriber("/topic_IMU",Command,callback=IMUCallback) #get psi
        rospy.Subscriber("/force_topic",Command,callback=forceCallback) # get force from velocityController
        rospy.Subscriber("/tau_topic",Command,callback=tauCallback) #get tau from courseController

        msg.ref_x = ref_x
        msg.ref_y = ref_y
        msg.reference = signal_f #note that 'reference' refers to signal_f
        msg.x = x
        msg.y = y
        msg.T_l = T_l
        msg.T_r = T_r
        msg.psi_d = psi_d
        msg.psi = psi
        msg.tau = tau
        msg.force = force
        msg.v = v
        
        pub.publish(msg)
        r.sleep()

