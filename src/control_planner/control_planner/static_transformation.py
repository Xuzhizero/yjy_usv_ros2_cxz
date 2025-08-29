#!/usr/bin/env python3
# coding: utf-8

# Neccessary library
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np

rospy.init_node("static_transformation")

static_transformStamped = geometry_msgs.msg.TransformStamped()

broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "world"
static_transformStamped.child_frame_id = "NED"

static_transformStamped.transform.translation.x = 10.0
static_transformStamped.transform.translation.y = 5.0
static_transformStamped.transform.translation.z = 2.0

quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi/2)
static_transformStamped.transform.rotation.x = quat[0]
static_transformStamped.transform.rotation.y = quat[1]
static_transformStamped.transform.rotation.z = quat[2]
static_transformStamped.transform.rotation.w = quat[3]

broadcaster.sendTransform(static_transformStamped)

x,y,z = 1,2,3
vector = [x,y,z]
v_base_link= [vector[0], vector[1],vector[2], 0] 
listener = tf.TransformListener()
loop_counter = 0
r = rospy.Rate(100)
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('world', 'NED', rospy.Time(0))
        v_map = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(rot, v_base_link), tf.transformations.quaternion_conjugate(rot))
        v_new = v_map[0:3]
        if loop_counter > 100:
            print(f"v_new:{v_new}")
            loop_counter = 0
        loop_counter += 1
    except Exception as e:
        print(f"Exception:{e}")
    r.sleep()





