#!/usr/bin/env python3
# coding: utf-8
####
#Especially for interactive mode
###

## V8 is modified for mode structure changes.
import time
import rospy

import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from waterballControlBasisV8 import Waterball_50


# Main function
def main_loop():
    flag, count = 0, 0
    loop_counter = 0
    time_clock, t, t_start = 0, 0, 0
    vx_d, vel_u = 0, 0
    i = 0
    try: 
        while not rospy.is_shutdown() and (time_clock<=30000): # 5mins stop automatically.
            #Compute psi_d.
            Ball.Sub.state_update(mode="Real")
            # psi_d = Psi_d_Generator.psi_d_update()

            # course control and velocity control
            Ball.Mode.mode_judge()

            # Ball.Basis.publish_cmd_vel(Ball.vx_d, Ball.angular_v_d)
            Ball.Basis.publish_v_w_force() # If the cmd_vel contains v_force& w_force rather than vx_d& angular_v_d, use this rather than the line above.
            
            ## used for publish and later visualization
            Ball.Basis.visualize_expected_path(Ball.e_list,Ball.n_list)
            Ball.Basis.visualize_actual_path(Ball.n, Ball.e)

            Ball.Basis.baselink_NED_transform(Ball.n, Ball.e, Ball.psi)
            # publish states
            Ball.Basis.state_Pub() #topic name:/topic_state
            
            ## used for printing neccessary info.
            if loop_counter > 10:
                print("\n vx_d:%f, angular_v_d:%f,\n psi_d:%f, psi:%f"%(Ball.vx_d,Ball.angular_v_d,Ball.psi_d,Ball.psi))
                print(f"angular_v:{Ball.angular_v}")
                print("lat:%f,  lon:%f"%(Ball.lat,Ball.lon))
                print("e:%f, n:%f, e1:%f,  n1:%f"%(Ball.e,Ball.n,Ball.e1,Ball.n1))
                print("mode:",Ball.mode)
                print(f"steady_state:{Ball.steady_state}")
                print(f"distance:{Ball.distance}")
                print(f"w_force:{Ball.w_force}, v_force:{Ball.v_force}")
                loop_counter = 0

            loop_counter += 1
            time_clock += 1
            
            # time.sleep(0.01)   
            r.sleep()
    except KeyboardInterrupt:
        pass

def initalization():
    # 创建一个发布器对象
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # 给rospy一些时间来创建发布器
    rospy.sleep(1)

    # 创建一个PoseWithCovarianceStamped消息对象
    initial_pose = PoseWithCovarianceStamped()
    
    # 设置消息的头部
    initial_pose.header.frame_id = 'world'

    # 设置机器人的位置
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    
    # 设置机器人的方向
    initial_pose.pose.pose.orientation.x = 1.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 0.0

    # 设置协方差 (这只是一个示例值)
    initial_pose.pose.covariance[0] = 0.0685
    initial_pose.pose.covariance[7] = 0.0685
    initial_pose.pose.covariance[35] = 0.0685

    # 发布消息
    pub.publish(initial_pose)
    rospy.loginfo("Initial pose set!")

if __name__ == "__main__":
    rospy.init_node("waterball_RealcontrolV7")
    r = rospy.Rate(50)
    # Basic class init

    Ball = Waterball_50()
    # Ball.Sub.get_lon0lat0(mode="Real")
    # main_loop
    main_loop()
    # # 退出之前，清除vehicle对象  
    for i in range(10):
        Ball.w_force = 0
        Ball.v_force = 0
        Ball.Basis.twist_msg.linear.y = Ball.v_force
        Ball.Basis.twist_msg.angular.z = Ball.w_force
        Ball.Basis.pub_v_w_force.publish(Ball.Basis.twist_msg)    
        time.sleep(0.1)
    print("Close waterball object")

