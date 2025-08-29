#!/usr/bin/env python3
import rclpy
import tf2_ros
import numpy as np
import random
from geometry_msgs.msg import Point


def generate_random_target():
    """生成随机目标的姿态角和距离"""
    random.seed(1)
    # 姿态角在-90度到90度之间，距离在10米内
    angle = random.uniform(-np.pi/2, np.pi/2)  # 弧度
    distance = random.uniform(5, 10)  # 米
    return angle, distance

def calculate_local_coordinates(angle, distance):
    """计算局部坐标系下的坐标"""
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    return x, y

def convert_to_global_coordinates(x, y,listener):
    """将局部坐标转换为全局坐标"""
    v_base_link = [x, y, 0, 0]  # z坐标设为0，假设在平面内移动
    try:
        # Wait for the listener to get the first transform to ensure accuracy
        listener.waitForTransform("NED", "base_link", rospy.Time(), rospy.Duration(4.0))
        
        (trans, rot) = listener.lookupTransform('NED', 'base_link', rospy.Time(0))
        v_rotated = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(rot, v_base_link),
            tf.transformations.quaternion_conjugate(rot)
        )
        v_NED = [v_rotated[0] + trans[0], v_rotated[1] + trans[1], v_rotated[2] + trans[2]]
        print(f"v_NED[0], v_NED[1]:{v_NED[0]},{v_NED[1]}")

        # Wait for the listener to get the first transform to ensure accuracy
        listener.waitForTransform("world", "base_link", rospy.Time(), rospy.Duration(4.0))
        (trans, rot2) = listener.lookupTransform('world', 'base_link', rospy.Time(0))
        v_rotated = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(rot2, v_base_link),
            tf.transformations.quaternion_conjugate(rot2)
        )
        v_map = [v_rotated[0] + trans[0], v_rotated[1] + trans[1], v_rotated[2] + trans[2]]
        print(f"v_map[0], v_map[1]:{v_map[0]},{v_map[1]}")


        return v_NED[0], v_NED[1],v_map[0],v_map[1]
    except Exception as e:
        print(f"Exception2:{e}")
        print("success22")
        return None, None,None,None

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('robot_motion_tracker')
    pub = rospy.Publisher("object_info",Point,queue_size=10)
    object_info_msg = Point()
    # 主循环
    while not rospy.is_shutdown():
        angle, distance = generate_random_target()
        object_info_msg.x = angle*180/3.14
        object_info_msg.y = distance
        print(f"object_info_msg:{angle},{distance}")
        pub.publish(object_info_msg)
        # x_local, y_local = calculate_local_coordinates(angle, distance)
        # x_global, y_global = convert_to_global_coordinates(x_local, y_local)
        # print(f"Global coordinates: x={x_global}, y={y_global}")
        rospy.sleep(1/10)  # 每秒更新一次
