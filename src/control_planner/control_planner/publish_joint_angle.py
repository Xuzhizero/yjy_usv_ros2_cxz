#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_node')
        
        # Publisher for joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber for rudder angle (in degrees)
        self.subscription = self.create_subscription(
            Float64,
            '/rudder_joint_angle',
            self.rudder_angle_callback,
            10)
        
        # Timer: 10Hz
        self.timer = self.create_timer(1/10, self.publish_joint_state)
        
        # Initial joint position (radians)
        self.joint_position = 1 * math.pi/180  # 45 degrees default
        
        self.get_logger().info('Joint state publisher node started')
        self.get_logger().info('Subscribing to /rudder_joint_angle for angle updates (degrees)')
    
    def rudder_angle_callback(self, msg):
        """Callback to update joint position from rudder angle topic"""
        # Convert degrees to radians
        self.joint_position = -msg.data * math.pi/180
        # self.get_logger().info(f'Received rudder angle: {msg.data:.2f}° → {self.joint_position:.4f} rad')
    
    def publish_joint_state(self):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_spherical_center_high_joint']
        msg.position = [self.joint_position]
        
        self.publisher_.publish(msg)
        # Uncomment for debugging:
        # self.get_logger().info(f'Publishing: {msg.name[0]} = {self.joint_position:.4f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()