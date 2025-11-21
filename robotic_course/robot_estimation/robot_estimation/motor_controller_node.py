#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.6)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers (Matching the topics used in your gz_bridge.yaml)
        # These topics now publish rad/s, not RPM.
        self.left_rpm_pub = self.create_publisher(Float64, '/left_motor_rpm', 10)
        self.right_rpm_pub = self.create_publisher(Float64, '/right_motor_rpm', 10)
        
        self.get_logger().info('Motor Controller Node (Inverse Kinematics) started')
    
    def cmd_vel_callback(self, msg):
        v = msg.linear.x    # Linear velocity (m/s)
        omega = msg.angular.z # Angular velocity (rad/s)
        
        # Inverse Kinematics: Calculate required wheel linear velocities (m/s)
        v_left = v - (omega * self.wheel_separation / 2.0)
        v_right = v + (omega * self.wheel_separation / 2.0)
        
        # Convert to Angular Velocity (rad/s): omega = v / r
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        # Prepare messages
        left_msg = Float64()
        left_msg.data = omega_left 
        
        right_msg = Float64()
        right_msg.data = omega_right
        
        # Publish the angular velocity commands (rad/s)
        self.left_rpm_pub.publish(left_msg)
        self.right_rpm_pub.publish(right_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()