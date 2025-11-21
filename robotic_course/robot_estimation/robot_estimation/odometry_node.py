#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Robot Parameters (must match MotorControllerNode)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.6)
        
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        
        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Joint State tracking (for calculating velocity)
        self.last_wheel_pos = {'left_wheel_joint': 0.0, 'right_wheel_joint': 0.0}
        self.last_time = self.get_clock().now()

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states', # Assuming Gazebo/Bridge publishes wheel data here
            self.joint_state_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odometry Node started')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        if dt == 0.0:
            return

        try:
            # 1. Get current and previous positions
            r_idx = msg.name.index('right_wheel_joint')
            l_idx = msg.name.index('left_wheel_joint')
            
            curr_pos_R = msg.position[r_idx]
            curr_pos_L = msg.position[l_idx]
            
            prev_pos_R = self.last_wheel_pos['right_wheel_joint']
            prev_pos_L = self.last_wheel_pos['left_wheel_joint']
            
            # 2. Calculate Wheel Displacements (Angular)
            d_theta_R = curr_pos_R - prev_pos_R
            d_theta_L = curr_pos_L - prev_pos_L

            # 3. Calculate Wheel Distances (Linear)
            d_s_R = d_theta_R * self.r
            d_s_L = d_theta_L * self.r

            # 4. Calculate Instantaneous Velocities and Angular Change (Motion Model)
            d_s_avg = (d_s_R + d_s_L) / 2.0
            d_theta_body = (d_s_R - d_s_L) / self.L
            
            # 5. Integrate State (Euler Integration)
            self.x += d_s_avg * math.cos(self.theta + d_theta_body / 2.0)
            self.y += d_s_avg * math.sin(self.theta + d_theta_body / 2.0)
            self.theta += d_theta_body
            
            # Normalize yaw (theta)
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Update State
            self.last_wheel_pos['right_wheel_joint'] = curr_pos_R
            self.last_wheel_pos['left_wheel_joint'] = curr_pos_L
            self.last_time = current_time

            # 6. Publish Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            # Set position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            # Set orientation (from theta)
            q = quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            
            self.odom_pub.publish(odom_msg)

            # 7. Broadcast TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

        except ValueError:
            self.get_logger().error("JointState missing 'right_wheel_joint' or 'left_wheel_joint'")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()