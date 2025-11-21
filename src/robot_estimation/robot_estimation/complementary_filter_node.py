#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

class ComplementaryFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')
        
        # Filter Parameters
        self.declare_parameter('alpha', 0.98) # Weight for gyroscope (high pass)
        self.alpha = self.get_parameter('alpha').value
        
        # State variables (Roll, Pitch, Yaw)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        
        # Subscribers and Publishers
        self.imu_sub = self.create_subscription(
            Imu,
            '/filtered_imu', # Subscribes to the filtered IMU data
            self.imu_callback,
            10
        )
        # Publishes the orientation estimate
        self.orientation_pub = self.create_publisher(
            QuaternionStamped, 
            '/estimation/orientation', 
            10
        )
        
        self.get_logger().info('Complementary Filter Node started')

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # 1. Accelerometer-derived orientation (Gravity Vector)
        # Note: assuming robot starts level and G is mainly in Z
        # Roll_accel: rotation about X axis
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        roll_accel = math.atan2(accel_y, accel_z)
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # 2. Gyroscope Integration (High Pass)
        # Integration of angular velocity gives angular displacement (delta angle)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        # 3. Complementary Filter Fusion
        # Gyro (Highpass) provides good short-term data (alpha)
        # Accel (Lowpass) provides good long-term data (1-alpha)
        self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_accel
        self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_accel
        # Yaw relies purely on gyro integration (requires external source for absolute correction)

        # 4. Publish Orientation (Quaternion)
        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        orientation_msg = QuaternionStamped()
        orientation_msg.header.stamp = current_time.to_msg()
        orientation_msg.header.frame_id = msg.header.frame_id
        orientation_msg.quaternion.x = q[0]
        orientation_msg.quaternion.y = q[1]
        orientation_msg.quaternion.z = q[2]
        orientation_msg.quaternion.w = q[3]
        
        self.orientation_pub.publish(orientation_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()