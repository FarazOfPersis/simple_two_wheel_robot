#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        
        # Parameters for lowpass filter
        self.declare_parameter('alpha', 0.9)  # Alpha for Complementary Filter style lowpass
        self.alpha = self.get_parameter('alpha').value
        
        # Bias Correction Variables (simple running average during startup)
        self.bias_accel = np.array([0.0, 0.0, 0.0])
        self.bias_gyro = np.array([0.0, 0.0, 0.0])
        self.calibration_samples = 100
        self.sample_count = 0
        self.calibrated = False
        
        # Filtered State
        self.filtered_accel = np.array([0.0, 0.0, 0.0])
        self.filtered_gyro = np.array([0.0, 0.0, 0.0])

        # Subscribers and Publishers
        self.imu_sub = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data_raw',  # Your IMU topic
            self.imu_callback,
            10
        )
        self.imu_pub = self.create_publisher(Imu, '/filtered_imu', 10)
        
        self.get_logger().info('IMU Filter Node started and calibrating...')

    def imu_callback(self, msg):
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        if not self.calibrated:
            # === Bias Correction (Calibration) ===
            if self.sample_count < self.calibration_samples:
                self.bias_accel += accel
                self.bias_gyro += gyro
                self.sample_count += 1
                return
            else:
                self.bias_accel /= self.calibration_samples
                self.bias_gyro /= self.calibration_samples
                # Subtract gravity (approx. 9.81 m/s^2) from the Z component
                self.bias_accel[2] -= 9.81  
                self.calibrated = True
                self.get_logger().info(f'IMU Calibrated. Accel Bias: {self.bias_accel}, Gyro Bias: {self.bias_gyro}')
        
        # === Bias Subtraction ===
        accel -= self.bias_accel
        gyro -= self.bias_gyro

        # === Lowpass Filter (Simple Exponential Smoothing) ===
        self.filtered_accel = self.alpha * self.filtered_accel + (1.0 - self.alpha) * accel
        self.filtered_gyro = self.alpha * self.filtered_gyro + (1.0 - self.alpha) * gyro

        # Publish filtered data
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.linear_acceleration.x = self.filtered_accel[0]
        filtered_msg.linear_acceleration.y = self.filtered_accel[1]
        filtered_msg.linear_acceleration.z = self.filtered_accel[2]
        filtered_msg.angular_velocity.x = self.filtered_gyro[0]
        filtered_msg.angular_velocity.y = self.filtered_gyro[1]
        filtered_msg.angular_velocity.z = self.filtered_gyro[2]
        
        # NOTE: Covariance fields are intentionally left blank/zero for simplicity
        self.imu_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()