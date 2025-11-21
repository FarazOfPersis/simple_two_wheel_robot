#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import QuaternionStamped
import math
import time

# --- Configuration for IMU Test ---
# Number of steps required for ImuFilterNode to complete its 100-sample calibration
N_CALIBRATION_STEPS = 120 

# Setting all mock biases to 0.0 for a simplified, robust stationary test case.
MOCK_ACCEL_BIAS_X = 0.0   
MOCK_ACCEL_BIAS_Y = 0.0  
MOCK_ACCEL_BIAS_Z_SENSOR = 0.0 
MOCK_GYRO_BIAS_Z = 0.0   
GRAVITY = 9.81

# Helper function (RPY to Quaternion)
def quaternion_from_euler(roll, pitch, yaw):
    # RPY to Quaternion conversion for testing
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = sr * cp * cy - cr * sp * sy # x
    q[1] = cr * sp * cy + sr * cp * sy # y
    q[2] = cr * cp * sy - sr * sp * cy # z
    q[3] = cr * cp * cy + sr * sp * cy # w
    return q

class EstimatorTestNode(Node):
    def __init__(self):
        super().__init__('estimator_test_node')
        
        # --- Publishers for Mock Data ---
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/zed/zed_node/imu/data_raw', 10)
        
        # --- Subscribers for Testing Results ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.orientation_sub = self.create_subscription(QuaternionStamped, '/estimation/orientation', self.orientation_callback, 10)
        self.imu_filtered_sub = self.create_subscription(Imu, '/filtered_imu', self.imu_filtered_callback, 10) 
        
        # --- Test State ---
        self.odom_received = False
        self.orientation_received = False
        self.imu_filtered_received = False 
        
        self.odom_result = None
        self.orientation_result = None
        self.filtered_imu_result = None
        
        self.publish_count = 0 
        self.timer = self.create_timer(0.05, self.timer_callback) 
        
        self.get_logger().info('Estimator Test Node started. Ready to publish mock data...')

    # --- Callbacks to Capture Results ---
    def odom_callback(self, msg):
        # We need to wait until some movement has been published before capturing the result
        if self.publish_count >= 10 and not self.odom_received:
            self.odom_result = msg
            self.odom_received = True

    def orientation_callback(self, msg):
        if not self.orientation_received:
            self.orientation_result = msg
            self.orientation_received = True

    def imu_filtered_callback(self, msg):
        # Capture result only after calibration steps are done to get the stable, filtered data
        if self.publish_count >= N_CALIBRATION_STEPS and not self.imu_filtered_received:
            self.filtered_imu_result = msg
            self.imu_filtered_received = True

    # --- Test Logic ---
    def timer_callback(self):
        # Check if all required results are captured
        if self.odom_received and self.orientation_received and self.imu_filtered_received:
            self.timer.cancel()
            self.run_tests()
            self.destroy_node() # Shut down the node after testing
            return
        
        # Publish mock data to trigger the estimation nodes
        self.publish_mock_data()
        self.publish_count += 1

    def publish_mock_data(self):
        now = self.get_clock().now().to_msg()
        
        # === Test Case 1: Odometry (Forward Movement) ===
        # Initialize JointState message
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # CRITICAL FIX: The odometry node attempts to index msg.position, 
        # so we must populate it, even if the position is mock static data (0.0).
        joint_msg.position = [0.0, 0.0] 

        if self.publish_count < 10: 
            # 10 rad/s velocity for the first 10 steps to ensure measurable distance
            joint_msg.velocity = [10.0, 10.0] 
        else:
            # Publish zero velocity after movement for clean stop
            joint_msg.velocity = [0.0, 0.0] 
            
        self.joint_pub.publish(joint_msg)


        # === Test Case 2 & 3: IMU Filter and Complementary Filter (Stationary with Known Bias = 0.0) ===
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'zed_imu_link' 
        
        # Raw IMU input: Bias (0.0) + Gravity (stationary)
        imu_msg.linear_acceleration.x = MOCK_ACCEL_BIAS_X
        imu_msg.linear_acceleration.y = MOCK_ACCEL_BIAS_Y
        imu_msg.linear_acceleration.z = GRAVITY + MOCK_ACCEL_BIAS_Z_SENSOR
        
        # Raw IMU input: Gyro Bias (0.0)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = MOCK_GYRO_BIAS_Z
        
        self.imu_pub.publish(imu_msg)


    def run_tests(self):
        self.get_logger().info("--- Running Filter Tests ---")
        
        # Define tolerances
        tolerance_odom = 1e-3
        tolerance_imu_filt = 0.1 
        tolerance_comp_filt = 0.1 

        # --- Test 1: Odometry Accuracy ---
        odom_x = self.odom_result.pose.pose.position.x
        
        if odom_x > 0.0:
            self.get_logger().info(f"[Odom Test] Passed: X Position is {odom_x:.3f} (Moved forward).")
        else:
            self.get_logger().error(f"[Odom Test] FAILED: X Position is {odom_x:.3f} (Did not move).")
            
        odom_yaw_q = self.odom_result.pose.pose.orientation.z
        if abs(odom_yaw_q) < tolerance_odom:
            self.get_logger().info(f"[Odom Test] Passed: Yaw (Quaternion Z) is close to zero.")
        else:
            self.get_logger().error(f"[Odom Test] FAILED: Yaw is {odom_yaw_q:.3f} (Expected near 0)")

        # --- Test 2: Complementary Filter Stability (Zero Tilt) ---
        orient_q = self.orientation_result.quaternion
        expected_q = quaternion_from_euler(0, 0, 0) # [0.0, 0.0, 0.0, 1.0]

        q_err = (abs(orient_q.x - expected_q[0]) + 
                 abs(orient_q.y - expected_q[1]) + 
                 abs(orient_q.z - expected_q[2]) + 
                 abs(orient_q.w - expected_q[3]))
                 
        if q_err < tolerance_comp_filt: 
            self.get_logger().info(f"[Comp Filter Test] Passed: Orientation is stable (q_err: {q_err:.3f}).")
        else:
            self.get_logger().error(f"[Comp Filter Test] FAILED: Orientation drift (q_err: {q_err:.3f}).")

        # --- Test 3: IMU Filter Bias Correction and Lowpass ---
        filtered_accel = self.filtered_imu_result.linear_acceleration
        filtered_gyro = self.filtered_imu_result.angular_velocity
        
        # Expected Linear Acceleration: [0, 0, G] 
        accel_x_passed = abs(filtered_accel.x) < tolerance_imu_filt
        accel_y_passed = abs(filtered_accel.y) < tolerance_imu_filt
        accel_z_passed = abs(filtered_accel.z - GRAVITY) < tolerance_imu_filt
        
        if accel_x_passed and accel_y_passed and accel_z_passed:
            self.get_logger().info(f"[IMU Filter Test] Passed: Linear Accel (X:{filtered_accel.x:.3f}, Y:{filtered_accel.y:.3f}, Z:{filtered_accel.z:.3f}) is correct (Expected ~0, ~0, ~{GRAVITY:.3f}).")
        else:
            self.get_logger().error(f"[IMU Filter Test] FAILED: Linear Accel (X:{filtered_accel.x:.3f}, Y:{filtered_accel.y:.3f}, Z:{filtered_accel.z:.3f}) is incorrect (Expected ~0, ~0, ~{GRAVITY:.3f}).")
            
        # Expected Angular Velocity (All should be zero after bias removal)
        gyro_x_passed = abs(filtered_gyro.x) < tolerance_imu_filt
        gyro_y_passed = abs(filtered_gyro.y) < tolerance_imu_filt
        gyro_z_passed = abs(filtered_gyro.z) < tolerance_imu_filt

        if gyro_x_passed and gyro_y_passed and gyro_z_passed:
            self.get_logger().info(f"[IMU Filter Test] Passed: Angular Velocity (X:{filtered_gyro.x:.3f}, Y:{filtered_gyro.y:.3f}, Z:{filtered_gyro.z:.3f}) is zeroed out (Expected ~0).")
        else:
            self.get_logger().error(f"[IMU Filter Test] FAILED: Angular Velocity (X:{filtered_gyro.x:.3f}, Y:{filtered_gyro.y:.3f}, Z:{filtered_gyro.z:.3f}) is incorrect. Check gyro bias removal.")


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()