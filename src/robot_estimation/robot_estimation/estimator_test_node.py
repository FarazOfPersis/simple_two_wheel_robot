#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import QuaternionStamped
import math
import time

# Helper function (should match the one used in your Odometry node, if possible)
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
    q[3] = cr * cp * cy + sr * sp * sy # w
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
        
        # --- Test State ---
        self.odom_received = False
        self.orientation_received = False
        self.odom_result = None
        self.orientation_result = None
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Estimator Test Node started. Ready to publish mock data...')

    # --- Callbacks to Capture Results ---
    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_result = msg
            self.odom_received = True

    def orientation_callback(self, msg):
        if not self.orientation_received:
            self.orientation_result = msg
            self.orientation_received = True

    # --- Test Logic ---
    def timer_callback(self):
        # Only run the tests once the results are captured
        if self.odom_received and self.orientation_received:
            self.timer.cancel()
            self.run_tests()
            self.destroy_node() # Shut down the node after testing
            return
        
        # If results are not yet received, publish mock data to trigger the estimation nodes
        self.publish_mock_data()

    def publish_mock_data(self):
        now = self.get_clock().now().to_msg()
        
        # === Test Case 1: Odometry (Forward Movement) ===
        # Assuming wheels moved exactly 1.0 radian (0.1m linear distance)
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        # Initial wheel position is 0, setting to 1.0 rad means 1.0 rad displacement
        joint_msg.position = [1.0, 1.0] 
        self.joint_pub.publish(joint_msg)

        # === Test Case 2: IMU Filter and Complementary Filter (Stationary with Gravity) ===
        # Raw IMU data (perfectly still, showing only gravity)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'zed_imu_link' 
        
        # Gravity in Z (approx 9.81 m/s^2) - this is the input to your IMU filter
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81 
        
        # Zero angular velocity (perfectly still)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        self.imu_pub.publish(imu_msg)


    def run_tests(self):
        self.get_logger().info("--- Running Filter Tests ---")

        # --- Test 1: Odometry Accuracy (Forward Movement) ---
        # Parameters: wheel_radius = 0.1, displacement = 1.0 rad
        # Expected displacement: dx = r * d_theta = 0.1 * 1.0 = 0.1 m
        odom_x = self.odom_result.pose.pose.position.x
        expected_x = 0.1
        
        # Tolerance for float comparison
        tolerance = 1e-3 

        if abs(odom_x - expected_x) < tolerance:
            self.get_logger().info(f"Odom Test Passed: X Position is {odom_x:.3f} (Expected {expected_x:.3f})")
        else:
            self.get_logger().error(f"Odom Test FAILED: X Position is {odom_x:.3f} (Expected {expected_x:.3f})")
            
        # Check Yaw is zero
        odom_yaw_q = self.odom_result.pose.pose.orientation.z
        if abs(odom_yaw_q) < tolerance:
            self.get_logger().info(f"Odom Test Passed: Yaw (Quaternion Z) is close to zero.")
        else:
            self.get_logger().error(f"Odom Test FAILED: Yaw is {odom_yaw_q:.3f} (Expected near 0)")

        # --- Test 2: Complementary Filter Stability (Zero Tilt) ---
        # Since the input IMU is stationary and level, the output orientation should be (0, 0, 0)
        # The filter must run long enough to converge, but the static input should converge fast.
        
        orient_q = self.orientation_result.quaternion
        
        # Expected quaternion for (0, 0, 0) Euler: q = [0, 0, 0, 1]
        expected_q = quaternion_from_euler(0, 0, 0) # [0.0, 0.0, 0.0, 1.0]

        q_err = (abs(orient_q.x - expected_q[0]) + 
                 abs(orient_q.y - expected_q[1]) + 
                 abs(orient_q.z - expected_q[2]) + 
                 abs(orient_q.w - expected_q[3]))
                 
        if q_err < 0.1: # Higher tolerance needed as filter takes time to converge
            self.get_logger().info(f"Complementary Filter Test Passed: Orientation is stable (q_err: {q_err:.3f}).")
        else:
            self.get_logger().error(f"Complementary Filter Test FAILED: Orientation drift (q_err: {q_err:.3f}). Check bias correction and filter alpha.")


def main(args=None):
    # NOTE: To run this test, you must launch it alongside your:
    # 1. OdometryNode
    # 2. ImuFilterNode
    # 3. ComplementaryFilterNode
    # Otherwise, this test node will fail because the topics won't be available.
    rclpy.init(args=args)
    node = EstimatorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()