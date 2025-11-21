import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_estimation'
    
    # Define common parameters for the nodes
    robot_params = {
        'wheel_radius': 0.1,
        'wheel_separation': 0.6
    }
    
    # 1. Motor Controller Node (Inverse Kinematics)
    motor_controller_node = Node(
        package=pkg_name,
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen',
        parameters=[robot_params]
    )
    
    # 2. IMU Filter Node (Lowpass and Bias Correction)
    imu_filter_node = Node(
        package=pkg_name,
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[{'alpha': 0.9}] # Lowpass filter strength
    )

    # 3. Complementary Filter Node (Orientation Estimation)
    complementary_filter_node = Node(
        package=pkg_name,
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        parameters=[{'alpha': 0.98}] # Gyro weight
    )

    # 4. Odometry Node (Motion Model Odometry and TF broadcast)
    odometry_node = Node(
        package=pkg_name,
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[robot_params]
    )

    return LaunchDescription([
        motor_controller_node,
        imu_filter_node,
        # complementary_filter_node,
        # odometry_node,
    ])