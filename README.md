# 🤖 Simple Two Wheel Robot

A ROS-based two-wheeled robot simulation project featuring sensor integration, state estimation, and motion control using Gazebo and RViz.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Robot Specifications](#robot-specifications)
- [Packages Description](#packages-description)
- [Sensor Integration](#sensor-integration)
- [Estimation Pipeline](#estimation-pipeline)
- [Topics](#topics)
- [Visualization](#visualization)
- [License](#license)
- [Contributing](#contributing)
- [Contact](#contact)

---

## 🎯 Overview

This project implements a complete two-wheeled differential drive robot with:
- URDF-based robot modeling 
- Sensor integration (LiDAR and IMU)
- State estimation using complementary filters
- Motion control with differential drive
- Full Gazebo simulation and RViz visualization

---

## ✨ Features

### Core Capabilities
- Differential drive kinematics
- Real-time sensor data processing
- State estimation and odometry
- Configurable motion control
- Complete ROS2 integration

---

## 🛠️ Installation
```bash
# Clone the repository
git clone https://github.com/FarazOfPersis/simple_two_wheel_robot

# Navigate to your ROS workspace
cd ~/ros2_ws/src

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

---

## 🚀 Usage

### Launch Gazebo Simulation

bash
ros2 launch robot_description gazebo.launch.py

**This launches:**
- Gazebo simulation environment
- Robot spawn with URDF/XACRO model
- ROS-Gazebo bridge
- RViz visualization
- Gazebo world file

### Launch Estimation Pipeline

bash
ros2 launch robot_estimation estimator.launch.py

**This launches:**
- Sensor processing nodes
- State estimation filters
- Motion controller
- Odometry publisher

---

## 🔧 Robot Specifications

### Physical Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| **LiDAR Radius** | 0.1 m | LiDAR sensor mounting radius |
| **LiDAR Height** | 0.1 m | LiDAR height from base |
| **Wheel Radius** | 0.1 m | Differential drive wheel radius |
| **Wheel Thickness** | 0.3 m | Wheel width/thickness |

### Robot Configuration Table

| Component | Height (m) | Radius (m) | Thickness (m) | Wheel Thickness (m) | Wheel Radius (m) | LiDAR Radius (m) |
|-----------|------------|------------|---------------|---------------------|------------------|------------------|
| **Robot** | 0.1 | 0.1 | 0.1 | 0.1 | 0.1 | 0.3 |

---

## 📦 Packages Description

### 1. `robot_description` Package

#### Purpose
Contains URDF file and launch configurations for robot visualization and simulation.

#### Key Components

**a) URDF Model**
- Writes a URDF file to define a differential drive robot
- Includes links, joints, inertial properties, and revolute joint configurations

**b) Motion Limits**
- Configures appropriate motion limits (joint limits) using ROS2 standards
- Selects proper joint types: `base_link`, `left_wheel_link`, `right_wheel_link`

**c) Launch File: `gazebo.launch.py`**

Launches the complete Gazebo simulation with:
- `gazebo-ros-bridge(gz_bridge.yaml)`: Communication bridge between Gazebo and ROS2
- `robot_spawn`: Spawns robot model in simulation
- `rviz`: Visualization interface
- `gazebo-sim-launch-with-world`: Gazebo environment with custom world file

**d) Launch File: `display.launch.py`**
- Launches RViz with robot model
- Adds `global_frame` configuration
- Sets `base_link` as the global frame
- Adds URDF file to RViz for robot model visualization

**e) RViz Configuration: `rviz.config.rviz`**
- Saves RViz configuration as `rviz.config.rviz`
- Includes robot model and TF tree visualization
- Stores visualization parameters for consistent display
  
**f) Lidar RViz Configuration: `lidar.config.rviz`**
- Saves RViz configuration as `lidar.config.rviz`
- Includes lidar visualization of the surrounding obstacles in 3d
- Stores visualization parameters for consistent display

---

### 2. `robot_estimation` Package

#### Purpose
Implements sensor processing, filtering, and state estimation for the robot.

#### Components
- Lowpass filters for sensor noise reduction
- Complementary filter for orientation estimation
- Motor controller for wheel velocity commands
- Motion model for differential drive control
- Odometry calculation and publishing

---

## 🔍 Sensor Integration

### LiDAR Sensor

**Configuration:**
- Adds LiDAR sensor to URDF
- Properly configured mounting frame using `frame_id_convertor` node
- Sensor position and orientation configured in URDF
- URDF sensor plugin for Gazebo integration

**Features:**
- Standard ROS topic conventions for LiDAR data
- Correct frame conventions following ROS standards
- Real-time point cloud 2 data

**Topics:**
- Publishes points data on `/scan` topic `/gz_lidar/points`
- Frame transformations properly configured with node `frame_id_convertor`
- Published to standard topic `/points`

**RViz Integration:**
- LiDAR point cloud display in RViz
- Real-time scan visualization
- Proper coordinate frame alignment

---

### IMU Sensor

**Configuration:**
- Integrated IMU sensor in URDF
- Provides acceleration and gyroscope data
- Proper sensor plugin configuration
- Calibrated measurements and orientation

**Data Provided:**
- Linear acceleration (3-axis accelerometer)
- Angular velocity (3-axis gyroscope)
- Raw sensor data for filtering

**Processing:**
- Input to complementary filter
- Noise reduction through lowpass filtering
- Orientation estimation

---

## 🎛️ Estimation Pipeline

### Filters and Controllers

| Component | Description |
|-----------|-------------|
| **Lowpass Filter** | Applied to accelerometer and gyroscope components for noise reduction |
| **Complementary Filter** | Fuses accelerometer and gyroscope data for orientation estimation<br/>**Topic:** `estimation/orientation` |
| **Motor Controller** | Position control with wheel velocity commands<br/>**Topics:** `left_wheel_rpm`, `right_wheel_rpm` |
| **Motion Model** | Differential drive controller with velocity commands<br/>**Topic:** `cmd_vel` |
| **Odometry** | Calculates robot position and publishes odometry<br/>Broadcasts `odom` → `base_link` transform |

### Estimator Launch

**File:** `estimator.launch.py`

Launches all estimation nodes including filters, controllers, and odometry calculation.

---

## 📡 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/points` | `sensor_msgs/msg/PointCloud2` | LiDAR scan data |
| `/zed/zed_node/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU raw data (accel + gyro) |
| `/estimation/orientation` | `geometry_msgs/Quaternion` | Filtered orientation estimate |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for motion control |
| `/left_wheel_rpm` | `std_msgs/Float64` | Left wheel RPM command |
| `/right_wheel_rpm` | `std_msgs/Float64` | Right wheel RPM command |
| `/odom` | `nav_msgs/Odometry` | Robot odometry data |

---

## 🎨 Visualization

### RViz Configuration (`gazebo.rviz`)

**Displays:**
- Robot URDF model
- LiDAR point cloud visualization
- TF frames tree
- Odometry path
- Global and local coordinate frames

**Features:**
- Saved configuration for consistent visualization
- Proper frame references
- Real-time sensor data display

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### How to Contribute
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📧 Contact

For questions or suggestions, please open an .

---

## 🙏 Acknowledgments

- Dr.Motahari and the rest of the teaching team of robotics Fall of 2025 SUT
- ROS2 Community
- Gazebo Simulation Framework
- URDF Tools
