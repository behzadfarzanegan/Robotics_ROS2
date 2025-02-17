# Bumperbot: ROS2-Based Mobile Robot Control & Estimation

## Overview
Bumperbot is a **ROS2-based mobile robot** designed for motion control, sensor fusion, and state estimation. This project integrates real-time actuation with **ros2_control**, simulates the robot in **Gazebo**, and implements an **Extended Kalman Filter (EKF)** for precise localization. The robot can be controlled manually via a **joystick** or autonomously using IMU and wheel encoders.

## Key Features
- **URDF-based robot modeling** integrated with `ros2_control`
- **Simulation and validation** in Gazebo
- **Joystick-based teleoperation** via ROS2 interfaces
- **IMU and wheel encoder fusion** using EKF for accurate state estimation
- **Real-time sensor data processing** for motion control

## Installation & Setup
### Prerequisites
Ensure you have **ROS2 (Humble or later)** installed and set up properly:
```bash
source /opt/ros/humble/setup.bash
```

### Clone the Repository
```bash
git clone https://github.com/behzadfarzanegan/Robotics_ROS2.git
cd Robotics_ROS2
```

### Install Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace
```bash
colcon build
source install/setup.bash
```

## Running the Simulation
### Launch Gazebo with the Robot
```bash
ros2 launch bumperbot_description bumperbot_gazebo.launch.py
```

### Spawn Bumperbot at a Custom Position
To set a custom spawn location:
```bash
ros2 launch bumperbot_description bumperbot_gazebo.launch.py spawn_x:=2.0 spawn_y:=3.0 spawn_yaw:=1.57
```

## Teleoperation (Manual Control)
### Using a Joystick
Ensure a **ROS2-compatible joystick** is connected, then launch:
```bash
ros2 launch bumperbot_control joystick_teleop.launch.py
```

## Autonomous Navigation
### Run State Estimation with EKF
```bash
ros2 launch bumperbot_localization ekf_localization.launch.py
```

### Start Motion Controller
```bash
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=true use_python:=true
```

## Contributing
Feel free to contribute by creating **pull requests** or reporting **issues**.

<div style="text-align: center; margin-top: 20px;">
    <video width="45%" controls poster="/images/bumperbot_preview.png">
        <source src="/behzadfarzanegan.github.io/images/bumperbot.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
    <video width="45%" controls>
        <source src="/behzadfarzanegan.github.io/images/bumperbot_control.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
</div>

