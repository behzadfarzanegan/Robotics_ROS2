# Bumperbot: ROS2-Based Mobile Robot Control & Estimation

## Overview
Bumperbot is a **ROS2-based mobile robot** designed for motion control, sensor fusion, and state estimation. This project integrates real-time actuation with **ros2_control**, simulates the robot in **Gazebo**, and implements an **Extended Kalman Filter (EKF)** for precise localization. The robot can be controlled manually via a **joystick** or autonomously using IMU and wheel encoders.

## Key Features
- **URDF-based robot modeling** integrated with `ros2_control`
- **Simulation and validation** in Gazebo
- **Joystick-based teleoperation** via ROS2 interfaces
- **IMU and wheel encoder fusion** using EKF for accurate state estimation
- **Real-time sensor data processing** for motion control

## Control Methodology
Bumperbot implements an **adaptive trajectory tracking control** approach inspired by robust control strategies for car-like vehicles with input constraints. The control algorithm is designed to ensure:
- **Precise trajectory tracking** even under disturbances
- **Adaptive gain tuning** for improved robustness
- **Satisfying input constraints** while maintaining stability

### Control Law
The control system consists of:
1. **Reference Trajectory Generation**: 
   - Desired position and velocity are computed using sinusoidal-based motion profiles.
   - The desired yaw angle and angular velocity are derived accordingly.
2. **Feedback Linearization and Control Gains**: 
   - Proportional-Derivative (PD) control with adaptive gain selection is implemented.
   - Error feedback is used to adjust velocity and yaw rate dynamically.
3. **Extended Kalman Filter (EKF)** for state estimation: 
   - Sensor fusion of **IMU** and **wheel encoders** enhances localization accuracy.
4. **Control Input Constraints**: 
   - Velocity (`v`) and angular velocity (`w`) are constrained within limits to prevent actuator saturation.
   - The control strategy dynamically adjusts gains based on the vehicle's state.

### Control Equations
The control law is given by:

\[ v = V_d \cos(\psi_e) + k_1 x_e \]
\[ w = \frac{k_3 \tilde{y} + 2V_d \sin(\psi_e) + \frac{k_2}{V_d} \cos(\psi_e) \dot{\psi_d} - \frac{k_2}{V_d^2} \sin(\psi_e) \dot{V_d}}{x_e + \frac{k_2}{V_d} \cos(\psi_e)} \]

Where:
- \( V_d \) and \( \dot{V_d} \) are the desired velocity and its time derivative
- \( \psi_e \) is the yaw error
- \( x_e, y_e \) are position errors
- \( k_1, k_2, k_3 \) are adaptive control gains

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
ros2 launch bumperbot_control motion_control.launch.py
```

## Demonstration Videos
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

## Contributing
Feel free to contribute by creating **pull requests** or reporting **issues**.


