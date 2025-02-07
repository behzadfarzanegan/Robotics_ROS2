#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np


class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        # Subscriptions and publications
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)

        # State [x, y, theta]
        self.state_ = np.array([0.0, 0.0, 0.0])  # Initial state: x, y, theta
        self.covariance_ = np.eye(3) * 100.0    # Initial uncertainty

        # Motion model noise
        self.motion_noise_ = np.diag([0.1, 0.1, 0.05])  # Variance for x, y, theta

        # Measurement noise
        self.measurement_noise_odom_ = np.diag([0.2, 0.2, 0.1])  # Odometry noise: x, y, theta
        self.measurement_noise_imu_ = 0.1  # IMU noise for angular velocity

        self.last_time_ = self.get_clock().now()  # Track time for dt
        self.imu_angular_velocity_z_ = 0.0  # Latest angular velocity from IMU

    def odomCallback(self, odom):
        # Get current time and calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_).nanoseconds * 1e-9  # Convert to seconds
        self.last_time_ = current_time

        # Odometry measurements
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        odom_theta = self.get_yaw_from_quaternion(odom.pose.pose.orientation)

        # Predict state
        linear_velocity = odom.twist.twist.linear.x
        self.statePrediction(linear_velocity, self.imu_angular_velocity_z_, dt)

        # Update state with odometry measurements
        measurement = np.array([odom_x, odom_y, odom_theta])
        self.measurementUpdateOdometry(measurement)

        # Publish updated odometry
        kalman_odom = odom
        kalman_odom.pose.pose.position.x = self.state_[0]
        kalman_odom.pose.pose.position.y = self.state_[1]
        kalman_odom.pose.pose.orientation = self.quaternion_from_yaw(self.state_[2])
        self.odom_pub_.publish(kalman_odom)

    def imuCallback(self, imu):
        # Update angular velocity from IMU
        self.imu_angular_velocity_z_ = imu.angular_velocity.z

    def statePrediction(self, linear_velocity, angular_velocity, dt):
        """Predict the next state."""
        theta = self.state_[2]  # Current orientation

        # Motion model
        dx = linear_velocity * np.cos(theta) * dt
        dy = linear_velocity * np.sin(theta) * dt
        dtheta = angular_velocity * dt

        # Update state
        self.state_[0] += dx
        self.state_[1] += dy
        self.state_[2] += dtheta

        # Normalize theta to [-pi, pi]
        self.state_[2] = np.arctan2(np.sin(self.state_[2]), np.cos(self.state_[2]))

        # Update covariance
        self.covariance_ += self.motion_noise_

    def measurementUpdateOdometry(self, measurement):
        """Update state based on odometry measurements."""
        # Kalman Gain
        K = self.covariance_ @ np.linalg.inv(self.covariance_ + self.measurement_noise_odom_)

        # Update state and covariance
        self.state_ += K @ (measurement - self.state_)
        self.covariance_ = (np.eye(3) - K) @ self.covariance_

    def get_yaw_from_quaternion(self, q):
        """Convert quaternion to yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        """Convert yaw to quaternion."""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = np.cos(yaw / 2)
        q.z = np.sin(yaw / 2)
        return q


def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
