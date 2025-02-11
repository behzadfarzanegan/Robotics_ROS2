#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
public:
    // Constructor
    KalmanFilter(const std::string &name);

private:
    // ROS 2 Subscribers for Odometry and IMU data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // ROS 2 Publisher to publish fused Odometry data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Variables for the Kalman filter
    double mean_;         // Mean value of the state estimate
    double variance_;     // Variance (uncertainty) of the state estimate
    double imu_angular_z_; // Angular velocity from IMU data (z-axis)
    bool is_first_odom_;  // Flag to check if it's the first odometry message
    double last_angular_z_; // Last angular velocity from odometry
    double motion_;       // The calculated motion or distance moved (can be based on odometry)
    nav_msgs::msg::Odometry kalman_odom_; // The Kalman-filtered odometry message to be published
    
    double motion_variance_ ;
    double measurement_variance_;

    void measurementUpdate();
    void statePrediction();
    
    // Callback functions for subscribing to Odometry and IMU topics
    void odomCallback(const nav_msgs::msg::Odometry &odom);
    void imuCallback(const sensor_msgs::msg::Imu &imu);

};

#endif // KALMAN_FILTER_HPP
