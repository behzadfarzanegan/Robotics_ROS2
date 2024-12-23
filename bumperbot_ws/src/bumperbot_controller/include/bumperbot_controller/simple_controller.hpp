#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

class SimpleController : public rclcpp::Node
{
public:
    explicit SimpleController(const std::string &node_name);

private:
    // Callback function for velocity messages
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);
    void jointCallback(const sensor_msgs::msg::JointState &msg);

    // ROS 2 Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Parameters
    double wheel_radius_;
    double wheel_separation_;

    // Speed conversion matrix (Eigen)
    Eigen::Matrix2d speed_conversion_;

    // Previous wheel positions
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;

    // Previous timestamp
    rclcpp::Time prev_time_;

    // Robot pose
    double x_;
    double y_;
    double theta_;

    // Odometry message
    nav_msgs::msg::Odometry odom_msgs_;

    // Quaternion for odometry orientation
    tf2::Quaternion q;

    // Transform broadcaster for TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif  // SIMPLE_CONTROLLER_HPP
