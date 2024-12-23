
#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>



class NoisyController : public rclcpp::Node
{
public:
    explicit NoisyController(const std::string &node_name);

private:
    // Callback function for velocity messages
    void jointCallback(const sensor_msgs::msg::JointState &msg);

    // ROS 2 Subscriber and Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;


    // Parameters
    double wheel_radius_;
    double wheel_separation_;


    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;

    rclcpp::Time prev_time_;
    double x_;
    double y_;
    double theta_;
    nav_msgs::msg::Odometry odom_msgs_;

    tf2::Quaternion q;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    

};

#endif  // NOISY_CONTROLLER_HPP