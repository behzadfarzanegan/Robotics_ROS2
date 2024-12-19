#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>

SimpleController::SimpleController(const std::string &name) : Node(name)
{
    // Declare parameters
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    // Get parameters
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);

    // Initialize publisher and subscriber
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/bumperbot_controller/cmd_vel", 10,
        std::bind(&SimpleController::velCallback, this, std::placeholders::_1));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimpleController::jointCallback, this, std::placeholders::_1)
    );

    // Initialize speed conversion matrix
    speed_conversion_ << wheel_radius_ / 2, wheel_radius_ / 2,
        -wheel_radius_ / wheel_separation_, wheel_radius_ / wheel_separation_;

    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);

    left_wheel_prev_pos_ =0;
    right_wheel_prev_pos_ =0;
    prev_time_ = get_clock()->now();

}

void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Calculate wheel speeds
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    // Publish wheel speeds
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));  // Left wheel speed
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));  // Right wheel speed

    wheel_cmd_pub_->publish(wheel_speed_msg);
}
void SimpleController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    double dp_right;
    double dp_left;
    dp_right = msg.position.at(0) - right_wheel_prev_pos_;
    dp_left = msg.position.at(1) - left_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    right_wheel_prev_pos_ = msg.position.at(0);
    left_wheel_prev_pos_ =  msg.position.at(1);
    prev_time_ = msg_time;

    double fi_left = dp_left/dt.seconds();
    double fi_right = dp_right/dt.seconds();

    double V = wheel_radius_/2*fi_right + wheel_radius_/2*fi_left;
    double w = wheel_radius_/wheel_separation_*fi_right - wheel_radius_/wheel_separation_*fi_left;

    RCLCPP_INFO_STREAM(get_logger(), "Linear Velocity is measured as: " << V<<" and Angular Velocity is : "<< w<<"\n");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
