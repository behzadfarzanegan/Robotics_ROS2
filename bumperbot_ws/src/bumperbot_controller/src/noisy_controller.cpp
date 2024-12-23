#include "bumperbot_controller/noisy_controller.hpp"
#include <random>
#include <memory>

NoisyController::NoisyController(const std::string &name) : Node(name)
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
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&NoisyController::jointCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

    left_wheel_prev_pos_ = 0;
    right_wheel_prev_pos_ = 0;
    prev_time_ = get_clock()->now();
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    q.setRPY(0, 0, theta_);

    odom_msgs_.header.frame_id = "odom";
    odom_msgs_.child_frame_id = "base_footprint_noisy";
    odom_msgs_.pose.pose.orientation.x = 0.0;
    odom_msgs_.pose.pose.orientation.y = 0.0;
    odom_msgs_.pose.pose.orientation.z = 0.0;
    odom_msgs_.pose.pose.orientation.w = 1.0;

    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";
}

void NoisyController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    std::default_random_engine noise_generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);
    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);

    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;
    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    right_wheel_prev_pos_ = wheel_encoder_right;
    left_wheel_prev_pos_ = wheel_encoder_left;
    prev_time_ = msg_time;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    double V = wheel_radius_ / 2 * (fi_right + fi_left);
    double w = wheel_radius_ / wheel_separation_ * (fi_right - fi_left);

    double d_s = wheel_radius_ / 2 * (dp_right + dp_left);
    double d_theta = wheel_radius_ / wheel_separation_ * (dp_right - dp_left);
    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    q.setRPY(0, 0, theta_);
    odom_msgs_.pose.pose.orientation.x = q.x();
    odom_msgs_.pose.pose.orientation.y = q.y();
    odom_msgs_.pose.pose.orientation.z = q.z();
    odom_msgs_.pose.pose.orientation.w = q.w();
    odom_msgs_.pose.pose.position.x = x_;
    odom_msgs_.pose.pose.position.y = y_;
    odom_msgs_.header.stamp = get_clock()->now();
    odom_msgs_.twist.twist.linear.x = V;
    odom_msgs_.twist.twist.angular.z = w;

    odom_pub_->publish(odom_msgs_);

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = get_clock()->now();
    br_->sendTransform(transform_stamped_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
