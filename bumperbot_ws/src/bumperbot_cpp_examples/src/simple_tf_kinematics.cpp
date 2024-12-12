#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name)
{
    // Initialize broadcasters
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Set up static transform
    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.30;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    // Broadcast static transform
    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(
        get_logger(), "Publishing static transform between "
                          << static_transform_stamped_.header.frame_id << " and "
                          << static_transform_stamped_.child_frame_id);

    // Set up timer for dynamic transforms
    timer_ = create_wall_timer(1s, std::bind(&SimpleTfKinematics::timerCallback, this));
}

void SimpleTfKinematics::timerCallback()
{
    // Set up dynamic transform
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
    dynamic_transform_stamped_.transform.translation.x = last_x_ + increment_x_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.30;
    dynamic_transform_stamped_.transform.rotation.x = 0.0;
    dynamic_transform_stamped_.transform.rotation.y = 0.0;
    dynamic_transform_stamped_.transform.rotation.z = 0.0;
    dynamic_transform_stamped_.transform.rotation.w = 1.0;

    // Broadcast dynamic transform
    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
    last_x_ = dynamic_transform_stamped_.transform.translation.x;

    RCLCPP_INFO(get_logger(), "Published dynamic transform. Current x: %.2f", last_x_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
