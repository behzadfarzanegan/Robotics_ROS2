#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name)
{
    // Initialize broadcasters
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize tf2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_); // Pass by reference

    // Set up static transform
    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.30;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(),
                       "Publishing static transform between "
                           << static_transform_stamped_.header.frame_id << " and "
                           << static_transform_stamped_.child_frame_id);

    // Timer for dynamic transform
    timer_ = create_wall_timer(1s, std::bind(&SimpleTfKinematics::timerCallback, this));

    // Service for getting transforms
    get_transform_srv_ = create_service<bumperbot_msgs::srv::GetTransform>(
        "get_transform",
        std::bind(&SimpleTfKinematics::getTransformCallback, this, std::placeholders::_1, std::placeholders::_2));

        last_orientation_.setRPY(0,0,0);
        orientation_increment_.setRPY(0,0,0.05);
}

void SimpleTfKinematics::timerCallback()
{
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped_.transform.translation.x = last_x_ + increment_x_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q = last_orientation_ * orientation_increment_;
    q.normalize();
    dynamic_transform_stamped_.transform.rotation.x = q.x();
    dynamic_transform_stamped_.transform.rotation.y = q.y();
    dynamic_transform_stamped_.transform.rotation.z = q.z();
    dynamic_transform_stamped_.transform.rotation.w = q.w();

    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
    last_x_ = dynamic_transform_stamped_.transform.translation.x;
    rotation_counter_ ++;
    last_orientation_ = q;
    if(rotation_counter_>=100){
        rotation_counter_ =0;
        orientation_increment_ = orientation_increment_.inverse();
    }

    RCLCPP_INFO(get_logger(), "Published dynamic transform. Current x: %.2f", last_x_);
}

void SimpleTfKinematics::getTransformCallback(
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res)
{
    RCLCPP_INFO_STREAM(get_logger(),
                       "Request Transform between " << req->frame_id << " and " << req->child_frame_id);

    geometry_msgs::msg::TransformStamped requested_transform;
    try
    {
        requested_transform = tf_buffer_->lookupTransform(req->frame_id, req->child_frame_id, rclcpp::Time(0));
        res->transform = requested_transform;
        res->success = true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Transform error: " << ex.what());
        res->success = false;
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
