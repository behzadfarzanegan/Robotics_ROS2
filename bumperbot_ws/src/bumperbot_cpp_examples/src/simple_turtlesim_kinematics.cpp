#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"
// #include <functional> // Required for std::placeholders

// Constructor: Initializes the node and subscribes to the turtle pose topics
SimpleTurtleKinematics::SimpleTurtleKinematics(const std::string &name) : Node(name)
{
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&SimpleTurtleKinematics::turtle1PoseCallback, this, std::placeholders::_1));

    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10,
        std::bind(&SimpleTurtleKinematics::turtle2PoseCallback, this, std::placeholders::_1));
}

// Callback: Updates the last known pose of turtle1
void SimpleTurtleKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle1_pose_ = pose;
}

// Callback: Updates the last known pose of turtle2 and logs the translation vector
void SimpleTurtleKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle2_pose_ = pose;

    // Ensure valid poses are set before computing the translation vector
    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

    // Log the translation vector
    RCLCPP_INFO(get_logger(), "Translation Vector turtle1 -> turtle2: Tx: %.2f, Ty: %.2f", Tx, Ty);
}

// Main function: Initializes ROS, spins the node, and shuts down ROS
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtleKinematics>("simple_turtle_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
