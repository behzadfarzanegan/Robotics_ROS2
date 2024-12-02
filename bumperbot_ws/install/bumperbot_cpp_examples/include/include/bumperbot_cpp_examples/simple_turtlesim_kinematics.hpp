#ifndef SIMPLE_TURTLE_KINEMATICS_HPP
#define SIMPLE_TURTLE_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

/**
 * @class SimpleTurtleKinematics
 * @brief A ROS 2 node to calculate and log the relative kinematics of two turtles in turtlesim.
 */
class SimpleTurtleKinematics : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for SimpleTurtleKinematics.
     * @param name The name of the ROS 2 node.
     */
    explicit SimpleTurtleKinematics(const std::string &name);

private:
    /**
     * @brief Callback function for receiving turtle1's pose.
     * @param pose Shared pointer to the Pose message.
     */
    void turtle1PoseCallback(const turtlesim::msg::Pose &pose);

    /**
     * @brief Callback function for receiving turtle2's pose.
     * @param pose Shared pointer to the Pose message.
     */
    void turtle2PoseCallback(const turtlesim::msg::Pose &pose);

    // Subscriptions to the pose topics of turtle1 and turtle2
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

    // Last known pose of turtle1
    turtlesim::msg::Pose last_turtle1_pose_;

    // Last known pose of turtle2
    turtlesim::msg::Pose last_turtle2_pose_;
};

#endif // SIMPLE_TURTLE_KINEMATICS_HPP
