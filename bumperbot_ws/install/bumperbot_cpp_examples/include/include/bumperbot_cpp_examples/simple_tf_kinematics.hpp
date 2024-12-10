#ifndef SIMPLE_KINEMATICS_HPP
#define SIMPLE_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

/**
 * @brief SimpleTfKinematics is a ROS 2 node that broadcasts static transforms.
 */
class SimpleTfKinematics : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new SimpleTfKinematics object.
     * 
     * @param name The name of the node.
     */
    SimpleTfKinematics(const std::string &name);

private:
    /// Static transform broadcaster to publish transforms.
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    /// TransformStamped message for defining static transforms.
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
};

#endif  // SIMPLE_KINEMATICS_HPP
