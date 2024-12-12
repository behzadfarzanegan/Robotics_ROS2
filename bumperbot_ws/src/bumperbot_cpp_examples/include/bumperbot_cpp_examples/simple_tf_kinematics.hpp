#ifndef SIMPLE_KINEMATICS_HPP
#define SIMPLE_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

/**
 * @brief SimpleTfKinematics is a ROS 2 node that broadcasts static and dynamic transforms.
 */
class SimpleTfKinematics : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new SimpleTfKinematics object with a custom name.
     * 
     * @param name The name of the node.
     */
    explicit SimpleTfKinematics(const std::string &name);

    /**
     * @brief Construct a new SimpleTfKinematics object with the default name.
     */
    SimpleTfKinematics();

private:
    /**
     * @brief Timer callback to broadcast dynamic transforms periodically.
     */
    void timerCallback();

    /// Static transform broadcaster to publish static transforms.
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    /// Dynamic transform broadcaster to publish dynamic transforms.
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

    /// TransformStamped message for static transform.
    geometry_msgs::msg::TransformStamped static_transform_stamped_;

    /// TransformStamped message for dynamic transform.
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    /// Last x position for dynamic transform.
    double last_x_ = 0;

    /// Increment value for dynamic transform's x position.
    double increment_x_ = 0.01;

    rclcpp::TimerBase::SharedPtr timer_;

};

#endif  // SIMPLE_KINEMATICS_HPP
