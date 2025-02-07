#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("circle_radius", 1.0)  # Radius of the circle Desired trajectory
        self.declare_parameter("angular_velocity", 0.01)  # Desired angular velocity


        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.circle_radius_ = self.get_parameter("circle_radius").get_parameter_value().double_value
        self.angular_velocity_ = self.get_parameter("angular_velocity").get_parameter_value().double_value

        self.kp_ = 0.05  # Proportional gain for position
        self.kd_ = 0.001  # Derivative gain for position
        self.kp_angle_ = 0.5  # Proportional gain for angle
        self.kd_angle_ = 0.001  # Derivative gain for angle


        self.get_logger().info("Using wheel radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = 0
        self.right_wheel_prev_pos_ = 0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0

        self.prev_error_ = 0.0
        self.prev_angle_error_ = 0.0


        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        self.get_logger().info("The joint_sub_ has been started" )

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.noisy_odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.velCallback, 10)
        # self.joint_sub2_ = self.create_subscription(JointState, "joint_states", self.velCallback, 10)

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.odom_msgs_ = Odometry()
        self.odom_msgs_.header.frame_id = "odom"
        self.odom_msgs_.child_frame_id = "base_footprint"
        self.odom_msgs_.pose.pose.orientation.x = 0.0
        self.odom_msgs_.pose.pose.orientation.y = 0.0
        self.odom_msgs_.pose.pose.orientation.z = 0.0
        self.odom_msgs_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_= TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)
    
    def update_position_error(self, t):
        # Compute the desired position on the circle at time t
        x_desired = self.circle_radius_ * math.cos(self.angular_velocity_ * t)
        y_desired = self.circle_radius_ * math.sin(self.angular_velocity_ * t)
        x_desired = self.circle_radius_ 
        y_desired = 0
        self.get_logger().info("the robot position at time %f is (%f,%f)" %(t,x_desired,y_desired))

        
        # Compute the position error (distance from the robot's current position to the desired position on the circle)
        position_error = (self.x_ - x_desired)*math.cos(self.theta_) + (self.y_ - y_desired)*math.sin(self.theta_)
        
        return position_error
    
    def update_angle_error(self, t):
        # Desired angle on the circle at time t
        theta_d = self.angular_velocity_ * t  # Desired orientation (angle) as a function of time
        theta_d = math.atan2(math.cos(self.angular_velocity_ * t),-math.sin(self.angular_velocity_ * t))
        theta_d = math.pi/2

        # Calculate the angle error (difference between desired and current robot orientation)
        angle_error = theta_d - self.theta_
        self.get_logger().info("the robot orientation at time %f is %f" %(t,theta_d))

        # Normalize the angle error to the range [-pi, pi] to avoid large angle jumps
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        return angle_error


    def velCallback(self, msg):

        t1 = self.get_clock().now().nanoseconds / 1e9
        t= t1-self.t0

        # Calculate the current position and angle errors
        position_error = self.update_position_error(t)
        angle_error = self.update_angle_error(t)

        # Proportional-Derivative control for linear velocity (v)
        v_control = self.kp_ * position_error + self.kd_ * (position_error - self.prev_error_)

        # Proportional-Derivative control for angular velocity (w)
        w_control = self.kp_angle_ * angle_error + self.kd_angle_ * (angle_error - self.prev_angle_error_)

                # Update previous errors for the next iteration
        self.prev_error_ = position_error
        self.prev_angle_error_ = angle_error


        # Implements the differential kinematic model
        # Convert control velocities to wheel speeds using inverse kinematics
        robot_speed = np.array([[v_control], [w_control]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        # Publish the wheel speed commands
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        dp_right= msg.position[0] - self.right_wheel_prev_pos_
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.right_wheel_prev_pos_ = msg.position[0]
        self.left_wheel_prev_pos_ = msg.position[1]
        self.prev_time_= Time.from_msg(msg.header.stamp)

        fi_left = dp_left/(dt.nanoseconds/S_TO_NS)
        fi_right = dp_right/(dt.nanoseconds/S_TO_NS)

        V = self.wheel_radius_/2*fi_right + self.wheel_radius_/2*fi_left
        w = self.wheel_radius_/self.wheel_separation_*fi_right - self.wheel_radius_/self.wheel_separation_*fi_left

        d_s = self.wheel_radius_/2*dp_right +self.wheel_radius_/2*dp_left
        d_theta = self.wheel_radius_/self.wheel_separation_*(dp_right-dp_left)
        self.theta_ +=d_theta
        self.x_+=d_s*math.cos(self.theta_)
        self.y_+=d_s*math.sin(self.theta_)


        q = quaternion_from_euler(0,0,self.theta_)
        self.odom_msgs_.pose.pose.orientation.x = q[0]
        self.odom_msgs_.pose.pose.orientation.y = q[1]
        self.odom_msgs_.pose.pose.orientation.z = q[2]
        self.odom_msgs_.pose.pose.orientation.w = q[3]
        self.odom_msgs_.pose.pose.position.x = self.x_
        self.odom_msgs_.pose.pose.position.y = self.y_
        self.odom_msgs_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msgs_.twist.twist.linear.x= V
        self.odom_msgs_.twist.twist.angular.z = w

        self.odom_pub_.publish(self.odom_msgs_)

        self.transform_stamped_.transform.translation.x  = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp =  self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)



        


        # self.get_logger().info("Linear Velociety measured by sensor: %f and angular velocity is %f " %(V,w))

        # self.get_logger().info(" x: %f , y: %f and theta %f " %(self.x_,self.y_,self.theta_))

def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()