#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class NoisyController(Node):
    def __init__(self):
        # Ensure the node is initialized with the correct name
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = 0
        self.right_wheel_prev_pos_ = 0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0


        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)

        self.odom_msgs_ = Odometry()
        self.odom_msgs_.header.frame_id = "odom"
        self.odom_msgs_.child_frame_id = "base_footprint_ekf"
        self.odom_msgs_.pose.pose.orientation.x = 0.0
        self.odom_msgs_.pose.pose.orientation.y = 0.0
        self.odom_msgs_.pose.pose.orientation.z = 0.0
        self.odom_msgs_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_= TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"



    def jointCallback(self, msg):
        wheel_encoder_right = msg.position[0]+np.random.normal(0,0.005)
        wheel_encoder_left = msg.position[1]+np.random.normal(0,0.005)

        dp_right= wheel_encoder_right - self.right_wheel_prev_pos_
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # self.right_wheel_prev_pos_ = msg.position[0]
        # self.left_wheel_prev_pos_ = msg.position[1]

        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.left_wheel_prev_pos_ = wheel_encoder_left
        
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

    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()