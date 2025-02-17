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

        self.k1 = 1.98  # Proportional gain for position
        self.k2 = 1.98  # Derivative gain for position
        self.k3 = 1.98  # Proportional gain for angle

        self.x_desired = 0.998
        self.y_desired = -1.47
        self.psi_desired = 0

        self.V_desired = 0.84
        self.V_desired_prev = 0.84
        self.dotpsi_desired = 0

        self.xe = 1.26
        self.ye = -1.47
        self.psie = 0.2


        self.get_logger().info("Using wheel radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = 0
        self.right_wheel_prev_pos_ = 0
        self.prev_time_ = self.get_clock().now()

        self.x_ = -0.128 # 0.0
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

        self.t_prev = 0
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
    
    def update_reference(self, t):
        # Compute the desired position on the circle at time t
        self.x_desired = 1.5 * math.sin(0.4* t + 0.2) +0.7
        self.y_desired = 1.5 * (1- math.cos(0.4* t+0.2))-1.5
        self.dotx_desired = 1.5 * 0.4*math.cos(0.4* t + 0.2)
        self.doty_desired = 1.5 * 0.4*math.sin(0.4* t + 0.2)
        self.dot2x_desired = -1.5 *0.4* 0.4*math.sin(0.4* t + 0.2)
        self.dot2y_desired = 1.5 * 0.4*0.4*math.cos(0.4* t + 0.2) 

        self.V_desired_prev = self.V_desired
        self.V_desired = math.sqrt(self.dotx_desired**2 + self.doty_desired**2)
        self.psi_desired = math.atan2(self.doty_desired, self.dotx_desired)
        self.dotpsi_desired = (self.dot2y_desired*self.dotx_desired - self.dot2x_desired*self.doty_desired) /(self.V_desired**2)
        self.tildey = self.ye +self.k2/self.V_desired*math.sin(self.psie)
    
    def update_angle_error(self, t):

        # Calculate the angle error (difference between desired and current robot orientation)
        self.psie = self.psi_desired - self.theta_
        self.xe = math.cos(self.theta_)*(self.x_desired - self.x_) + math.sin(self.theta_)*(self.y_desired - self.y_)
        self.ye = -math.sin(self.theta_)*(self.x_desired - self.x_) + math.cos(self.theta_)*(self.y_desired - self.y_)




    def velCallback(self, msg):
        t1 = self.get_clock().now().nanoseconds / 1e9
        t = t1 - self.t0
        delta_t = t - self.t_prev
        self.t_prev = t
        delta_t = max(delta_t, 1e-5)  # Prevent division by zero

        # Update reference trajectory and errors
        self.update_reference(t)
        self.update_angle_error(t)
        self.dotV_desired = (self.V_desired - self.V_desired_prev) / delta_t
        # self.dotV_desired =0


        # Constraint limits
        um1 = 10.5  # Max speed
        um2 = 3.5  # Max angular velocity

        # Ensure no division by zero
        eps = 1e-5
        self.tildey = self.ye + self.k2 / max(self.V_desired, eps) * math.sin(self.psie)

        # Control law (corrected)
        v_control = self.V_desired * math.cos(self.psie) + self.k1 * self.xe
        w_control = (self.k3 * self.tildey + 2 * self.V_desired * math.sin(self.psie) + 
                    self.k2 / max(self.V_desired, eps) * math.cos(self.psie) * self.dotpsi_desired - 
                    self.k2 / max(self.V_desired**2, eps) * math.sin(self.psie) * self.dotV_desired) / \
                    (self.xe + self.k2 / max(self.V_desired, eps) * math.cos(self.psie))

        # Apply constraints
        v_control = np.clip(v_control, -um1, um1)
        w_control = np.clip(w_control, -um2, um2)

        # Convert to wheel speeds
        robot_speed = np.array([[v_control], [w_control]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        # Publish wheel speed
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