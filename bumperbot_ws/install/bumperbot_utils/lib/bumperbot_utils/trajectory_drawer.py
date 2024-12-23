#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped



class TrajectoryDrawer(Node):

    def __init__(self):
        super().__init__("trajectory_drawer")
        self.declare_parameter("odom_topic", "bumperbot_controller/odom")
        odom_topic_ = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.sub_ = self.create_subscription(Odometry, odom_topic_, self.msgCallback, 10)
        self.pub_ = self.create_publisher(Path, "bumperbot_controller/trajectory", 10)
        self.path_ = Path()
        
        self.get_logger().info("Trajectory drawer node is started.")

    def msgCallback(self, msg):
        pose_stamped_ = PoseStamped()
        pose_stamped_.header= msg.header
        pose_stamped_.pose = msg.pose.pose

        self.path_.poses.append(pose_stamped_)
        self.path_.header.stamp = msg.header.stamp
        self.path_.header.frame_id = msg.header.frame_id

        self.get_logger().info(f"Recieved pose: {pose_stamped_.pose.position}")
        self.pub_.publish(self.path_)


def main():
    rclpy.init()

    trajectory_drawer = TrajectoryDrawer()
    rclpy.spin(trajectory_drawer)
    
    trajectory_drawer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()