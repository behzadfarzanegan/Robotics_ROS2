#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import time
imu_pub_ = None

def imuCallback(imu):
    global imu_pub_
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub_.publish(imu)

def main():
    global imu_pub_
    rclpy.init()
    node = Node("imu_republisher_node")  
    time.sleep(1) 

    imu_pub_ = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub_ = node.create_subscription(Imu, "imu/out", imuCallback,10)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main':
    main()
