import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler,quaternion_multiply,quaternion_inverse



class SimpleTfKinematics(Node):

    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.x_increment_ = 0.01
        self.last_x_ = 0
        self.rotation_counter_ = 0
        self.last_orientation_ = quaternion_from_euler(0,0,0)
        self.orientation_increment_ = quaternion_from_euler(0,0,0.01)


        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        self.static_transform_Stamped_ = TransformStamped()
        self.dynamic_transform_Stamped_ = TransformStamped()

        self.static_transform_Stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_Stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_Stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_Stamped_.transform.translation.x = 0.0
        self.static_transform_Stamped_.transform.translation.y = 0.0
        self.static_transform_Stamped_.transform.translation.z = 0.3
        self.static_transform_Stamped_.transform.rotation.x = 0.0
        self.static_transform_Stamped_.transform.rotation.y = 0.0
        self.static_transform_Stamped_.transform.rotation.z = 0.0
        self.static_transform_Stamped_.transform.rotation.w = 1.0


        self.static_tf_broadcaster_.sendTransform(self.static_transform_Stamped_)

        self.get_logger().info("Publishing static transform between %s and %s" 
                               %(self.static_transform_Stamped_.header.frame_id , self.static_transform_Stamped_.child_frame_id))
        self.timer_ = self.create_timer(0.1, self.timerCallback)
        
        self.get_transform_srv_ = self.create_service(GetTransform,"get_transform", self.getTransformCallback)


    def timerCallback(self):
        self.dynamic_transform_Stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_Stamped_.header.frame_id = "odom"
        self.dynamic_transform_Stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_Stamped_.transform.translation.x = self.x_increment_ + self.last_x_
        self.dynamic_transform_Stamped_.transform.translation.y =0.0
        self.dynamic_transform_Stamped_.transform.translation.z =0.0
        q = quaternion_multiply(self.last_orientation_,self.orientation_increment_)
        self.dynamic_transform_Stamped_.transform.rotation.x =q[0]
        self.dynamic_transform_Stamped_.transform.rotation.y =q[1]
        self.dynamic_transform_Stamped_.transform.rotation.z =q[2]
        self.dynamic_transform_Stamped_.transform.rotation.w =q[3]

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_Stamped_)
        self.last_x_ = self.dynamic_transform_Stamped_.transform.translation.x
        self.last_orientation_ = q
        self.rotation_counter_+=1
        self.get_logger().info("rotation counter: %d " %self.rotation_counter_ )
        if self.rotation_counter_ >=100:
            self.rotation_counter_ = 0
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)


    def getTransformCallback(self,req,res):
        self.get_logger().info("Requested Transform between %s and %s" %(req.frame_id, req.child_frame_id))
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An error occured while transforming between %s and %s" %(req.frame_id, req.child_frame_id) )
            res.success = False
            return res
        
        res.transform = requested_transform
        res.success = True

        return res
        


def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

