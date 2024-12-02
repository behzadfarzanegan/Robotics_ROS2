#
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtlesim1_pos_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.turtlePoseCallback1,10)
        self.turtlesim2_pos_sub_ = self.create_subscription(Pose, "/turtle2/pose", self.turtlePoseCallback2,10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtlePoseCallback1(self,msg):
        self.last_turtle1_pose_ = msg


    def turtlePoseCallback2(self,msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle1_pose_.x - self.last_turtle2_pose_.x
        Ty = self.last_turtle1_pose_.y - self.last_turtle2_pose_.y


        self.get_logger().info("""\n
                               Translation vector turtle1 -> turtle2\n
                               Tx: %f \n
                               Ty: %f \n"""% (Tx, Ty))

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
