import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")

        self.service_ = self.create_service(AddTwoInts,"add_two_ints",self.serviceCallback)

        self.get_logger().info("Service add_two_ints is ready!")

    def serviceCallback(self, req, res):

        self.get_logger().info("New request Received a: %d and b: %d" %(req.a,req.b))

        res.sum = req.a + req.b

        self.get_logger().info("Returning sum: %d" %res.sum)

        return res
    
def main():
    rclpy.init()
    simple_service_server_ = SimpleServiceServer()
    rclpy.spin(simple_service_server_)
    simple_service_server_.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()




