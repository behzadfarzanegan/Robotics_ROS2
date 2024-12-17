#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        // Create the service and bind the callback function
        service_ = create_service<bumperbot_msgs::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&SimpleServiceServer::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO_STREAM(get_logger(), "Service add_two_ints is ready \n");
    }

private:
    // Service definition
    rclcpp::Service<bumperbot_msgs::srv::AddTwoInts>::SharedPtr service_;

    // Service callback function
    void serviceCallback(
        const std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Request> req,
        std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO_STREAM(get_logger(), "New request received: a = " << req->a << " and b = " << req->b);

        // Compute the sum
        response->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(get_logger(), "Returning sum: " << response->sum);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
