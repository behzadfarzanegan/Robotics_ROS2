#include<rclcpp/rclcpp.hpp>
#include<bumperbot_msgs/srv/add_two_ints.hpp>
using namespace std::chrono_literals;

class SimpleServiceClient : public rclcpp::Node
{
public:
    SimpleServiceClient(int a ,int b) : Node("simple_sevice_client")
    {
        client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(1s)){
            RCLCPP_INFO_STREAM(get_logger(), "Service not available, waiting again ...");
        }
        req_ = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
        req_->a = a;
        req_->b = b;
        auto result = client_->async_send_request(req_,std::bind(&SimpleServiceClient::responseCallback,this, std::placeholders::_1));
    }

private:
    rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;
    std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Request> req_;

    void responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future){

        if(future.valid()){
            RCLCPP_INFO_STREAM(get_logger(), "Service respose: "<< future.get()->sum);
        }
        else{
            RCLCPP_ERROR(get_logger(),"Service Failure");
        }
        
    }
};


int main(int argc, char* argv[])
{   
    if (argc !=3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"wrong number of argument");
        return 1;
    }
    rclcpp::init(argc,argv);
   // Parse arguments
    int a = std::atoi(argv[1]);
    int b = std::atoi(argv[2]);

    // Create the client node
    auto node = std::make_shared<SimpleServiceClient>(a, b);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
