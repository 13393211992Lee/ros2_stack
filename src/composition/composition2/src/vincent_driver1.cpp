
#include "rclcpp/rclcpp.hpp"

class VincentDriver : public rclcpp::Node
{
public:
    VincentDriver() : Node("vincent_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Default construction: hello");
    }

    VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver_node",options)
    {
        RCLCPP_INFO(this->get_logger(), "With args(options) construction:hello options");
    }

private:

};



int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<VincentDriver>(options));
    rclcpp::shutdown();
    return 0;
}