#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;


/*
subscriber 标准框架
*/


namespace ns
{
class Node4Sub : public rclcpp::Node
{
public:
    Node4Sub() : Node("node4_sub_node")
    {
    node4_sub_ = this->create_subscription<std_msgs::msg::String>(
        "string_topic", 10, std::bind(&Node4Sub::node4_sub_cb, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node4_sub_;
    void node4_sub_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message");
        RCLCPP_INFO(this->get_logger(), "data: %s",msg->data.c_str());
    }
};
}  // end namespace :ns

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ns::Node4Sub>());
    rclcpp::shutdown();
    return 0;
}
