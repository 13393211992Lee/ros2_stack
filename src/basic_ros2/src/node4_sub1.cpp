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
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("string_topic", 10, topic_callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}  // end namespace :ns

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ns::Node4Sub>());
    rclcpp::shutdown();
    return 0;
}
