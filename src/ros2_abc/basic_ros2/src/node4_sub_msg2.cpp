#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "basic_interfaces/msg/num.hpp"                                       

using std::placeholders::_1;
class SubcriberNode : public rclcpp::Node{
public:
  SubcriberNode() : Node("publisher_node")
  {
    //ros2 sub init
    sub_ = this->create_subscription<basic_interfaces::msg::Num>(
      "topic", 10, std::bind(&SubcriberNode::sub_callback, this, _1));
  }

private:
  rclcpp::Subscription<basic_interfaces::msg::Num>::SharedPtr sub_;   //ros2 subscriber
  // ros2 sub callback
  void sub_callback(const basic_interfaces::msg::Num::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message %ld",msg->num);
  }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubcriberNode>());
  rclcpp::shutdown();
  return 0;
}
