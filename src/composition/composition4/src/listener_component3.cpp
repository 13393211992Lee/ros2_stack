
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
namespace composition
{
class Listener3 : public rclcpp::Node
{
public:
  Listener3(const rclcpp::NodeOptions & options) : Node("listener3", options)
  {
    using std::placeholders::_1;
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&Listener3::sub_cb, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  void sub_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message %s",msg->data.c_str());
  }
};
}  // namespace composition



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener3)
