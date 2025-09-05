#include "composition4/listener_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace composition
{
Listener::Listener(const rclcpp::NodeOptions & options):Node("listener", options)
{
  using std::placeholders::_1;
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "chatter", 10, std::bind(&Listener::sub_cb, this, _1));
}
void Listener::sub_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(Listener::get_logger(), "Received message %s", msg->data.c_str());
   
  }

}  // namespace /* namespace_name */


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)