#ifndef COMPOSITION4__LISTENER_COMPONENT_HPP_
#define COMPOSITION4__LISTENER_COMPONENT_HPP_


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
namespace composition
{
class Listener_2 : public rclcpp::Node
{
public:
    Listener_2();
    ~Listener_2() override = default;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    void sub_callback(const std_msgs::msg::String::SharedPtr msg);

};
}  // namespace composition
#endif   //COMPOSITION4__LISTENER_COMPONENT_HPP_
