#ifndef COMPOSITION4__TALKER_COMPONENT_HPP_
#define COMPOSITION4__TALKER_COMPONENT_HPP_


#include "composition4/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
namespace composition
{
class Talker : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC
    explicit Talker(const rclcpp::NodeOptions & options );

protected:
    void on_timer();

private:
    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace /* namespace_name */
#endif  // COMPOSITION4__TALKER_COMPONENT_HPP_

