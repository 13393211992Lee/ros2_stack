#ifndef COMPOSITION__CLIENT_COMPONENT_HPP_
#define COMPOSITION__CLIENT_COMPONENT_HPP_

#include "composition3/visibility_control.h"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace composition
{

class Client : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Client(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

#endif  // COMPOSITION__CLIENT_COMPONENT_HPP_
