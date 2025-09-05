#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_

#include "composition3/visibility_control.h"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace composition
{

class Server : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Server(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

}  // namespace composition

#endif  // COMPOSITION__SERVER_COMPONENT_HPP_
