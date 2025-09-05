#include "composition3/server_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace composition
{

Server::Server(const rclcpp::NodeOptions & options)
: Node("Server", options)
{
  auto handle_add_two_ints =
    [this](
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    ) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "Incoming request: [a: %" PRId64 ", b: %" PRId64 "]",
        request->a, request->b);
      std::flush(std::cout);
      response->sum = request->a + request->b;
    };

  srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Server)
