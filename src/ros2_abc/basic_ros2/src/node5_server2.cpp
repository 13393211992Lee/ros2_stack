
/*
创建node 方式（推荐）
*/
#include  "rclcpp/rclcpp.hpp"
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
// 常用的ros template
using std::placeholders::_1;
using std::placeholders::_2;

class LocalNode : public rclcpp::Node
{
public:
  LocalNode() : Node("local_node1")
  {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(&LocalNode::server_cb, this, _1, _2));
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
  void server_cb(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a+ request->b;
    RCLCPP_INFO(this->get_logger(), "request param: a=%.ld , b=%.ld",request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "response result: total=%.ld",response->sum);
  }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalNode>());
  rclcpp::shutdown();
  return 0;
}
