#include "rclcpp/rclcpp.hpp"
#include "basic_interfaces/srv/add_three_ints.hpp"                                        // CHANGE


using std::placeholders::_1;
using std::placeholders::_2;

class ServerInterface : public rclcpp::Node
{
public:
  ServerInterface() : Node("server")
  {

    server_ = this->create_service<basic_interfaces::srv::AddThreeInts>(
      "add_three_ints", std::bind(&ServerInterface::server_cb, this, _1, _2));
  }

private:
  rclcpp::Service<basic_interfaces::srv::AddThreeInts>::SharedPtr server_;
  void server_cb(
    const std::shared_ptr<basic_interfaces::srv::AddThreeInts::Request> request,
    std::shared_ptr<basic_interfaces::srv::AddThreeInts::Response> response){
      RCLCPP_INFO(this->get_logger(), "请求的参数为：a=%ld  b=%ld  c=%ld  ",request->a,request->b,request->c);
      response->sum = request->a+request->b+request->c;
      RCLCPP_INFO(this->get_logger(), "sum=%ld",response->sum);
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerInterface>());
  rclcpp::shutdown();
  return 0;
}
