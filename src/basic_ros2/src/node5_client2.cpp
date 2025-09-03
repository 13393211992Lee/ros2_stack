

#include  "rclcpp/rclcpp.hpp"
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
using std::placeholders::_1;

// 常用的ros template


class LocalNode : public rclcpp::Node
{
public:
  LocalNode(int argc, char * argv[]) : Node("local_node1"){

    // 1. 解析命令行参数
    parse_arguments(argc, argv);

    //2 .初始化客户端
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    // 使用从命令行获取的参数
    request->a = a_;
    request->b = b_;

    //3. 等待服务启动
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "被中断，等待服务器时退出.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "服务器不可用，再次等待...");
    }
    //4. 发送请求
    RCLCPP_INFO(this->get_logger(), "发送请求: a:%ld, b:%ld",request->a ,request->b);
    client_->async_send_request(
      request, std::bind(&LocalNode::client_cb, this, _1));
  }

private:
  long int a_ = 0;  // 存储第一个加数
  long int b_ = 0;  // 存储第二个加数
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;

  void parse_arguments(int argc, char * argv[]){
      if (argc == 3) {
        a_ = std::stol(argv[1]);  // 从第一个参数获取a的值
        b_ = std::stol(argv[2]);  // 从第二个参数获取b的值
      }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return;
      }
  }
  void client_cb(ServiceResponseFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "收到响应: %ld + %ld = %ld", a_, b_, response->sum);
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalNode>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
