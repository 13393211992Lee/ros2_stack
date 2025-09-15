#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/*
  publiser 的标准构造
  节点里的参数
*/


#include "rclcpp/rclcpp.hpp"

namespace ns
{
class StringPublisher : public rclcpp::Node
{
public:
  StringPublisher() : Node("string_pub_node"),count_(10000){
  this->declare_parameter("my_parameter","cici");
  string_pub_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&StringPublisher::timer_cb, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  void timer_cb(){
    std::string my_param = this->get_parameter("my_parameter").as_string();
    auto message = std_msgs::msg::String();
    message.data = "Hello, "+my_param+ std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    string_pub_->publish(message);
  }

};
}  // end ns



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ns::StringPublisher>());
  rclcpp::shutdown();
  return 0;
}
