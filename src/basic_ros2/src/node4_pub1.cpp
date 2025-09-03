#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/*
官方提供：https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
publiser
回调函数直接用 lambda 表达式实现
*/

class MinimalPublisher : public rclcpp::Node    //通过继承rclcpp::Node 来创建节点类, this 都指的是该节点。
{
public:
  MinimalPublisher()
  : Node("publisher_node"), count_(0) //node_name：publisher_node & count_ 初始化为 0
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);

    // 创建一个定时器
    //timer_callback 的 lambda 函数
    auto timer_callback =[this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}