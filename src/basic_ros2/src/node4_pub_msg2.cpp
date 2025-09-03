/*
自定义消息：basic_interfaces::msg::Num
*/
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "basic_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

class PublisherNum : public rclcpp::Node
{
public:
  PublisherNum() : Node("publisher_num_node"),count_(10)
  {
    //ros2 publisher Init 
    publisher_ = this->create_publisher<basic_interfaces::msg::Num>("topic", 10);
    
    // ros2 timer init
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PublisherNum::timer_cb, this));
  }

private:
  rclcpp::Publisher<basic_interfaces::msg::Num>::SharedPtr publisher_;  //ros2 publisher
  rclcpp::TimerBase::SharedPtr timer_;  //ros2 timer
  size_t count_;

  void timer_cb()
  {
    auto msg = basic_interfaces::msg::Num();
    msg.num = ++count_;
    RCLCPP_INFO(this->get_logger(), "Publishing message %.ld",msg.num);
    publisher_->publish(msg);
  }

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNum>());
  rclcpp::shutdown();
  return 0;
}
