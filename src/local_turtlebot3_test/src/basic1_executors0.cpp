#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;
class basic1_executors0: public rclcpp::Node{
private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
  void timer_callback(){
    message_.data+=1;
    publisher_->publish(message_);
  }
public:
  basic1_executors0(): Node("publisher_node"){
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_na",10);
    timer_ = this->create_wall_timer(500ms,std::bind(&basic1_executors0::timer_callback, this));
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<basic1_executors0>());
  rclcpp::shutdown();
  return 0;
}

/*
节点发布信息

$ ros2 run local_turtlebot3_test basic1_executors0 
$ ros2 topic echo /topic_na
 */