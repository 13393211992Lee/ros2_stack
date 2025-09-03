
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node{
public:
  PublisherNode(): Node("publisher_node"){
    publisher_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);
    timer_ = create_wall_timer(
      500ms, std::bind(&PublisherNode::timer_callback, this));
  }

  void timer_callback(){
    message_.data += 1;
    publisher_->publish(message_);
    // 打印当前线程 ID 和时间戳
    RCLCPP_INFO(get_logger(), "Publisher: data=%d, thread_id=%ld, time=%ld",
      message_.data,
      std::hash<std::thread::id>{}(std::this_thread::get_id()),  // 线程ID哈希值
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())  // 时间戳
    );
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

class SubscriberNode : public rclcpp::Node{
public:
  SubscriberNode(): Node("subscriber_node"){
    subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  void callback(const std_msgs::msg::Int32::SharedPtr msg){
    // RCLCPP_INFO(get_logger(), "Hello %d", msg->data);
    // 打印当前线程 ID、时间戳和接收的数据
    RCLCPP_INFO(get_logger(), "Subscriber: Hello  %d, thread_id=%ld, time=%ld",
      msg->data,
      std::hash<std::thread::id>{}(std::this_thread::get_id()),
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_pub = std::make_shared<PublisherNode>();
  auto node_sub = std::make_shared<SubscriberNode>();

  // rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor;

  // 向执行器添加节点
  executor.add_node(node_pub);
  executor.add_node(node_sub);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}