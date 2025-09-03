
/*
创建node 方式（不推荐）
*/
#include  "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化
  rclcpp::init(argc, argv);
  // 创建节点
  auto node = rclcpp::Node::make_shared("demo01");
  RCLCPP_INFO(node->get_logger(),"一个线程一个节点的方式创建node");
  // 释放资源
  rclcpp::shutdown();
}
