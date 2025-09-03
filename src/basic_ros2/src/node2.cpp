
/*
创建node 方式（推荐）
*/
#include  "rclcpp/rclcpp.hpp"
#include <memory>

// 常用的ros template

namespace nmsp
{
class LocalNode : public rclcpp::Node
{
public:
  LocalNode() : Node("local_node1")
  {
    RCLCPP_INFO(this->get_logger(), "继承的方式实现node创建");
  }

private:

};
} 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nmsp::LocalNode>());
  rclcpp::shutdown();
  return 0;
}
