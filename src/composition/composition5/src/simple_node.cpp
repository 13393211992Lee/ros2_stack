#include <memory>
#include "rclcpp/rclcpp.hpp"



/**
 * @brief 一个标准的ros2 类结构
 */

namespace ns
{
class SimpleNode : public rclcpp::Node
{
public:
  SimpleNode(): Node("Get_SimpleNode_Info")
  {
    RCLCPP_INFO(this->get_logger(), "一个标准 ros2 class 结构");
  }

private:

};
}  // end ns

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ns::SimpleNode>());
  rclcpp::shutdown();
  return 0;
}
