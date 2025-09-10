#include <memory>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 使用节点接口模板类（C++）
 * 使用 SharedPtr 访问节点信息
 */
namespace ns
{
void node_info(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "Node name: %s", node->get_name());
}

class SimpleNode2 : public rclcpp::Node
{
public:
    SimpleNode2(const std::string & node_name) 
    : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "构造函数...");
    }
};
}  // end ns


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<ns::SimpleNode2>("Simple_Node2");
    ns::node_info(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
