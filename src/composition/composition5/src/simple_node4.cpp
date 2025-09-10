#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
/**
 * @brief 使用节点接口模板类（C++）
 * Explicitly pass rclcpp::node_interfaces
 * 显式传递 rclcpp::node_interfaces
 * 一种更健壮的方法，适用于所有节点类型，是显式地将 rclcpp::node_interfaces 作为函数参数传递，
 */


namespace ns
{
void nodeinfo(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_log_interface){
    RCLCPP_INFO(node_log_interface->get_logger(), "Node name: %s", node_base_interface->get_name());
}
class SimpleNode : public rclcpp::Node
{
public:
    SimpleNode(const std::string & node_name) : Node(node_name){}
private:

};

// explicit:显式 防止编译器进行隐式类型转换, 用于修饰单参数构造函数
// LifecycleTalker node = "my_node";  // 不推荐的隐式转换
// LifecycleTalker node("my_node");     //正确方式
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}
};
}  // namespace ns



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_simple = std::make_shared<ns::SimpleNode>("Simple_Node");
    auto node_life = std::make_shared<ns::LifecycleTalker>("Lifecycle_Node");
    ns::nodeinfo(node_simple->get_node_base_interface(),node_simple->get_node_logging_interface());
    ns::nodeinfo(node_life->get_node_base_interface(),node_life->get_node_logging_interface());
}
