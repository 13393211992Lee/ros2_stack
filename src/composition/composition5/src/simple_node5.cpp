/**
 * @brief  访问 Node 类型信息推荐的方式是通过 Node Interfaces
 * 
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
using MyNodeInterfaces =
    rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeLoggingInterface
    >;

void node_info(MyNodeInterfaces interfaces){
    auto base_interface = interfaces.get_node_base_interface();
    auto log_interface = interfaces.get_node_logging_interface();
    RCLCPP_INFO(log_interface->get_logger(),"Node name: %s", base_interface->get_name() );
}

class SimpleNode : public rclcpp::Node
{
public:
    SimpleNode(std::string node_name) : Node(node_name)
    {}

private:

};

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false) 
    : rclcpp_lifecycle::LifecycleNode(node_name, 
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<SimpleNode>("Simple_Node");
    auto lc_node = std::make_shared<LifecycleTalker>("Simple_LifeCycle_Node");
    node_info(*node);
    node_info(*lc_node);
}
