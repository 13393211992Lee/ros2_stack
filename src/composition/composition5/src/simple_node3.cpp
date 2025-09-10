#include <memory>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 使用节点接口模板类（C++）
 * 使用 rclcpp::executors::SingleThreadedExecutor 运行多个节点
 */

namespace ns
{
    void getNodeInfo(rclcpp::Node::SharedPtr node){
        RCLCPP_INFO(node->get_logger(), "节点name %s",node->get_name() );
    }

    // SimpleNode3
    class SimpleNode3 : public rclcpp::Node{
    public:
        SimpleNode3(const std::string & node_name ) : Node(node_name){}
    };

    // LifecycleTalker
    class LifecycleTalker : public rclcpp::Node{
    public:
        LifecycleTalker(const std::string & node_name) : Node(node_name){}
    };



}  // namespace ns



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<ns::SimpleNode3>("Simple_Node3");
    auto lc_node = std::make_shared<ns::LifecycleTalker>("Simple_LifeCycle_Node");
    exe.add_node(node);
    exe.add_node(lc_node);
    ns::getNodeInfo(node);
    ns::getNodeInfo(lc_node);

    exe.spin();
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS 2 已关闭");
    return 0;
}
