#include "rclcpp/rclcpp.hpp"

/**
 * @file vincent_driver.cpp
 * @brief 文森特驱动节点的实现文件
 * 
 * 该文件包含了VincentDriver类的定义与实现，
 * 这是一个基于ROS 2的节点组件，演示了不同构造函数的使用方法。
 * 
 * @author jj
 * @version 1.0
 * @date 2023-10-01
 */

/**
 * @class VincentDriver
 * @brief 驱动节点类
 * 
 * 继承自rclcpp::Node，实现了一个简单的ROS 2节点组件，
 * 提供了默认构造函数和带参数的构造函数，用于演示节点初始化方式。
 */
class VincentDriver : public rclcpp::Node
{
public:
    /**
     * @brief 默认构造函数
     * 
     * 初始化节点，节点名称为"vincent_driver_node"，
     * 并输出初始化日志信息。
     */
    VincentDriver() : Node("vincent_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Default construction: hello");
    }

    /**
     * @brief 带参数的构造函数
     * 
     * 接受NodeOptions参数初始化节点，节点名称为"vincent_driver_node"，
     * 并输出带参数初始化的日志信息。
     * 
     * @param options 节点初始化选项
     */
    VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "With args(options) construction: hello options");
    }

private:
    // 私有成员变量和函数可以在这里添加注释
};

/**
 * @brief 主函数
 * 
 * 初始化ROS 2，创建VincentDriver节点实例并运行，
 * 直到收到退出信号。
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态码，0表示正常退出
 */
int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<VincentDriver>(options));
    rclcpp::shutdown();
    return 0;
}
    