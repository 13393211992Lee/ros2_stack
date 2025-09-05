
/*
组件优势：
    开销少：多个组件可以共享同一个进程，减少资源占用，适合复杂系统
    灵活性：可以在运行时通过ros2 component unload卸载组件，或加载新组件
定义组件：
    vincent_driver2.cpp
CMakeLists.txt：
    # 定义component 1：依赖项
    # find_package(rclcpp_components REQUIRED)

    # 定义component 2： add_executable 替换为 add_library
    add_library(vincent_driver2_component SHARED src/vincent_driver1.cpp) 

    # 定义component 3： 构建依赖
    ament_target_dependencies(
    vincent_driver2_component
    "rclcpp"
    "rclcpp_components" #新添加
    )
    # 定义component 4： 声明组件
    rclcpp_components_register_node(
    vincent_driver2_component
    PLUGIN "ns::VincentDriver"
    EXECUTABLE vincent_driver2
    )
    # 定义component 5： CMake中作用于该目标的安装命令，改为库安装
    ament_export_targets(export_vincent_driver2_component)
    install(TARGETS vincent_driver2_component
            EXPORT export_vincent_driver2_component
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )

启动方式：

1. 运行可执行文件 节点单独启动：$ ros2 run composition2 vincent_driver2
2. 使用composition动态加载组件：
    $ ros2 run rclcpp_components component_container
    $ ros2 component load /ComponentManager composition2 ns::VincentDriver
3. launch 启动
    $ ros2 launch composition2 vincent_driver2.launch.py
*/
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp> 

namespace ns{
class VincentDriver : public rclcpp::Node
{
public:
    VincentDriver() : Node("vincent_driver2_node")
    {
        RCLCPP_INFO(this->get_logger(), "Default construction: hello");
    }

    VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver_node",options)
    {
        RCLCPP_INFO(this->get_logger(), "With args(options) construction:hello options");
    }

private:

};
} // end namespace ns

RCLCPP_COMPONENTS_REGISTER_NODE(ns::VincentDriver)