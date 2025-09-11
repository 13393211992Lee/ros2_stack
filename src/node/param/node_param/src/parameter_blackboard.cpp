// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>    // C++智能指针（管理对象生命周期，避免内存泄漏）
#include <string>    // C++字符串处理

// ROS 2参数相关服务接口：用于“列出参数”的服务定义
#include "rcl_interfaces/srv/list_parameters.hpp"
// ROS 2核心节点库：提供Node类、日志、参数管理等核心功能
#include "rclcpp/rclcpp.hpp"
// ROS 2组件注册库：用于将节点注册为“可动态加载的组件”（支持ROS 2的组件化机制）
#include "rclcpp_components/register_node_macro.hpp"

// 自定义节点的可见性控制：确保跨编译单元（.cpp/.h）的符号正确导出（通常在头文件中定义）
#include "node_param/visibility_control.h"

/**
 * @brief “参数黑板节点” 本质是一个 集中式参数存储服务
 * 
 * 启动方式： $ ros2 run node_param parameter_blackboard
 * 启动方式2：
 *  $ ros2 run rclcpp_components component_container
 *  $ ros2 component load /ComponentManager node_param node_param::ParameterBlackboard
 */
namespace node_param
{

class ParameterBlackboard : public rclcpp::Node
{
public:
  NODE_PARAM_PUBLIC
  explicit ParameterBlackboard(rclcpp::NodeOptions options)
  : Node(
      "parameter_blackboard",
      options.allow_undeclared_parameters(true).               // 允许向节点设置“未提前声明”的参数（默认不允许）
      automatically_declare_parameters_from_overrides(true))   // 自动将“外部覆盖的参数”声明为节点的参数
  {
    RCLCPP_INFO(
      this->get_logger(),
      "名为“%s”的参数黑板节点已就绪，并且已在提供“%zu”个参数！",
      this->get_fully_qualified_name(),     // 获取节点的“全限定名称”（格式：/命名空间/节点名，默认命名空间为/）
      this->list_parameters(  
        {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)  // 递归列出所有参数（包括嵌套参数）
        .names.size());
  }
};

}  // namespace node_param

RCLCPP_COMPONENTS_REGISTER_NODE(node_param::ParameterBlackboard)
