// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "node_param/visibility_control.h"

/**
 * @brief 参数管理的基础示例
 * 声明参数、设置参数值，并通过参数服务客户端（SyncParametersClient）查询和列出指定参数及参数前缀
 * SyncParametersClient 同步客户端：
 *    调用set_parameters会等待结果返回.
 *    无需额外调用spin_until_future_complete
 *    代码流程更线性，适合简单同步场景
 * 
 * 启动方式1：$ ros2 run node_param  list_parameters
 * 启动方式2. 使用composition动态加载组件：
    $ ros2 run rclcpp_components component_container
    $ ros2 component load /ComponentManager node_param demo_nodes_cpp::ListParameters
 */
using namespace std::chrono_literals;
namespace node_param
{

class ListParameters : public rclcpp::Node
{
public:
  NODE_PARAM_PUBLIC
  explicit ListParameters(const rclcpp::NodeOptions & options)
  : Node("list_parameters", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // 禁用标准输出缓冲，确保日志实时打印
    // 声明参数
    this->declare_parameter("foo", 0);
    this->declare_parameter("bar", "");
    this->declare_parameter("baz", 0.);
    this->declare_parameter("foo.first", 0);  // 层级参数 foo.first（父参数 foo，子参数 first）
    this->declare_parameter("foo.second", 0); // 层级参数 foo.second
    this->declare_parameter("foobar", false);

    // 创建同步参数客户端并等待服务
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // 设置参数值
    RCLCPP_INFO(this->get_logger(), "Setting parameters...");
    // Set several different types of parameters.
    auto set_parameters_results = parameters_client->set_parameters(
      {
        rclcpp::Parameter("foo", 2),
        rclcpp::Parameter("bar", "hello"),
        rclcpp::Parameter("baz", 1.45),
        rclcpp::Parameter("foo.first", 8),
        rclcpp::Parameter("foo.second", 42),
        rclcpp::Parameter("foobar", true),
      });

    // 查询并列出参数
    RCLCPP_INFO(this->get_logger(), "Listing parameters...");
    // 查询前缀为 "foo" 或 "bar" 的参数，层级深度最大为 10
    auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);

    std::stringstream ss;
    ss << "\nParameter names:";
    for (auto & name : parameters_and_prefixes.names) { // 遍历所有匹配的参数名称
      ss << "\n " << name;
    }
    ss << "\nParameter prefixes:";
    for (auto & prefix : parameters_and_prefixes.prefixes) {  // 遍历所有匹配的参数前缀
      ss << "\n " << prefix;
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    rclcpp::shutdown();
  }
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(node_param::ListParameters)
