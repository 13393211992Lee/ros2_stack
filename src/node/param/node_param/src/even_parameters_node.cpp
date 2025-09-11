// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "node_param/visibility_control.h"

/**
 * @brief 参数验证与动态参数管理
 * 权限范围： 本节点下的参数
 * 触发类型：参数的新增| 修改| 删除| 均会触发. 查询 不会触发.
 * 
 * 启动方式1： ros2 run node_param even_parameters_node
 * 启动方式2： ros2 run rclcpp_components component_container
 *  ros2 component load /ComponentManager node_param  demo_nodes_cpp::EvenParameterNode
 */
namespace node_param
{
class EvenParameterNode : public rclcpp::Node
{
public:
  NODE_PARAM_PUBLIC         // 符号可见性控制宏，用于组件化加载
  explicit EvenParameterNode(rclcpp::NodeOptions options)
  : Node("even_parameters_node", options.allow_undeclared_parameters(true))
  {
    RCLCPP_INFO(get_logger(), "此示例节点展示了一个参数回调");
    RCLCPP_INFO(get_logger(), "该回调只接收 偶数的 参数更新");
    RCLCPP_INFO(get_logger(), "尝试运行“ros2 param set /来成功设置一个参数");
    RCLCPP_INFO(get_logger(), "eg: $  ros2 param set /even_parameters_node height 2");

    // 声明一个参数变更请求回调函数
    // 此函数将强制只允许设置偶数参数
    // 任何其他更改都将被丢弃
    auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters)
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          rclcpp::ParameterType parameter_type = parameter.get_type();
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_INFO(
              this->get_logger(), "parameter '%s' deleted successfully",
              parameter.get_name().c_str()
            );
            result.successful &= true;
          } else if (rclcpp::ParameterType::PARAMETER_INTEGER == parameter_type) {
            if (parameter.as_int() % 2 != 0) {
              RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Requested value '" << parameter.as_int() << "' for parameter '" <<
                  parameter.get_name() << "' is not an even number: rejecting change..."
              );
              result.reason = "only even integers(偶数) can be set";
              result.successful = false;
            } else {
              RCLCPP_INFO(
                this->get_logger(),
                "parameter '%s' has changed and is now: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str()
              );
              result.successful &= true;
            }
          } else {
            RCLCPP_INFO(
              this->get_logger(), "only integer parameters can be set\n"
              "requested value for parameter '%s' is not an even number（偶数）, rejecting change...",
              parameter.get_name().c_str()
            );
            result.reason = "only integer parameters can be set";
            result.successful = false;
          }
        }
        return result;
      };
    // callback_handler needs to be alive to keep the callback functional
    callback_handler = this->add_on_set_parameters_callback(param_change_callback);
  }

  OnSetParametersCallbackHandle::SharedPtr callback_handler;
};

}  // namespace node_param

RCLCPP_COMPONENTS_REGISTER_NODE(node_param::EvenParameterNode)
