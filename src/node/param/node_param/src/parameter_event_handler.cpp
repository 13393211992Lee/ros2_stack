// Copyright 2021 Open Source Robotics Foundation, Inc.
// Copyright (c) 2020 Intel Corporation
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

#include <cinttypes>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"

// A utility class to assist in spinning a separate node
/**
 * @brief 一个辅助创建独立节点的实用类
 */
class NodeThread
{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : node_(node_base)
  {
    thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_.add_node(node_);  // 将节点添加到执行器
        executor_.spin();           // 执行器开始循环处理回调
        executor_.remove_node(node_);
      });
  }

  // 模板构造函数：支持直接传入节点对象
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  // 析构函数：停止执行器并等待线程结束
  ~NodeThread()
  {
    executor_.cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  const char * node_name = "node";
  const char * param_name = "age";

  // 创建本地节点并声明一个整数参数
  auto node = rclcpp::Node::make_shared(node_name);
  node->declare_parameter(param_name, 0);

  // 创建远程节点（在单独命名空间）并声明一个字符串参数
  auto remote_node_name = "remote_node";
  auto remote_node_namespace = "/ns";
  auto remote_param_name = "name";
  auto remote_node = rclcpp::Node::make_shared(remote_node_name, remote_node_namespace);
  remote_node->declare_parameter(remote_param_name, "cici");
  auto remote_thread = std::make_unique<NodeThread>(remote_node);     // 在单独线程运行

  // 创建参数订阅者用于监控参数变化（监控本地节点和远程节点）
  // ParameterEventHandler是 ROS 2 中用于监控参数变化的核心类
  auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node);

  // 首先，为本地整数参数设置一个回调函数。在本例中，我们不提供节点名称（第三个可选参数）。
  // 本地参数回调：监听当前节点的特定参数变化
  auto cb1 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(),
        "cb1: 接收到更新指令！参数： \"%s\" 参数类型 %s: \"%" PRId64 "\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_int());
    };
  auto handle1 = param_subscriber->add_parameter_callback(param_name, cb1);

  // 远程参数回调：监听指定远程节点的特定参数变化
  auto cb2 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(), "cb2: 接收到更新指令！参数： \"%s\" 类型: %s: \"%s\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_string().c_str());
    };
  auto fqn = remote_node_namespace + std::string("/") + remote_node_name; // 远程节点的全限定名
  auto handle2 = param_subscriber->add_parameter_callback(
    remote_param_name, cb2, fqn);

  // 监听所有参数事件并进行自定义过滤
  auto cb3 =
    [fqn, remote_param_name, &node](const rcl_interfaces::msg::ParameterEvent & event) {
      // 使用正则表达式过滤感兴趣的节点
      std::regex re("(/n/.*)|(/ci)");
      if (regex_match(event.node, re)) {

        // 方法1：获取特定参数
        rclcpp::Parameter p;
        if (rclcpp::ParameterEventHandler::get_parameter_from_event(
            event, p,
            remote_param_name, fqn))
        {
          RCLCPP_INFO(
            node->get_logger(), "cb3:  接收到更新指令！ parameter \"%s\" of type: %s: \"%s\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_string().c_str());
        }

        // 方法2：枚举事件中的所有参数
        auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
        for (auto & p : params) {
          RCLCPP_INFO(
            node->get_logger(), "cb3:  接收到更新指令！ parameter \"%s\" of type: %s: \"%s\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.value_to_string().c_str());
        }
      }
    };
  auto handle3 = param_subscriber->add_parameter_event_callback(cb3);

  printf("此演示正在监控以下参数的变化:\n\n");
  printf("\tnode: \"%s\"\n", node_name);
  printf("\tparameter: \"%s\"\n", param_name);
  printf("\n");
  printf("\tnode: \"%s\"\n", fqn.c_str());
  printf("\tparameter: \"%s\"\n", remote_param_name);
  printf("\n");
  printf(
    "要在单独的shell/控制台窗口中激活回调 "
    "执行以下任一示例命令s:\n\n");
  printf("\t$ ros2 param set %s %s 21\n", node_name, param_name);
  printf("\t$ ros2 param set %s %s \"string value to set\"\n\n", fqn.c_str(), remote_param_name);

  // Process messages until 
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
