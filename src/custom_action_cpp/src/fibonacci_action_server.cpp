#include <functional>
#include <memory>
#include <thread>
// 路径：/home/zhang/ws_github_nav2/build/custom_action_interfaces/rosidl_generator_cpp/custom_action_interfaces/action/fibonacci.hpp
#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"
// url: https://github.com/ros2/examples/tree/jazzy/rclcpp
namespace custom_action_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    // 构造函数 explicit：防止隐式类型转换
  CUSTOM_ACTION_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    // 处理 goal
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Fibonacci::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // 处理 cancle
    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    // 处理一个 新的goal 并开始执行
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    // 构造函数还会实例化一个新的动作服务器
    /*
    action server 6个步骤：
    1. action type: Fibonacci
    2. 添加action 的节点： this
    3. action 名称："fibonacci"
    4. 处理 goal 的callback: handle_goal
    5. 处理取消的callback: handle_cancel
    6. 处理接受的callback: handle_accepted
    lambda实现方式
    */
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted
    );
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

};  // class FibonacciActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionServer)