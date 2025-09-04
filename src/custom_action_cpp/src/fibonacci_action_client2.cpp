
#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

class ActionClient : public rclcpp::Node
{
public:
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  ActionClient() : Node("action_client_node")
  {
    RCLCPP_INFO(this->get_logger(), "hello");
    //创建action client
    fibonacci_ = rclcpp_action::create_client<Fibonacci>(
      this, "fibonacci");
    
    // 等待 action server 启动
    while (!fibonacci_->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待 action server 时被关闭！");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "继续等待 Action server ");
    }

    // ros2 action client asyc send goal
    using std::placeholders::_1;
    using std::placeholders::_2;
    auto goal = Fibonacci::Goal();
    goal.order = 10;
    auto send_goal_options =
      rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionClient::result_callback, this, _1);
    RCLCPP_INFO(this->get_logger(), "发送 goal:%d",goal.order);

    fibonacci_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr fibonacci_ ;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

  // goal 的合法性
  void goal_response_callback(const ClientGoalHandle::SharedPtr & future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal 被 action server 拒绝");
    } else {
      RCLCPP_INFO(this->get_logger(), "action server 接受 goal, 等待结果");
    }
  }
  
  // 连续反馈
  void feedback_callback(
    ClientGoalHandle::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    int32_t per_sequence = 0;
    for (int32_t number : feedback->partial_sequence) {
      per_sequence = number;
    }
    RCLCPP_INFO(this->get_logger(), "Received feedback %d",per_sequence);
  }
  
  //最终结果
  void result_callback(const ClientGoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action Server 成功执行该 goal");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal 不被允许");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal 被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知的返回结果");
        return;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionClient>());
  rclcpp::shutdown();
  return 0;
}
