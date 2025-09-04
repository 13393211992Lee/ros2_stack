
#ifndef  CUSTOM_ACTION_INTERFACES_FIBONSCCI_HPP_
#define CUSTOM_ACTION_INTERFACES_FIBONSCCI_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_action_interfaces/action/fibonacci.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LocalActionServer : public rclcpp::Node
{
public:
  using Fibonacci =custom_action_interfaces::action::Fibonacci;
  LocalActionServer() : Node("local_action_server")
  {

    // ros2_action_server_init
    fibonacci_shared_ptr = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&LocalActionServer::goal_callback, this, _1, _2),
      std::bind(&LocalActionServer::cancel_callback, this, _1),
      std::bind(&LocalActionServer::accepted_callback, this, _1));
  }


private:
  // ros2_action_server
  rclcpp_action::Server<Fibonacci>::SharedPtr fibonacci_shared_ptr;

  // ros2_action_server_callback
      using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
      
      rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
      {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        if (false) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      
      rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      
      void accepted_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
      {
        std::thread(std::bind(&LocalActionServer::execute, this, _1), goal_handle).detach();
      }
      
      void execute(const std::shared_ptr<ServerGoalHandle> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<custom_action_interfaces::action::Fibonacci::Feedback>();
        auto result = std::make_shared<custom_action_interfaces::action::Fibonacci::Result>();
        rclcpp::Rate loop_rate(1);
        
        //逻辑
        result->sequence.push_back(1);
        result->sequence.push_back(3);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled %d ",result->sequence.at(0));
        RCLCPP_INFO(this->get_logger(), "Goal Canceled %d ",result->sequence.at(1));
        for (size_t i = 2; i <= goal->order; i++) {
          if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_ERROR(this->get_logger(), "Goal Canceled");
            return;
          }

          int next_num =  result->sequence[i-1] + result->sequence[i-2];
          result->sequence.push_back(next_num);

          //发送反馈
          feedback->partial_sequence = result->sequence;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled %d ",next_num);
          
          loop_rate.sleep();
        }
      
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Successfully executed goal");
      }
    


}; // end class LocalActionServer

#endif  // CUSTOM_ACTION_INTERFACES_FIBONSCCI_HPP_


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalActionServer>());
  rclcpp::shutdown();
  return 0;
}
