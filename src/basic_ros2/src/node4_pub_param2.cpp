#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
/*
节点里的参数 

0参启动：$ ros2 run basic_ros2 node4_pub_param2

带参数启动：$ ros2 run basic_ros2 node4_pub_param2 --ros-args -p my_parameter:='ccc'
修改参数：  $ ros2 param  set /minimal_param_node  my_parameter asdf
通过launch文件修改：
  $ ros2 launch basic_ros2 node4_pub_param2.launch.py 

*/
using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "我自定义的参数!";

    this->declare_parameter("my_parameter", "world" ,param_desc);

    auto timer_callback = [this](){
      std::string my_param = this->get_parameter("my_parameter").as_string();

      RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
      this->set_parameters(all_new_parameters);
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}