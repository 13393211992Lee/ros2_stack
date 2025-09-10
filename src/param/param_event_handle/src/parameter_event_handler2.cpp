#include <memory>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 监控参数的变化
 * 不采用lambda函数的实现方式
 * ParameterEventHandler  用于 监听参数变化
 * eg:
 * $ ros2 run param_event_handle  parameter_event_handler #启动后
 * $ ros2 param set node_with_parameters an_int_param 43    #修改参数
 * 这时 parameter_event_handler 可以监听到 参数的变化
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"

class MonitoringParamChange : public rclcpp::Node
{
public:
  MonitoringParamChange(std::string node_name) : Node(node_name)
  {
    //  声明参数
    this->declare_parameter("height",10);
    // 创建参数事件处理器
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    // 绑定参数回调到成员函数，使用std::bind将当前对象指针传入
    auto callback = std::bind(&MonitoringParamChange::on_parameter_changed ,this,std::placeholders::_1);
    // 添加被监控的参数：height
    cb_handle_ = param_event_handler_->add_parameter_callback("height", callback);
  }

private:
  // 参数事件处理器
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  // 回调句柄（用于管理回调生命周期）
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  // 参数变化的回调函数（替代lambda）
  void on_parameter_changed(const rclcpp::Parameter & p){
    RCLCPP_INFO(
      this->get_logger(), 
      "Callback: Parameter \"%s\" updated. New value: %ld (type: %s)",
      p.get_name().c_str(),
      p.as_int(),
      p.get_type_name().c_str()
    );
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitoringParamChange>("MonitoringParamChange_node"));
  rclcpp::shutdown();
  return 0;
}
