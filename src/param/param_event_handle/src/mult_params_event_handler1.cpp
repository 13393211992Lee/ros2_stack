#include <memory>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 监控 多个 参数
 * 方式1： 为每个参数单独注册回调（适合少量参数）
 * ParameterEventHandler  用于 监听参数变化
 */
class MonitoringParamChange : public rclcpp::Node
{
public:
  MonitoringParamChange(std::string node_name) : Node(node_name)
  {
    //  声明参数
    this->declare_parameter("height",2);
    this->declare_parameter("length",5);
    this->declare_parameter("width", 3);
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    // 共用回调函数
    auto callback = std::bind(&MonitoringParamChange::on_parameter_changed ,this,std::placeholders::_1);
    // 添加被监控的参数：height
    cb_handle_height = param_event_handler_->add_parameter_callback("height", callback);
    cb_handle_width = param_event_handler_->add_parameter_callback("width", callback);
    cb_handle_length = param_event_handler_->add_parameter_callback("length", callback);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  // 回调句柄（用于管理回调生命周期）
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_height;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_width;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_length;
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
