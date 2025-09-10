#include <memory>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 监控 多个 参数
 * 方式2：
 */
class MonitoringParamChange : public rclcpp::Node
{
public:
  MonitoringParamChange(std::string node_name) : Node(node_name)
  {
    this->declare_parameter("height",2);
    this->declare_parameter("length",5);
    this->declare_parameter("width", 3);
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto callback = std::bind(&MonitoringParamChange::on_parameter_changed ,this,std::placeholders::_1);

    // 修改 改为vector
    std::vector<std::string> params_to_monitor = {"height", "width", "length"};
    for (const auto &param : params_to_monitor)
    {
      RCLCPP_INFO(this->get_logger(), "param: %s" ,param.c_str());
      cb_handles_.push_back(param_event_handler_->add_parameter_callback(param, callback));
    }
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  //  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_group_; //避免用固定值接收
  // 改为：
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles_;
  // 参数变化的回调函数
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
