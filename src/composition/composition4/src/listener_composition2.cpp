#include "composition4/listener_component2.hpp"

namespace composition
{
Listener_2::Listener_2() : Node("listener2")
{
    using std::placeholders::_1;
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&Listener_2::sub_callback, this, _1));
}

void Listener_2::sub_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
}
}  // namespace composition

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<composition::Listener_2>());
    rclcpp::shutdown();
    return 0;
}
