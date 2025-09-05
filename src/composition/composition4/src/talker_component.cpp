#include "composition4/talker_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

namespace composition
{
    Talker::Talker(const rclcpp::NodeOptions & options)
    :Node("talker", options),count_(0)
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
        timer_= create_wall_timer(1s,[this]() {return this->on_timer();} );
    }

    void Talker::on_timer(){
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "hi" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "发布： %s" ,msg->data.c_str());
        std::flush(std::cout);
        pub_ ->publish(std::move(msg));

    }
}  // namespace /* namespace_name */
