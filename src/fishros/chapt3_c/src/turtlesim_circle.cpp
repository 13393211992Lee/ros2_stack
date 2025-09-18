/*
<<  m  ,  >>
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"




namespace turtle_control
{
class TurtleControlCircle : public rclcpp::Node
{
public:
    TurtleControlCircle() : Node("turtle_draw_circle")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleControlCircle:: timer_cb, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_cb()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x= 0.3;
        msg.angular.z = 0.3;
        pub_->publish(msg);
    }
};
}  // namespace turtle_control


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtle_control::TurtleControlCircle>());
    rclcpp::shutdown();
    return 0;
}
