#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "chapt5_interface/srv/patrol.hpp"

using Patrol = chapt5_interface::srv::Patrol;


class TurtleControlClient : public rclcpp::Node
{
public:
    TurtleControlClient() : Node("turtle_control_lient")
    {

    }

private:
    rclcpp::Client</* srv_type */>::SharedPtr /* client_name */;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControlClient>());
    rclcpp::shutdown();
    return 0;
}
