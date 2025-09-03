#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

class TurtleBot3MoveNode : public rclcpp::Node
{
public:
    TurtleBot3MoveNode() : Node("turtlebot3_move_node")
    {
        // 声明并获取参数
        this->declare_parameter("linear_speed", 0.2);  // 线速度(m/s)
        this->declare_parameter("angular_speed", 0.5); // 角速度(rad/s)
        this->declare_parameter("loop_rate", 10.0);    // 发布频率(Hz)
        
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        double loop_rate;
        this->get_parameter("loop_rate", loop_rate);
        
        // 创建速度指令发布者 (使用TwistStamped类型)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10);
        
        // 创建控制服务
        control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/control_movement",
            std::bind(&TurtleBot3MoveNode::control_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建定时器用于发布速度指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / loop_rate)),
            std::bind(&TurtleBot3MoveNode::publish_velocity, this));
        
        // 初始化移动状态
        is_moving_ = false;
        
        RCLCPP_INFO(this->get_logger(), "TurtleBot3移动控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "线速度: %.2f m/s, 角速度: %.2f rad/s",
                   linear_speed_, angular_speed_);
    }

private:
    void control_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        is_moving_ = request->data;
        
        if (is_moving_)
        {
            RCLCPP_INFO(this->get_logger(), "开始移动");
            response->success = true;
            response->message = "机器人开始移动";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "停止移动");
            response->success = true;
            response->message = "机器人已停止";
        }
    }
    
    void publish_velocity()
    {
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        
        // 设置时间戳
        twist_msg.header.stamp = this->now();
        // 设置坐标系 (通常为base_link或base_footprint)
        twist_msg.header.frame_id = "base_link";
        
        if (is_moving_)
        {
            // 移动时设置速度
            twist_msg.twist.linear.x = linear_speed_;  // 前进
            twist_msg.twist.angular.z = angular_speed_; // 旋转
        }
        else
        {
            // 停止时速度为0
            twist_msg.twist.linear.x = 0.0;
            twist_msg.twist.angular.z = 0.0;
        }
        
        cmd_vel_pub_->publish(twist_msg);
    }
    
    // 节点成员变量
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool is_moving_;
    double linear_speed_;
    double angular_speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleBot3MoveNode>());
    rclcpp::shutdown();
    return 0;
}
    
/*


启动t3
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

运行移动控制节点
$ ros2 run local_turtlebot3_test turtlebot3_move 

移动
ros2 service call /control_movement std_srvs/srv/SetBool "{data: true}"
停止
ros2 service call /control_movement std_srvs/srv/SetBool "{data: false}"

速度参数：
ros2 param set /turtlebot3_move_node linear_speed 0.3
ros2 param set /turtlebot3_move_node angular_speed 0.0  # 只前进不旋转


*/