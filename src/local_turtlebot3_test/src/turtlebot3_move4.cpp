/*
演示了一个简单的动作序列：前进 1 米 -> 右转 90 度 -> 前进 5 秒。


启动节点：
$ ros2 run local_turtlebot3_test turtlebot3m4

启动:
$ ros2 run turtlesim turtlesim_node --ros-args --remap /turtle1/cmd_vel:=/cmd_vel

*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class RobotMovementController : public rclcpp::Node
{
public:
    RobotMovementController() : Node("robot_movement_controller")
    {
        // 声明参数
        this->declare_parameter<double>("linear_speed", 0.2);    // 线速度(m/s)
        this->declare_parameter<double>("angular_speed", 0.5);   // 角速度(rad/s)
        this->declare_parameter<double>("move_distance", 1.0);   // 移动距离(m)
        this->declare_parameter<double>("rotate_angle", M_PI/2); // 旋转角度(rad)，默认90度
        this->declare_parameter<double>("move_time", 5.0);       // 移动时间(s)

        // 获取参数
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        this->get_parameter("move_distance", move_distance_);
        this->get_parameter("rotate_angle", rotate_angle_);
        this->get_parameter("move_time", move_time_);

        // 创建速度指令发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 创建计时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotMovementController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Robot movement controller initialized");
        RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f m/s", linear_speed_);
        RCLCPP_INFO(this->get_logger(), "Angular speed: %.2f rad/s", angular_speed_);
    }

    // 移动指定时间
    void move_for_time(double seconds)
    {
        stop_movement(); // 先停止任何当前运动
        movement_type_ = MovementType::TIME;
        target_duration_ = std::chrono::duration<double>(seconds);
        start_time_ = this->now();
        
        // 设置前进速度
        twist_msg_.linear.x = linear_speed_;
        twist_msg_.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Moving forward for %.2f seconds", seconds);
    }

    // 移动指定距离
    void move_distance(double distance)
    {
        stop_movement();
        movement_type_ = MovementType::DISTANCE;
        
        // 计算需要的时间：距离/速度
        double duration = distance / linear_speed_;
        target_duration_ = std::chrono::duration<double>(duration);
        start_time_ = this->now();
        
        // 设置前进速度
        twist_msg_.linear.x = linear_speed_;
        twist_msg_.angular.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Moving %.2f meters", distance);
    }

    // 旋转指定角度（弧度）
    void rotate_angle(double angle_rad)
    {
        stop_movement();
        movement_type_ = MovementType::ROTATION;
        
        // 计算需要的时间：角度/角速度
        double duration = std::abs(angle_rad) / angular_speed_;
        target_duration_ = std::chrono::duration<double>(duration);
        start_time_ = this->now();
        
        // 设置旋转速度（方向由角度符号决定）
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = angle_rad > 0 ? angular_speed_ : -angular_speed_;
        
        RCLCPP_INFO(this->get_logger(), "Rotating %.2f degrees", angle_rad * 180 / M_PI);
    }

    // 停止移动
    void stop_movement()
    {
        movement_type_ = MovementType::STOP;
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg_);
    }

    // 获取参数的方法，供main函数使用
    double get_linear_speed() const { return linear_speed_; }
    double get_angular_speed() const { return angular_speed_; }
    double get_move_distance() const { return move_distance_; }
    double get_rotate_angle() const { return rotate_angle_; }
    double get_move_time() const { return move_time_; }

private:
    enum class MovementType { STOP, TIME, DISTANCE, ROTATION } movement_type_ = MovementType::STOP;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_msg_;
    
    rclcpp::Time start_time_;
    std::chrono::duration<double> target_duration_;
    
    double linear_speed_;    // 线速度(m/s)
    double angular_speed_;   // 角速度(rad/s)
    double move_distance_;   // 默认移动距离(m)
    double rotate_angle_;    // 默认旋转角度(rad)
    double move_time_;       // 默认移动时间(s)

    void timer_callback()
    {
        if (movement_type_ == MovementType::STOP)
        {
            return;
        }

        // 检查是否达到目标时间
        auto elapsed = this->now() - start_time_;
        if (elapsed >= target_duration_)
        {
            RCLCPP_INFO(this->get_logger(), "Movement completed");
            stop_movement();
            return;
        }

        // 发布速度指令
        cmd_vel_pub_->publish(twist_msg_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMovementController>();
    
    // 演示：先前进1米，然后右转90度，然后前进2秒
    // 注意：在实际使用中，这些操作应该根据需要调用
    
    // 等待节点准备好
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // 前进指定距离（1米）
    double move_distance = node->get_move_distance();
    double linear_speed = node->get_linear_speed();
    node->move_distance(move_distance);
    rclcpp::spin_some(node);
    
    // 计算所需时间并转换为纳秒整数类型
    double distance_duration = move_distance / linear_speed;
    auto distance_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(distance_duration)
    );
    rclcpp::sleep_for(distance_duration_ns + std::chrono::seconds(1));
    
    // 右转90度（默认值）
    double rotate_angle = node->get_rotate_angle();
    double angular_speed = node->get_angular_speed();
    node->rotate_angle(rotate_angle);
    rclcpp::spin_some(node);
    
    // 计算旋转所需时间并转换为纳秒整数类型
    double rotate_duration = std::abs(rotate_angle) / angular_speed;
    auto rotate_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(rotate_duration)
    );
    rclcpp::sleep_for(rotate_duration_ns + std::chrono::seconds(1));
    
    // 前进指定时间（5秒）
    double move_time = node->get_move_time();
    node->move_for_time(move_time);
    rclcpp::spin_some(node);
    
    // 转换为纳秒整数类型
    auto move_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(move_time)
    );
    rclcpp::sleep_for(move_time_ns + std::chrono::seconds(1));
    
    // 最终停止
    node->stop_movement();
    
    rclcpp::shutdown();
    return 0;
}
