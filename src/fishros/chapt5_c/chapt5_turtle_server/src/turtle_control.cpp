#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "cmath"
#include "geometry_msgs/msg/twist.hpp"
/**
 * @brief  
 * /turtle1/pose [turtlesim/msg/Pose]
 * /turtle1/cmd_vel [geometry_msgs/msg/Twist]
 * 1. 订阅当前乌龟的位置
 * 2. 计算线性速度
 * 3. 发布
 */


using Pose = turtlesim::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
class TurtleControl : public rclcpp::Node
{
public:
    TurtleControl() : Node("turtle_control_node")
    {
        using std::placeholders::_1;
        sub_turtle_pose_ = this->create_subscription<Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleControl::sub_turtle_cb, this, _1));

        pub_turtle_twist_= this->create_publisher<Twist>("/turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleControl::pub_turtle_twist, this));

    }

private:
    // 目标位置
    double goal_x = 3.0;
    double goal_y = 4.0;
    // 比例系数
    double k_linear = 0.9;    // 线性速度比例系数
    double k_angular = 0.6;   // 角速度比例系数
    // 当前位置
    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;

    rclcpp::Publisher<Twist>::SharedPtr pub_turtle_twist_;
    rclcpp::Subscription<Pose>::SharedPtr sub_turtle_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pub_turtle_twist()
    {
        auto twist = calculate_twist();
        pub_turtle_twist_->publish(twist);
    }

    Twist calculate_twist(){
        Twist twist;
        float distance_x = goal_x - current_x;
        float distance_y = goal_y - current_y;

        // 计算两点距离
        float distance_2point = std::sqrt(distance_x * distance_y + distance_y * distance_y);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        //                         "两点距离:%2.f",distance_2point);
        
        // 如果距离目标点足够近，则停止移动
        if(distance_2point < 0.1){
            twist.linear.x=0.0;
            twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "已到达目标点!");
            return twist;
        }

        // 计算目标方向角度
        double target_angle = atan2(distance_y , distance_x);
        double angle_diff = target_angle- current_theta;
        // 角度差 归一到 [-pi pi]
        if (angle_diff > M_PI) {
            angle_diff -= 2 * M_PI;
        } else if (angle_diff < -M_PI) {
            angle_diff += 2 * M_PI;
        }

        // 计算速度指令 1  先运动 后转向
        // if (distance_2point > 0.5) {
        //     twist.angular.z = k_angular * angle_diff; // 调整方向 比例控制角速度
        //     twist.linear.x = distance_2point * k_linear;
            
        //     if (twist.linear.x > 1.0) {// 限制最大线速度
        //         twist.linear.x = 1.0;
        //     }
        //     } else {
        //         twist.linear.x = 0.0;
        //         twist.angular.z = 0.0;
                
        //         //与目标点的剩余距离<0.5m,结束导航
        //         // RCLCPP_INFO(this->get_logger(), "已经导航至目标点附近。");
        //     }
        
        // 计算速度指令
        twist.angular.z = k_angular * angle_diff;
        // 当角度偏差较小时才前进，避免在转向时前进
        if (fabs(angle_diff) < 0.9) {  // 约17度的阈值
            twist.linear.x = k_linear * distance_2point;
            // 限制最大线速度
            if (twist.linear.x > 1.5) {
                twist.linear.x = 1.5;
            }
        } else {
            twist.linear.x = 0.0;  // 角度偏差大时只转向不前进
        }
        
        return twist;
    }
    void sub_turtle_cb(const Pose::SharedPtr msg)
    {
        // 更新当前位置信息
        current_x = msg->x;
        current_y = msg->y;
        current_theta = msg->theta;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControl>());
    rclcpp::shutdown();
    return 0;
}
