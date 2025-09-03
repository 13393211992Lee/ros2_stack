/*
航点跟随：
给它设定 4个航点（A → B → C → D）。
它会自动依次导航到每个点。

编译：
$ colcon build --packages-select local_turtlebot3_test --symlink-install
source install/setup.bash

仿真：
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

该节点：
$ ros2 run local_turtlebot3_test turtlebot3m5 

移动机器人
 ros2 run teleop_twist_keyboard  teleop_twist_keyboard  --ros-args --param stamped:=true  

查看机器人话题：
$ ros2 topic info /cmd_vel --verbose


概述流程：
在 Gazebo 里把一只 Box 放到 肉眼认为的 turtlebot3 正前方。
用键盘控制节点把机器人向前开，却发现：
    激光点云在 rviz2 里确实能看到 Box 的红点；
    但代码取 ranges[size/2]（0° 射线）时，返回的是 inf。
于是你怀疑“激光雷达的 0° 射线没打中 Box”。
进一步用键盘向前/后移动机器人，发现：
只有当机器人“倒退”时，0° 射线才测到 Box；向前反而测不到。
结论：turtlebot3 的激光雷达 0° 射线并非从车体几何正前方射出，而是从几何正后方射出。

答案（一句话）：
turtlebot3 的 LDS-01 激光雷达默认把 0° 定义为“车体尾部方向”，180° 才是“车体正前方”。
所以 Box 放在“车体正前方”时，对应的是 scan 中的 180° 索引，而非 0° 索引。
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>

// 自定义clamp函数，兼容C++17之前的标准
template <typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 定义航点结构体
struct Waypoint {
    double x;  // x坐标
    double y;  // y坐标
    Waypoint(double x_, double y_) : x(x_), y(y_) {}
};

class WaypointFollower : public rclcpp::Node
{
public:
    WaypointFollower() : Node("waypoint_follower"), current_waypoint_idx_(0)
    {
        // 初始化航点列表
        waypoints_ = {
            Waypoint(1.0, 0.0),   
            Waypoint(1.0, 1.0),   
            Waypoint(0.0, 1.0),   
            Waypoint(0.0, 0.0)    
        };

        // 创建速度指令发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        // cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 创建里程计订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&WaypointFollower::odom_callback, this, std::placeholders::_1)
        );
        
        // 创建控制循环定时器（10Hz）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointFollower::control_loop, this)
        );

        // 初始化机器人位置和姿态
        current_x_ = 5.5;
        current_y_ = 5.5;
        current_yaw_ = 0.0;

        // 控制参数
        distance_tolerance_ = 0.1;  // 到达航点的距离 tolerance (米)
        angle_tolerance_ = 0.3;     // 角度 tolerance (弧度)
        max_linear_speed_ = 0.2;    // 最大线速度 (m/s)
        max_angular_speed_ = 0.5;   // 最大角速度 (rad/s)

        RCLCPP_INFO(this->get_logger(), "航点跟随节点已启动，共%d个航点", (int)waypoints_.size());
    }

private:
    // 里程计回调函数：更新机器人当前位置和姿态
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新位置
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // 从四元数计算航向角yaw（绕z轴旋转角度）
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = 2 * atan2(qz, qw);  // 范围：-π ~ π
        // RCLCPP_INFO(this->get_logger(), "当前位置(%.2f ,%.2f),角度为：",current_x_ ,current_y_,current_yaw_);
    }

    // 控制循环：计算并发布速度指令
    void control_loop()
    {
        // 检查是否所有航点都已到达
        if (current_waypoint_idx_ >= waypoints_.size()) {
            stop_robot();
            RCLCPP_INFO_ONCE(this->get_logger(), "所有航点已到达！");
            return;
        }
        // 获取当前目标航点
        Waypoint target = waypoints_[current_waypoint_idx_];
        // RCLCPP_INFO_ONCE(this->get_logger(), "当前目标航点%.2f", target);
        // 计算到目标航点的距离和角度
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance_to_target = sqrt(dx * dx + dy * dy);
        double target_yaw = atan2(dy, dx);
        // 计算航向角误差（归一化到-π到π）
        double yaw_error = target_yaw - current_yaw_;
        // RCLCPP_INFO(this->get_logger(), "机器人需要的朝向 和 当下机器人实际的朝向 角度两者的差值(%.2f）",yaw_error);

        if (yaw_error > M_PI) {
            yaw_error -= 2 * M_PI;
        } else if (yaw_error < -M_PI) {
            yaw_error += 2 * M_PI;
        }
        // RCLCPP_INFO(this->get_logger(), "归一化(+- 2pi) yaw_error: %.2f",yaw_error);
        // 创建速度消息
        auto twist_stamped = geometry_msgs::msg::TwistStamped();

        // 检查是否到达当前航点
        if (distance_to_target < distance_tolerance_) {
            stop_robot();
            current_waypoint_idx_++;
            
            if (current_waypoint_idx_ < waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "已到达航点 %ld，前往下一个航点: (%.2f, %.2f)",
                           current_waypoint_idx_,
                           waypoints_[current_waypoint_idx_].x, 
                           waypoints_[current_waypoint_idx_].y);
            }
            return;
        }

        // 控制逻辑：先转向目标方向，再前进
        twist_stamped.header.stamp = this->now();
        twist_stamped.header.frame_id = "base_link";
        if (fabs(yaw_error) > angle_tolerance_) {
            // 转向目标方向（只旋转）
            // 使用自定义的clamp函数限制角速度
            twist_stamped.twist.angular.z = clamp(2.0 * yaw_error, -max_angular_speed_, max_angular_speed_);
            twist_stamped.twist.linear.x = 0.0;
        } else {
            // 已朝向目标，可以前进
            // 使用自定义的clamp函数限制线速度和角速度
            twist_stamped.twist.linear.x = clamp(0.5 * distance_to_target, 0.0, max_linear_speed_);
            twist_stamped.twist.angular.z = clamp(2.0 * yaw_error, -max_angular_speed_, max_angular_speed_);
        }

        // 发布速度指令
        cmd_vel_pub_->publish(twist_stamped);
    }

    // 停止机器人
    void stop_robot()
    {
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.twist.linear.x = 0.0;
        twist_stamped.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_stamped);
    }

    // 成员变量
    std::vector<Waypoint> waypoints_;           
    size_t current_waypoint_idx_;               
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double current_x_, current_y_, current_yaw_;
    double distance_tolerance_, angle_tolerance_;
    double max_linear_speed_, max_angular_speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollower>());
    rclcpp::shutdown();
    return 0;
}
    
