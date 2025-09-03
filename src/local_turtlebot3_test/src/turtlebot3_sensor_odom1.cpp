/*
    查阅odom2
*/
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>
#include <mutex>
#include <cmath>
#include <iomanip>

// 自定义消息类型：简化的机器人运动状态（实际项目中需在msg文件中定义）
struct RobotState {
    double x;          // x坐标(m)
    double y;          // y坐标(m)
    double yaw;        // 偏航角(deg)
    double linear_x;   // x方向线速度(m/s)
    double angular_z;  // z方向角速度(deg/s)
    rclcpp::Time stamp;// 时间戳
};

class OdomProcessor : public rclcpp::Node
{
public:
    OdomProcessor() : Node("odom_processor")
    {
        // 1. 声明参数（支持启动时配置）
        declare_parameters();

        // 2. 获取参数
        get_parameters();

        // 3. 初始化订阅者和发布者
        init_communication();


        // 4. 初始化轨迹记录文件
        init_trajectory_log();

        RCLCPP_INFO(this->get_logger(), "Odom processor initialized.");
        RCLCPP_INFO(this->get_logger(), "Motion boundary: x=[%.2f, %.2f], y=[%.2f, %.2f]",
                   x_min_, x_max_, y_min_, y_max_);
    }

    ~OdomProcessor()
    {
        // 关闭轨迹日志文件
        if (traj_file_.is_open()) {
            traj_file_.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory log saved to: %s", traj_log_path_.c_str());
        }
    }

private:
    // 核心参数
    double x_min_, x_max_;         // 运动边界x范围(m)
    double y_min_, y_max_;         // 运动边界y范围(m)
    double max_linear_speed_;      // 最大线速度阈值(m/s)
    double max_angular_speed_;     // 最大角速度阈值(rad/s)
    std::string traj_log_path_;    // 轨迹日志文件路径
    double log_interval_;          // 日志输出间隔(s)
    bool enable_stop_on_boundary_; // 超出边界时是否发送停止指令

    // 通信对象
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    // 状态变量
    RobotState current_state_;
    rclcpp::Time last_log_time_;
    std::ofstream traj_file_;
    std::mutex state_mutex_;       // 线程安全锁

    // 1. 声明参数
    void declare_parameters()
    {
        this->declare_parameter<double>("x_min", -5.0);
        this->declare_parameter<double>("x_max", 5.0);
        this->declare_parameter<double>("y_min", -5.0);
        this->declare_parameter<double>("y_max", 5.0);
        this->declare_parameter<double>("max_linear_speed", 1.0);
        this->declare_parameter<double>("max_angular_speed", 1.5);
        this->declare_parameter<std::string>("traj_log_path", "trajectory.csv");
        this->declare_parameter<double>("log_interval", 1.0);
        this->declare_parameter<bool>("enable_stop_on_boundary", false);
    }

    // 2. 获取参数
    void get_parameters()
    {
        this->get_parameter("x_min", x_min_);
        this->get_parameter("x_max", x_max_);
        this->get_parameter("y_min", y_min_);
        this->get_parameter("y_max", y_max_);
        this->get_parameter("max_linear_speed", max_linear_speed_);
        this->get_parameter("max_angular_speed", max_angular_speed_);
        this->get_parameter("traj_log_path", traj_log_path_);
        this->get_parameter("log_interval", log_interval_);
        this->get_parameter("enable_stop_on_boundary", enable_stop_on_boundary_);
    }

    // 3. 初始化通信
    void init_communication()
    {
        // 订阅里程计话题（使用传感器数据QoS）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&OdomProcessor::odom_callback, this, std::placeholders::_1)
        );

        // 发布停止指令（用于边界保护）
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 发布当前位置（供其他节点使用）
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    }

    // 4. 初始化轨迹日志
    void init_trajectory_log()
    {
        traj_file_.open(traj_log_path_, std::ios::out | std::ios::trunc);
        if (traj_file_.is_open()) {
            // 写入CSV表头
            traj_file_ << "timestamp,sec,nanosec,x(m),y(m),yaw(deg),linear_x(m/s),angular_z(deg/s)\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory log file: %s", traj_log_path_.c_str());
        }
    }

    // 5. 四元数转偏航角（简化版，只关注航向）
    double quat_to_yaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 弧度
        return yaw * 180.0 / M_PI;  // 转换为度
    }

    // 6. 检查是否超出运动边界
    bool is_out_of_boundary(double x, double y)
    {
        return (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_);
    }

    // 7. 检查速度是否异常
    bool is_speed_abnormal(double linear_x, double angular_z)
    {
        return (std::abs(linear_x) > max_linear_speed_ || 
                std::abs(angular_z) > max_angular_speed_);
    }

    // 8. 记录轨迹到文件
    void log_trajectory(const RobotState& state)
    {
        if (!traj_file_.is_open()) return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        // 写入CSV格式数据
        state.stamp.nanoseconds();
        traj_file_ << std::fixed << std::setprecision(6)
                   << state.stamp.seconds() << ","
                   << state.stamp.nanoseconds() << ","
                   << state.x << ","
                   << state.y << ","
                   << state.yaw << ","
                   << state.linear_x << ","
                   << state.angular_z << "\n";
    }

    // 9. 里程计回调函数（核心处理逻辑）
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 解析核心数据
        RobotState new_state;
        new_state.stamp = msg->header.stamp;
        new_state.x = msg->pose.pose.position.x;
        new_state.y = msg->pose.pose.position.y;
        new_state.yaw = quat_to_yaw(msg->pose.pose.orientation);
        new_state.linear_x = msg->twist.twist.linear.x;
        new_state.angular_z = msg->twist.twist.angular.z * 180.0 / M_PI; // 转度/秒

        // 线程安全更新状态
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = new_state;
        }

        // 功能1：发布简化的当前位置（供导航或可视化使用）
        publish_current_pose(msg);

        // 功能2：运动边界检查与保护
        if (is_out_of_boundary(new_state.x, new_state.y)) {
            RCLCPP_WARN(this->get_logger(), "Robot out of boundary! (x=%.2f, y=%.2f)", new_state.x, new_state.y);
            if (enable_stop_on_boundary_) {
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_cmd);
                RCLCPP_WARN(this->get_logger(), "Sent stop command due to boundary violation");
            }
        }

        // 功能3：速度异常检测
        if (is_speed_abnormal(new_state.linear_x, msg->twist.twist.angular.z)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Abnormal speed detected! linear=%.2f, angular=%.2f",
                                new_state.linear_x, msg->twist.twist.angular.z);
        }

        // 功能4：定时记录轨迹（控制频率，避免高频写入）
        if ((this->now() - last_log_time_).seconds() >= log_interval_) {
            log_trajectory(new_state);
            last_log_time_ = this->now();
        }

        // 功能5：定时打印关键状态（调试用）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "\nCurrent State:"
                            "\n  Position: (%.2f, %.2f), Yaw: %.2f°"
                            "\n  Speed: linear=%.2f m/s, angular=%.2f°/s",
                            new_state.x, new_state.y, new_state.yaw,
                            new_state.linear_x, new_state.angular_z);
    }

    // 发布当前位置（简化版）
    void publish_current_pose(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = odom_msg->header;
        pose_msg.pose = odom_msg->pose.pose;
        current_pose_pub_->publish(pose_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomProcessor>());
    rclcpp::shutdown();
    return 0;
}
