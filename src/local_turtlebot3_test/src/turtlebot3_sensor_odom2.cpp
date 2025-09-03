
/*
需求：
    1. 实时解析里程计数据，提取位置 (x,y)、航向角 (yaw)、线速度、角速度
    2. 定时将轨迹数据写入 CSV 文件
    3. 运动边界保护
    4. 速度异常检测

# 启动 turtlebot3仿真
$ ros2 launch turtlebot3_gazebo empty_world.launch.py 

# 该节点
$ ros2 run local_turtlebot3_test turtlebot3_sensor_odom0 
# 节点启动带参数
$ ros2 run local_turtlebot3_test turtlebot3_sensor_odom2 --ros-args  -p x_min:=-3.0 -p x_max:=3.0   -p enable_stop_on_boundary:=true   -p traj_log_path:=./my_trajectory.csv

# turtlebot3 控制器
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args -p stamped:=true
*/


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>
#include "geometry_msgs/msg/twist_stamped.hpp"
// 自定义消息类型：简化的机器人运动状态（实际项目中需在msg文件中定义）
struct RobotState {
    double x;          // x坐标(m)
    double y;          // y坐标(m)
    double yaw;        // 偏航角(deg)
    double linear_x;   // x方向线速度(m/s)
    double angular_z;  // z方向角速度(deg/s)
    // rclcpp::Time stamp;// 时间戳
    builtin_interfaces::msg::Time stamp;   // 更换时间数据源
};

class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("OdomSubscriber_Node")
    {
        // 声明 获取 参数
        declare_parameters();
        get_parameters();
        current_parameters();
        // 通信
        init_communication();
        
        // 保存日志
        init_trajectory_log();
    }
    ~OdomSubscriber()
    {
        // 关闭轨迹日志文件
        if (traj_file_.is_open()) {
            traj_file_.close();
            RCLCPP_INFO(this->get_logger(), "轨迹信息保存在: %s 文件中" , traj_log_path_.c_str());
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

    // 状态变量
    std::ofstream traj_file_;
    rclcpp::Time last_log_time_ = this->now();
    std::mutex state_mutex_;
    RobotState current_state_;

    // 通信对象
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher;
    // 1.声明参数
    void declare_parameters(){
        this->declare_parameter<std::string>("traj_log_path", "trajectory.csv");
        this->declare_parameter<double>("log_interval", 1.0);
        this->declare_parameter<double>("x_min", 0.0);
        this->declare_parameter<double>("x_max", 7.0);
        this->declare_parameter<double>("y_min", 0.0);
        this->declare_parameter<double>("y_max", 7.0);
        this->declare_parameter<double>("max_linear_speed", 1.0);
        this->declare_parameter<double>("max_angular_speed", 1.0);
        this->declare_parameter<bool>("enable_stop_on_boundary", false);
        

    }

    // 2.获取参数
    void get_parameters(){
        this->get_parameter("traj_log_path", traj_log_path_);
        this->get_parameter("log_interval", log_interval_);
        this->get_parameter("x_min", x_min_);
        this->get_parameter("x_max", x_max_);
        this->get_parameter("y_min", y_min_);
        this->get_parameter("y_max", y_max_);
        this->get_parameter("max_linear_speed", max_linear_speed_);
        this->get_parameter("max_angular_speed", max_angular_speed_);
        this->get_parameter("enable_stop_on_boundary", enable_stop_on_boundary_);
    }

    // 输出当前参数
    void current_parameters(){
        RCLCPP_INFO_ONCE(this->get_logger(), 
            "加载的参数信息如下："
            "\n 边界x:%.2f %.2f,"
            "\n 边界y: %.2f %.2f"
            "\n 最大线速度：%.2f"
            "\n 最大角速度: %.2f"
            "\n 日志路径: %s"
            "\n 超出边界是否停止: %s", 
            x_min_, x_max_,
            y_min_, y_max_,
            max_linear_speed_,
            max_angular_speed_,
            traj_log_path_.c_str(),
            enable_stop_on_boundary_ ? "true" : "false"
        );
    }
    // 3.通信
    void init_communication(){
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            rclcpp::SensorDataQoS(),  // 适用于传感器数据的QoS设置
            std::bind(&OdomSubscriber::odom_callback,this,std::placeholders::_1)
        );

        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Odometry 初始化完成. 正在监听 /odom...");
    }
    // 4.初始化轨迹日志
    void init_trajectory_log()
    {
        traj_file_.open(traj_log_path_, std::ios::out | std::ios::trunc);
        if (traj_file_.is_open()) {
            // 写入CSV表头
            traj_file_ << "timestamp,sec,nanosec,x(m),y(m),yaw(deg),linear_x(m/s),angular_z(deg/s)\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "打开文件夹: %s失败", traj_log_path_.c_str());
        }
    }

    // 5.四元数转偏航角（简化版，只关注航向）
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
        traj_file_ << std::fixed << std::setprecision(6)
                   << state.stamp.sec << ","
                   << state.stamp.nanosec << ","
                   << state.x << ","
                   << state.y << ","
                   << state.yaw << ","
                   << state.linear_x << ","
                   << state.angular_z << "\n";
    }

    // 9. 里程计回调函数（核心处理逻辑）
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_){
        
        // 解析核心数据
        RobotState new_state;
        new_state.stamp = odom_msgs_->header.stamp;     // 不使用统一的时间源 会有异常
        new_state.x = odom_msgs_->pose.pose.position.x;
        new_state.y = odom_msgs_->pose.pose.position.y;
        new_state.yaw = quat_to_yaw(odom_msgs_->pose.pose.orientation);
        new_state.linear_x = odom_msgs_->twist.twist.linear.x;
        new_state.angular_z = odom_msgs_->twist.twist.angular.z * 180.0 / M_PI; // 转度/秒
        // 线程安全更新状态
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = new_state;
        }

        // 功能1：发布简化的当前位置（供导航或可视化使用）
        // publish_current_pose(odom_msgs_);

        // 功能2：运动边界检查与保护
        if (is_out_of_boundary(new_state.x, new_state.y)) {
            RCLCPP_WARN(this->get_logger(), "Robot out of boundary! (x=%.2f, y=%.2f)", new_state.x, new_state.y);
            if (enable_stop_on_boundary_) {
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = this->now();
                stop_cmd.twist.linear.x = 0.0;
                stop_cmd.twist.angular.z = 0.0;
                cmd_vel_publisher->publish(stop_cmd);
                RCLCPP_WARN(this->get_logger(), "由于进入设定边界, 发送停止指令");
            }
        }

        // 功能3：速度异常检测
        if (is_speed_abnormal(new_state.linear_x, new_state.angular_z)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "速度异常：速度超过阈值 .linear=%.2f, angular=%.2f",
                                new_state.linear_x, new_state.angular_z);
        }


        // 功能4：定时记录轨迹（控制频率，避免高频写入）
        if ((this->now() - last_log_time_).seconds() >= log_interval_) {
            log_trajectory(new_state);
            last_log_time_ = this->now();
        }
        // 功能5：定时打印关键状态（调试用）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "\n robot 当前状态:"
                            "\n  Position: (%.2f, %.2f), Yaw: %.2f°"
                            "\n  Speed: linear=%.2f m/s, angular=%.2f°/s",
                            new_state.x, new_state.y, new_state.yaw,
                            new_state.linear_x, new_state.angular_z);

    }


};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}

/*

RCLCPP_INFO_THROTTLE 控制输出速率
RCLCPP_INFO_ONCE    只输出一次             
*/