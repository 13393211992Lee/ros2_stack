
/*
需求：将 IMU 数据写入 CSV 文件

编译：
$ colcon build --packages-select local_turtlebot3_test --symlink-install
$ source install/setup.bash

# 启动 turtlebot3仿真
$ ros2 launch turtlebot3_gazebo empty_world.launch.py 

# 该节点
 ros2 run local_turtlebot3_test turtlebot3_sensor_imu 

# turtlebot3 控制器
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args -p stamped:=true

*/
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

#include <fstream>  //文件操作
class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber_node")
    {
        declare_parameters();
        get_declare_parameters();

        // 通信
        init_communication();
        
        init_trajectory_log();
    }
    ~ImuSubscriber()
    {
        // 关闭轨迹日志文件
        if (traj_file_.is_open()) {
            traj_file_.close();
            RCLCPP_INFO(this->get_logger(), "轨迹信息保存在: %s 文件中" , traj_log_path_.c_str());
        }
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    // 写入csv文件的参数
    double log_interval_;           // 日志输出间隔(s)
    std::string traj_log_path_;     
    std::ofstream traj_file_; 
    rclcpp::Time last_log_time_ = this->now();
    // 1.声明 和获取 参数
    void declare_parameters(){
        this->declare_parameter<double>("log_interval",1.00);
        this->declare_parameter<std::string>("traj_log_path","imu_trajectory.csv");
    }
    void get_declare_parameters(){
        this->get_parameter("traj_log_path", traj_log_path_);
        this->get_parameter("log_interval", log_interval_);
    }

    // 通信
    void init_communication(){
        imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSubscriber::callback_imu, this, _1));
    }

    // 初始化轨迹日志
    void init_trajectory_log()
    {
        traj_file_.open(traj_log_path_, std::ios::out | std::ios::trunc);
        if (traj_file_.is_open()) {
            // 写入CSV表头
            traj_file_ << "时间戳(秒),时间戳(纳秒),";  // 时间戳
            traj_file_ << "加速度x(m/s²),加速度y(m/s²),加速度z(m/s²),";  // 线性加速度
            traj_file_ << "角速度x(rad/s),角速度y(rad/s),角速度z(rad/s),";  // 角速度
            traj_file_ << "四元数x,四元数y,四元数z,四元数w\n";  // 姿态四元数
        } else {
            RCLCPP_ERROR(this->get_logger(), "打开文件夹: %s失败", traj_log_path_.c_str());
        }
    }

    // 记录轨迹到文件
    void log_trajectory(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!traj_file_.is_open()) 
            return; // 文件未打开则退出

        // 写入数据（按表头顺序，用逗号分隔）
        // 1. 时间戳（秒 + 纳秒）
        traj_file_ << msg->header.stamp.sec << ","
                  << msg->header.stamp.nanosec << ",";

        // 2. 线性加速度（x,y,z）
        traj_file_ << msg->linear_acceleration.x << ","
                  << msg->linear_acceleration.y << ","
                  << msg->linear_acceleration.z << ",";

        // 3. 角速度（x,y,z）
        traj_file_ << msg->angular_velocity.x << ","
                  << msg->angular_velocity.y << ","
                  << msg->angular_velocity.z << ",";

        // 4. 四元数（x,y,z,w）
        traj_file_ << msg->orientation.x << ","
                  << msg->orientation.y << ","
                  << msg->orientation.z << ","
                  << msg->orientation.w << "\n";
        // 强制刷新缓冲区（确保数据实时写入文件，可选）
        traj_file_.flush();
    }


    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),1000,
            "\n Frame: %s, Time: %d.%09d"
            "\n Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f"
            "\n Angular Velocity角速度: x=%.2f, y=%.2f, z=%.2f rad/s"
            "\n Linear Acceleration线加速度: x=%.2f, y=%.2f, z=%.2f m/s²",
            //header
            msg->header.frame_id.c_str(),
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            //Orientation
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w,
            //Angular Velocity
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z,
            //linear Acceleration
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        // 定时记录轨迹（控制频率，避免高频写入）
        if ((this->now() - last_log_time_).seconds() >= log_interval_) {
            log_trajectory(msg);
            last_log_time_ = this->now();
        }

    }

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
}

/*
$ ros2 interface show sensor_msgs/msg/Imu

# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance # Row major x, y z

*/

/*
将 IMU 数据写入 CSV 文件

*/