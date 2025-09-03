
/*
需求：运动状态监测（静止 / 运动判断）
问题: 调整阈值，噪声敏感性 对运动状态的判断有很大影响
方案： 
    1. 增加状态持续时间判断，过滤瞬时波动
    2. 对 IMU 数据进行平滑滤波，减少瞬时噪声
编译：
$ colcon build --packages-select local_turtlebot3_test --symlink-install
$ source install/setup.bash

# 启动 turtlebot3仿真
$ ros2 launch turtlebot3_gazebo empty_world.launch.py 

# 该节点
 ros2 run local_turtlebot3_test turtlebot3_sensor_imu 

# turtlebot3 控制器
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args -p stamped:=true

数据可视化
#$ ros2 run rqt_plot rqt_plot 
选择话题 和需要查看的消息
*/
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber_node"), 
        is_moving_(false), 
        moving_count_(0), 
        stopped_count_(0),
        filter_window_size_(5)  // 滑动平均窗口大小（5帧）
    {
        declare_parameters();
        get_declare_parameters();
        imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSubscriber::callback_imu, this, _1));
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    double accel_threshold_;
    double angular_threshold_;
    bool is_moving_;                    // 当前状态（经稳定性校验后）
    int moving_count_;                  // 连续“运动”状态帧数
    int stopped_count_;                 // 连续“静止”状态帧数
    const int stable_threshold_ = 10;    // 状态稳定所需的连续帧数（1帧≈0.02秒）
    int filter_window_size_;            // 滑动平均窗口大小
    std::deque<double> accel_history_;  // 加速度历史值队列（用于滤波）
    std::deque<double> angular_history_;// 角速度历史值队列（用于滤波）

    void declare_parameters(){
        this->declare_parameter<double>("accel_threshold",0.2);        //  加速度阈值 m/s²
        this->declare_parameter<double>("angular_threshold",0.1);      //  角速度阈值 rad/s
    }
    void get_declare_parameters(){
        this->get_parameter("accel_threshold",accel_threshold_);
        this->get_parameter("angular_threshold",angular_threshold_);
    }

    // 滑动平均滤波函数：计算最近 N 个数据点的平均值
    double moving_average(std::deque<double>& history, double new_value, int window_size) {
        history.push_back(new_value);
        if (history.size() > window_size) {
            history.pop_front();
        }
        double sum = 0.0;
        for (double val : history) sum += val;
        return sum / history.size();
    }

    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // 1. 计算原始加速度和角速度大小
        // （去除重力影响：default静止时加速度≈9.8m/s²）
        double raw_accel = sqrt(
            pow(msg->linear_acceleration.x , 2)+
            pow(msg->linear_acceleration.y, 2)+
            pow(msg->linear_acceleration.z -9.8, 2)
        );
        double raw_angular = sqrt(
            pow(msg->angular_velocity.x, 2)+
            pow(msg->angular_velocity.y, 2)+
            pow(msg->angular_velocity.z, 2)
        );

        // 2. 滑动平均滤波，减少噪声
        double accel_linear = moving_average(accel_history_, raw_accel, filter_window_size_);
        double angular_vel = moving_average(angular_history_, raw_angular, filter_window_size_);

        // 3. 基于单帧滤波后的数据，判断临时状态（未校验稳定性）
        bool temp_moving = (accel_linear > accel_threshold_) || (angular_vel > angular_threshold_);

        // 4. 状态稳定性校验：累计连续相同状态的帧数
        if (temp_moving) {
            moving_count_++;
            stopped_count_ = 0;  // 重置静止计数器
        } else {
            stopped_count_++;
            moving_count_ = 0;  // 重置运动计数器
        }

        // 5. 只有当连续stable_threshold_帧状态一致时，才更新最终状态
        bool was_moving = is_moving_;
        if (moving_count_ >= stable_threshold_) {
            is_moving_ = true;
        } else if (stopped_count_ >= stable_threshold_) {
            is_moving_ = false;
        }

        // 6. 状态变化时打印日志
        if (is_moving_ && !was_moving) {
            RCLCPP_INFO(this->get_logger(), "Robot 开始运行...");
        } else if (!is_moving_ && was_moving) {
            RCLCPP_INFO(this->get_logger(), "Robot 停止运行...");
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