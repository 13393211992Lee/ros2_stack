
/*
需求：运动状态监测（静止 / 运动判断）
调整阈值，噪声敏感性 对运动状态的判断有很大影响

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

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber_node")
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
    bool is_moving_ = false;
    void declare_parameters(){
        this->declare_parameter<double>("accel_threshold",0.2);        // m/s²
        this->declare_parameter<double>("angular_threshold",0.1);      // rad/s
    }
    void get_declare_parameters(){
        this->get_parameter("accel_threshold",accel_threshold_);
        this->get_parameter("angular_threshold",angular_threshold_);
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

        // 计算加速度大小（去除重力影响：default静止时加速度≈9.8m/s²）
        double accel_linear = sqrt(
            pow(msg->linear_acceleration.x , 2)+
            pow(msg->linear_acceleration.y, 2)+
            pow(msg->linear_acceleration.z -9.8, 2)
        );
        // 计算角速度大小
        double angular_vel = sqrt(
            pow(msg->angular_velocity.x, 2)+
            pow(msg->angular_velocity.y, 2)+
            pow(msg->angular_velocity.z, 2)
        );
        // 判断运动状态
        // 用 was_moving 保存更新前的 is_moving_（即上一时刻状态），再与更新后的 is_moving_（当前状态）
        bool was_moving = is_moving_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),1000,
            "\n 当前加速度 accel_linear: %.2f"
            "\n 当前角速度 angular_vel: %.2f", 
            accel_linear, angular_vel
        );
        is_moving_ = (accel_linear > accel_threshold_) || (angular_vel > angular_threshold_);

        // 状态变化时打印日志
        if (is_moving_ && !was_moving)
        {
            RCLCPP_INFO(this->get_logger(), "Robot 开始运行...");
        }
        else if (!is_moving_ && was_moving)
        {
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