/*


需求：传感器预处理节点（sensor_preprocessor.cpp）

    对激光雷达数据进行过滤，去除地面点和噪声点
    对 IMU 数据进行低通滤波，减少高频噪声
    使用 message_filters 实现激光雷达、IMU 和里程计数据的时间同步
    发布处理后的传感器数据供 Cartographer 使用

测试：

1. turtlebot3:
ros2 launch turtlebot3_gazebo empty_world.launch.py 

2. node:
ros2 run local_turtlebot3_test turtlebot3_sensor_fusion4

3. launch
ros2 launch local_turtlebot3_test cartographer_fusion.launch.py 

对比策略 融合前后的话题消息 ：
rviz2 | matlib
均方根rmse指数 | 平均绝对误差mae

*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::placeholders;
using namespace message_filters;

class SensorPreprocessor : public rclcpp::Node
{
public:
    SensorPreprocessor() : Node("sensor_preprocessor")
    {
        // 声明参数
        declare_parameter("laser_ground_threshold", 0.1);  // 地面点阈值(m)
        declare_parameter("laser_noise_threshold", 0.5);   // 噪声点阈值(m)
        declare_parameter("imu_lowpass_alpha", 0.3);       // IMU低通滤波系数

        // 获取参数
        get_parameter("laser_ground_threshold", laser_ground_threshold_);
        get_parameter("laser_noise_threshold", laser_noise_threshold_);
        get_parameter("imu_lowpass_alpha", imu_lowpass_alpha_);

        // 订阅原始传感器话题
        laser_sub_.subscribe(this, "/scan");
        imu_sub_.subscribe(this, "/imu");
        odom_sub_.subscribe(this, "/odom");

        // 创建时间同步器 (允许±50ms的时间差)
        sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(
            MySyncPolicy(100), laser_sub_, imu_sub_, odom_sub_);
        sync_->registerCallback(std::bind(&SensorPreprocessor::sync_callback, this, _1, _2, _3));

        // 创建发布者
        filtered_laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
        filtered_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/filtered_imu", 10);
        synchronized_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/synchronized_odom", 10);

        RCLCPP_INFO(get_logger(), "Sensor preprocessor initialized");
    }

private:
    // 同步策略定义
    typedef sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, 
                                           sensor_msgs::msg::Imu, 
                                           nav_msgs::msg::Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;

    // 订阅者和同步器
    Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
    Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<Sync> sync_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr synchronized_odom_pub_;

    // 参数
    double laser_ground_threshold_;
    double laser_noise_threshold_;
    double imu_lowpass_alpha_;
    
    // 用于IMU低通滤波的历史数据
    sensor_msgs::msg::Imu last_imu_;
    bool has_last_imu_ = false;

    // 同步回调函数
    void sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser,
                      const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                      const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        // 处理激光雷达数据
        auto filtered_laser = filter_laser_scan(laser);
        
        // 处理IMU数据
        auto filtered_imu = filter_imu_data(imu);
        
        // 直接转发里程计数据(已同步)
        synchronized_odom_pub_->publish(*odom);
        
        // 发布处理后的传感器数据
        filtered_laser_pub_->publish(*filtered_laser);
        filtered_imu_pub_->publish(*filtered_imu);
    }

    // 激光雷达数据过滤
    std::unique_ptr<sensor_msgs::msg::LaserScan> filter_laser_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser)
    {
        auto filtered = std::make_unique<sensor_msgs::msg::LaserScan>(*laser);
        int valid_count = 0;  // 统计有效点数量
        float last_valid_range = -1.0;  // 过滤地面点和噪声点
        for (size_t i = 0; i < laser->ranges.size(); ++i)
        {
            float range = laser->ranges[i];

            // 过滤无效值（超出量程）
            if (range < laser->range_min || range > laser->range_max)
            {
                filtered->ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }
            
            // 仅在地面候选角度且距离过近时过滤（避免误删近距离障碍物）
            // if (is_ground_angle && range < laser_ground_threshold_)
            if (range < laser_ground_threshold_)
            {
                filtered->ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }


            
            // 过滤噪声点 (距离突变)
            if (last_valid_range > 0.0)
            {
                float diff = fabs(range - last_valid_range);
                // float dynamic_threshold = laser_noise_threshold_ * (1 + 0.1 * last_valid_range);  // 远处放宽阈值
                if (diff > laser_noise_threshold_)
                {
                    filtered->ranges[i] = std::numeric_limits<float>::infinity();
                    continue;
                }
            }
            
            last_valid_range = range;
            valid_count++;  // 有效点计数+1
        }
        // 关键：若所有点被过滤，返回原始数据（避免Cartographer无数据崩溃）
        if (valid_count == 0)
        {
            RCLCPP_WARN(get_logger(), "No valid laser points! Using original data.");
            return std::make_unique<sensor_msgs::msg::LaserScan>(*laser);
        }

        RCLCPP_DEBUG(get_logger(), "Laser filtered: %d valid points (total: %zu)", valid_count, laser->ranges.size());
        return filtered;
    }

    // IMU数据过滤
    std::unique_ptr<sensor_msgs::msg::Imu> filter_imu_data(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
    {
        auto filtered = std::make_unique<sensor_msgs::msg::Imu>(*imu);
        
        // 应用低通滤波减少高频噪声
        if (has_last_imu_)
        {
            // 角速度低通滤波
            filtered->angular_velocity.x = imu_lowpass_alpha_ * imu->angular_velocity.x + 
                                          (1 - imu_lowpass_alpha_) * last_imu_.angular_velocity.x;
            filtered->angular_velocity.y = imu_lowpass_alpha_ * imu->angular_velocity.y + 
                                          (1 - imu_lowpass_alpha_) * last_imu_.angular_velocity.y;
            filtered->angular_velocity.z = imu_lowpass_alpha_ * imu->angular_velocity.z + 
                                          (1 - imu_lowpass_alpha_) * last_imu_.angular_velocity.z;
                                          
            // 线加速度低通滤波
            filtered->linear_acceleration.x = imu_lowpass_alpha_ * imu->linear_acceleration.x + 
                                             (1 - imu_lowpass_alpha_) * last_imu_.linear_acceleration.x;
            filtered->linear_acceleration.y = imu_lowpass_alpha_ * imu->linear_acceleration.y + 
                                             (1 - imu_lowpass_alpha_) * last_imu_.linear_acceleration.y;
            filtered->linear_acceleration.z = imu_lowpass_alpha_ * imu->linear_acceleration.z + 
                                             (1 - imu_lowpass_alpha_) * last_imu_.linear_acceleration.z;
        }
        
        // 保存当前IMU数据用于下一次滤波
        last_imu_ = *filtered;
        has_last_imu_ = true;
        
        return filtered;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
