/*
需求：   自定义融合节点（基于互补滤波）
        高频：信任 IMU 的角速度和加速度  & 抑制 Odom 的高频噪声
        长期：信任 Odom 的位置和姿态    & 修正 IMU 的漂移

IMU 的优势在于高频响应，能快速捕捉瞬间的运动（比如角速度、加速度的快速变化），但长期会漂移（低频误差累积）。
Odom（比如轮式里程计）在低频时更稳定，位置和姿态的长期趋势可靠，但对高频运动响应差，容易有高频噪声（比如轮子打滑带来的突变）。

*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
using std::placeholders::_1;
using std::placeholders::_2;

class ImuOdomFusion : public rclcpp::Node
{
public:
    ImuOdomFusion() : Node("imu_odom_fusion")
    {
        // 订阅IMU和Odom话题（时间同步）
        imu_sub_.subscribe(this, "/imu");
        odom_sub_.subscribe(this, "/odom");
        
        // 近似时间同步（允许微小时间差）
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), imu_sub_, odom_sub_);

        // 注册回调函数，当两个话题的消息同步后调用
        sync_->registerCallback(std::bind(&ImuOdomFusion::callback, this, _1 , _2));

        // 发布融合后的位姿
        fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);

        // 初始化互补滤波参数
        alpha_ = 0.9;  // IMU高频信任权重
        beta_ = 0.1;   // Odom长期校正权重
        last_time_ = this->get_clock()->now();
        is_initialized_ = false;
    }

private:
    // 消息过滤器订阅者
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    // 定义同步策略类型
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::msg::Imu, nav_msgs::msg::Odometry> MySyncPolicy;
    // 同步器
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;

    // 互补滤波参数
    double alpha_;
    double beta_;
    rclcpp::Time last_time_;
    bool is_initialized_;
    nav_msgs::msg::Odometry fused_odom_;

    // odom_imu 融合方法入口
    void fused_odom_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                        const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {

        // 计算时间差
        rclcpp::Duration dt = this->get_clock()->now() - last_time_;
        last_time_ = this->get_clock()->now();

        // 初始化
        if (!is_initialized_) {
            fused_odom_ = *odom;
            is_initialized_ = true;
            return;
        }
        
        nav_msgs::msg::Odometry fused;
        fused.header.stamp = this->now();
        fused.header.frame_id = "map";
        fused.child_frame_id ="odom";

        // 1. 姿态融合：短期信任IMU角速度积分，长期信任Odom姿态
        // 从IMU角速度积分计算姿态变化（简化）
        tf2::Quaternion imu_quat(
            imu->orientation.x,
            imu->orientation.y,
            imu->orientation.z,
            imu->orientation.w
        );

        // 从里程计获取参考姿态
        tf2::Quaternion odom_quat(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w
        );
        // 互补滤波融合姿态：高频IMU + 低频Odom校正
        tf2::Quaternion fused_quat = imu_quat.slerp(odom_quat, beta_);//beta_:0 imu_quat ||  beta_:1 odom_quat
        fused.pose.pose.orientation.x = fused_quat.x();
        fused.pose.pose.orientation.y = fused_quat.y();
        fused.pose.pose.orientation.z = fused_quat.z();
        fused.pose.pose.orientation.w = fused_quat.w();

        // 2. 位置融合：长期信任Odom，短期用IMU加速度修正
        // 计算IMU加速度积分得到的位置变化
        double dt_sec = dt.seconds();
        geometry_msgs::msg::Point imu_pos_delta;
        imu_pos_delta.x = imu->linear_acceleration.x * dt_sec * dt_sec;
        imu_pos_delta.y = imu->linear_acceleration.y * dt_sec * dt_sec;
        imu_pos_delta.z = imu->linear_acceleration.z * dt_sec * dt_sec;

        // 互补滤波融合位置
        fused.pose.pose.position.x = alpha_ * (fused_odom_.pose.pose.position.x + imu_pos_delta.x) + 
                                   beta_ * odom->pose.pose.position.x;
        fused.pose.pose.position.y = alpha_ * (fused_odom_.pose.pose.position.y + imu_pos_delta.y) + 
                                   beta_ * odom->pose.pose.position.y;
        fused.pose.pose.position.z = alpha_ * (fused_odom_.pose.pose.position.z + imu_pos_delta.z) + 
                                   beta_ * odom->pose.pose.position.z;

        // 3. 速度融合
        fused.twist.twist.linear.x = alpha_ * (fused_odom_.twist.twist.linear.x + imu->linear_acceleration.x * dt_sec) +
                                    beta_ * odom->twist.twist.linear.x;
        fused.twist.twist.linear.y = alpha_ * (fused_odom_.twist.twist.linear.y + imu->linear_acceleration.y * dt_sec) +
                                    beta_ * odom->twist.twist.linear.y;
        fused.twist.twist.linear.z = alpha_ * (fused_odom_.twist.twist.linear.z + imu->linear_acceleration.z * dt_sec) +
                                    beta_ * odom->twist.twist.linear.z;

        // 保存当前融合结果用于下一帧计算
        fused_odom_ = fused;

        fused_pub_->publish(fused);
    }
    // 融合回调函数
    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu, 
                 const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        fused_odom_imu(imu, odom);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdomFusion>());
    rclcpp::shutdown();
    return 0;
}

/*
--------------------nav_msgs/msg/Odometry--------------------
$ ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose
	Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	float64[36] covariance

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
	Twist twist
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	float64[36] covariance




---------------------sensor_msgs/msg/Imu---------------------
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