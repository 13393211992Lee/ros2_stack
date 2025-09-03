/*
里程计话题 /odom（nav_msgs/msg/Odometry）
处理里程计消息

# 启动 turtlebot3仿真
$ ros2 launch turtlebot3_gazebo empty_world.launch.py 
# 该节点
$ ros2 run local_turtlebot3_test turtlebot3_sensor_odom0 
# turtlebot3 控制器
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args -p stamped:=true

*/
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("OdomSubscriber_Node")
    {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            rclcpp::SensorDataQoS(),  // 适用于传感器数据的QoS设置
            std::bind(&OdomSubscriber::odom_callback,this,std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Odometry 初始化完成. 订阅 /odom...中");
        RCLCPP_INFO(this->get_logger(), "Odometry subscriber initialized. Listening to /odom...");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        // 将四元数转换为欧拉角（滚转、俯仰、偏航）
    void quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat, 
                             double& roll, double& pitch, double& yaw)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);  // 直接初始化 在ros中推荐使用方式
        // tf2::Quaternion q = tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
        // auto q = tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);  // 单位：弧度
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs_){
        // 1. 提取头部信息
        std::string frame_id = odom_msgs_ -> header.frame_id;
        std::string child_frame_id = odom_msgs_ ->child_frame_id;
        auto stamp = odom_msgs_ ->header.stamp;
        // 2. 提取姿态信息（四元数转欧拉角）
        double roll, pitch, yaw;
        quaternion_to_euler(odom_msgs_ ->pose.pose.orientation ,roll, pitch, yaw);
        // 3. 提取位置信息（x, y, z）
        double x = odom_msgs_ ->pose.pose.position.x;
        double y = odom_msgs_ ->pose.pose.position.y;
        double z = odom_msgs_ ->pose.pose.position.z;
        // 4. 提取角速度信息（滚转、俯仰、偏航方向）
        double angular_x = odom_msgs_ ->twist.twist.angular.x;
        double angular_y = odom_msgs_ ->twist.twist.angular.y;
        double angular_z = odom_msgs_ ->twist.twist.angular.z;
        // 5. 提取线速度信息（x, y, z方向）
        double linear_x = odom_msgs_ ->twist.twist.linear.x;
        double linear_y = odom_msgs_ ->twist.twist.linear.y;
        double linear_z = odom_msgs_ ->twist.twist.linear.z;
        // 使用THROTTLE限制输出频率，避免刷屏
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),1000,
            "\n Odometry Data:"
            "\n  Timestamp: %d.%09d"
            "\n  Frame ID: %s, Child Frame ID: %s"
            "\n  Position (m): x=%.2f, y=%.2f, z=%.2f"
            "\n  Orientation (deg): roll=%.2f, pitch=%.2f, yaw=%.2f"
            "\n  Linear Velocity (m/s): x=%.2f, y=%.2f, z=%.2f"
            "\n  Angular Velocity (deg/s): x=%.2f, y=%.2f, z=%.2f",
            stamp.sec, stamp.nanosec,
            frame_id.c_str(), child_frame_id.c_str(),
            x, y, z,
            roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI,  // 转换为度
            linear_x, linear_y, linear_z,
            angular_x * 180/M_PI, angular_y * 180/M_PI, angular_z * 180/M_PI
        );
    }
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
