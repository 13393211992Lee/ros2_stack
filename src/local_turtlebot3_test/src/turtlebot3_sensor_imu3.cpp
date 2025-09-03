// m , ->
// 四元数 和欧拉角的转换

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
using std::placeholders::_1;

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("ImuSubscriber_Node")
    {
        imu_subcriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSubscriber::imu_subs_callback, this, _1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subcriber;
    double roll;
    double pitch;
    double yaw;
    double roll2;
    double pitch2;
    double yaw2;
    double x;
    double y;
    double z;
    double w;

    void loginfo_throttle(const sensor_msgs::msg::Imu::SharedPtr msg){
        x = msg->orientation.x;
        y = msg->orientation.y;
        z = msg->orientation.z;
        w = msg->orientation.w;
        double roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
        double pitch = asin(2*(w*y - z*x));
        double yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*y));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "\n x:%.2f"
            "\n y:%.2f"
            "\n z:%.2f"
            "\n w:%.2f"
            "\n roll:%.2f"
            "\n pitch:%.2f"
            "\n yaw:%.2f"
            ,
            x,y,z,w,roll,pitch,yaw
        );

        
    }
    void quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat, 
                             double& roll, double& pitch, double& yaw)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);  // 直接初始化 在ros中推荐使用方式
        // tf2::Quaternion q = tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
        // auto q = tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);

        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);  // 单位：弧度
    }

    void imu_subs_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message");
        loginfo_throttle(msg);
        quaternion_to_euler(msg ->orientation ,roll2, pitch2, yaw2);
       
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "\n ------------"
            "\n roll:%.2f"
            "\n pitch:%.2f"
            "\n yaw:%.2f"
            ,
            roll2,pitch2,yaw2
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
}
