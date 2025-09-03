/*
订阅激光雷达话题 /scan 查看障碍物到robot的距离

仿真：
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

$ ros2 run local_turtlebot3_test turtlebot3sensor0 

$ ros2 interface proto sensor_msgs/msg/LaserScan
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
angle_min: 0.0          扫描的起始角度
angle_max: 0.0          扫描的结束角度
angle_increment: 0.0    相邻两个激光点之间的角度差（单位：弧度）
time_increment: 0.0     相邻两个激光点的时间间隔（单位：秒）
scan_time: 0.0          完成一次完整扫描的时间（单位：秒）
range_min: 0.0          激光雷达能检测到的最小距离（单位：米）
range_max: 0.0          激光雷达能检测到的最大距离（单位：米）
ranges: []              浮点型数组，存储每个角度对应的距离测量值（单位：米）
intensities: []         浮点型数组，存储每个激光点的反射强度

*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
class SensorSub : public rclcpp::Node
{
public:
    SensorSub() : Node("sensor_sub_node")
    {
        subscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::QoS(10),
            std::bind(&SensorSub::subscan_callback , this , std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscan_;
    void subscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr lasescan_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received  lasescan_msg message");
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->angle_max: %.2f",lasescan_msg->angle_max);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->angle_min: %.2f",lasescan_msg->angle_min);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->angle_increment: %.2f",lasescan_msg->angle_increment);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->time_increment: %.2f",lasescan_msg->time_increment);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->scan_time: %.2f",lasescan_msg->scan_time);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->range_min: %.2f",lasescan_msg->range_min);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->range_max: %.2f",lasescan_msg->range_max);
        RCLCPP_INFO_ONCE(this->get_logger(), "lasescan_msg->ranges.size(): %zu",lasescan_msg->ranges.size());

        // a. 数组
        const std::vector<int> deg_list = {0, 45, 90, 135, 180, 225, 270, 315, 360};
        float front_distance;
        for (size_t i = 0; i < deg_list.size(); i++)
        {
            front_distance = lasescan_msg->ranges[deg_list[i]];
            RCLCPP_INFO(this->get_logger(), "range[%d]front_distance: %.2f" ,deg_list[i],front_distance);
        }
        
        // b. 单值
        // float front_distance = lasescan_msg->ranges[90];
        // RCLCPP_INFO(this->get_logger(), "front_distance: %.2f" ,front_distance);

        //过滤无效值
        if(std::isnan(front_distance) || front_distance > lasescan_msg->range_max){
            RCLCPP_INFO(this->get_logger(), "前方无障碍物");
        }else{
            RCLCPP_INFO(this->get_logger(), "前方距离: %.2f米", front_distance);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorSub>());
    rclcpp::shutdown();
    return 0;
}

/*

| 类型（典型变量）                           | printf 格式符            | 示例代码                                             |
| ---------------------------------- | --------------------- | ------------------------------------------------ |
| **int / std::int32\_t / int64\_t** | `%d` / `%ld` / `%lld` | `int a=5; printf("%d",a);`                       |
| **unsigned / std::size\_t**        | `%u` / `%zu`          | `size_t n=360; printf("%zu",n);`                 |
| **float**                          | `%f` 或 `%.2f`         | `float f=3.14f; printf("%.2f",f);`               |
| **double**                         | `%f` 或 `%.3lf`        | `double d=3.1415; printf("%.3f",d);`             |
| **char / char**\*                  | `%c` / `%s`           | `char c='A'; printf("%c",c);`                    |
| **std::string**                    | 先转 `c_str()` 再用 `%s`  | `std::string s="hello"; printf("%s",s.c_str());` |
| **指针地址**                         | `%p`                    | `void* p=&x; printf("%p",p);`                    |


*/
