/*
需求： 订阅原始图像 + 相机内参、同步数据
重点 “ 同步数据 message_filters”
*/
/*
rclcpp::Subscription & message_filters::Subscriber 的区别之处

rclcpp::Subscription：  处理单话题
message_filters::Subscriber： 多话题协同处理

*/

#include "rclcpp/rclcpp.hpp"
#include <cstdlib>  // 引入llabs函数的头文件
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace cammer
{
class CameraProcessor : public rclcpp::Node
{
public:
    CameraProcessor() : Node("camera_processor_node")
    {
        // 创建两个消息过滤器订阅者
        image_sub_.subscribe(this, "/camera/image_raw");
        info_sub_.subscribe(this, "/camera/camera_info");
        
        // 定义近似时间同步策略，允许一定的时间差
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, 
                                                               sensor_msgs::msg::CameraInfo> MySyncPolicy;
        
        // 创建同步器，队列大小为10，使用定义的同步策略
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), 
                                                                              image_sub_, 
                                                                              info_sub_);
        
        // 注册回调函数，当两个话题的消息同步后调用
        sync_->registerCallback(std::bind(&CameraProcessor::sync_callback, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Camera processor node initialized with time synchronization");
    }

private:
    // 消息过滤器订阅者
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    
    // 同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, 
                                                           sensor_msgs::msg::CameraInfo> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    
    // 同步回调函数，同时处理图像和相机信息
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "收到同步的图像和相机内参消息");
        
        // 打印图像信息
        loginfo_image_raw(image_msg);
        
        // 打印相机内参信息
        loginfo_camera_info(info_msg);
        
        // 检查时间戳差异
        int64_t time_diff = llabs((image_msg->header.stamp.sec - info_msg->header.stamp.sec) * 1000000000LL +
                               (image_msg->header.stamp.nanosec - info_msg->header.stamp.nanosec));
                               
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "同步消息时间差: %ld 纳秒--------------", time_diff);
    }

    void loginfo_image_raw(const sensor_msgs::msg::Image::ConstSharedPtr msg){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "/camera/image_raw 数据:"
        "\n frame_id:%s"
        "\n Time:%d s,%09u ns"
        "\n height:%d ,width:%d "
        "\n encoding: %s"
        "\n is_bigendian: %d"
        "\n step: %d"
        "\n ",
        msg->header.frame_id.c_str() ,
        msg->header.stamp.sec ,
        msg->header.stamp.nanosec,
        msg->height ,
        msg->width ,
        msg->encoding.c_str() ,
        msg->is_bigendian ,
        msg->step
        );
    }
    
    void loginfo_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "/camera/camera_info 数据:"
        "\n frame_id:%s"
        "\n Time:%d s,%09u ns"
        "\n 图像宽度: %d, 高度: %d"
        "\n 相机矩阵 K: [%.2f, %.2f, %.2f; %.2f, %.2f, %.2f; %.2f, %.2f, %.2f]"
        "\n 畸变系数 D: [%.4f, %.4f, %.4f, %.4f, %.4f]"
        "\n 投影矩阵 P: [%.2f, %.2f, %.2f, %.2f; %.2f, %.2f, %.2f, %.2f; %.2f, %.2f, %.2f, %.2f]"
        "\n ",
        msg->header.frame_id.c_str(),
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        msg->width, msg->height,
        // 相机矩阵 K
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8],
        // 畸变系数 D
        msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4],
        // 投影矩阵 P
        msg->p[0], msg->p[1], msg->p[2], msg->p[3],
        msg->p[4], msg->p[5], msg->p[6], msg->p[7],
        msg->p[8], msg->p[9], msg->p[10], msg->p[11]
        );
    }
};
}  //end namespace

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cammer::CameraProcessor>());
    rclcpp::shutdown();
    return 0;
}
    