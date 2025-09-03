/*
/camera/camera_info	                sensor_msgs/msg/CameraInfo	                相机内参（焦距、畸变系数、图像尺寸等）
/camera/image_raw	                sensor_msgs/msg/Image	                    原始未压缩图像（RGB 格式，数据量大）
/camera/image_raw/compressed	    sensor_msgs/msg/CompressedImage	JPEG        压缩图像（节省带宽，推荐远程传输）
/camera/image_raw/compressedDepth	sensor_msgs/msg/CompressedImage	            深度图像压缩格式（若相机支持深度）
*/

/*
/camera/camera_info
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/compressedDepth
/camera/image_raw/theora
/camera/image_raw/zstd
/clock
/cmd_vel
/imu
/joint_states
/odom
/parameter_events
/robot_description
/rosout
/scan
/tf
/tf_static

*/


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using std::placeholders::_1;
namespace cammer
{
class CameraProcessor : public rclcpp::Node
{
public:
    CameraProcessor() : Node("camera_processor_node")
    {
        image_raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&CameraProcessor::image_raw_sub_callback, this, _1));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;

    void camera_info_sub_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){

    }

    void loginfo_image_raw(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
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
    void image_raw_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message");
        loginfo_image_raw(msg);
        
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
