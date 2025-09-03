/*
topic                                                       type
/camera/image_raw/compressed                    sensor_msgs/msg/CompressedImage
/camera/image_raw/compressedDepth               sensor_msgs/msg/CompressedImage

需求：解析 话题 /camera/image_raw/compressed &  /camera/image_raw/compressedDepth

*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
using std::placeholders::_1;

namespace Image
{
    class ImageRawClass : public rclcpp::Node
    {
    public:
        ImageRawClass() : Node("image_raw_node"){
        compressed_image_ptr_ =  this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed", 10, std::bind(&ImageRawClass::compressed_image_cb, this, _1));
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_ptr_ ;

        void compressed_image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
            RCLCPP_INFO_ONCE(this->get_logger(), "Received message from topic: /camera/image_raw/compressed");
            PrintCompressedImage(msg);
        }

        // 读取照片
        void PrintCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
            RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,"打印CompressedImage数据");
            try {
            // 直接将压缩图像转换为OpenCV格式（BGR8）
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (!image.empty()) {
                cv::imshow("Color Image", image);
                cv::waitKey(1);
            }
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "彩色图像解析错误: %s", e.what());
        }
        }
    };
}  // namespace Image

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Image::ImageRawClass>());
    rclcpp::shutdown();
    return 0;
}







