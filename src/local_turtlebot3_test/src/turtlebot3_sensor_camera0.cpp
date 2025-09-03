#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "cv_bridge/cv_bridge.h"  // 转换ROS图像与OpenCV图像
#include "opencv2/opencv.hpp"     // 图像处理
#include <chrono>

using namespace std::chrono_literals;
using namespace message_filters;

class CameraProcessor : public rclcpp::Node
{
public:
    CameraProcessor() : Node("camera_processor")
    {
        // 订阅原始图像和相机内参（时间同步）
        image_sub_.subscribe(this, "/camera/image_raw");
        info_sub_.subscribe(this, "/camera/camera_info");

        // 时间同步器：近似时间同步（允许微小时间差）
        sync_ = std::make_shared<SyncPolicy>(10);  // 队列大小10
        sync_->connectInput(image_sub_, info_sub_);
        sync_->registerCallback(&CameraProcessor::sync_callback, this);

        // （可选）订阅压缩图像（如需处理压缩数据）
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed", 10,
            std::bind(&CameraProcessor::compressed_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "相机处理器节点启动，开始接收图像数据...");
    }

private:
    // 原始图像和相机内参订阅器
    Subscriber<sensor_msgs::msg::Image> image_sub_;
    Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    // 压缩图像订阅器（可选）
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;

    // 时间同步器类型（匹配Image和CameraInfo）
    using SyncPolicy = ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<Synchronizer<SyncPolicy>> sync_;

    // 图像保存计数器
    int save_count_ = 0;

    // 同步回调：处理原始图像+相机内参
    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        // 1. 解析相机内参（仅首次打印）
        static bool info_printed = false;
        if (!info_printed) {
            RCLCPP_INFO(this->get_logger(), "\n相机内参信息：");
            RCLCPP_INFO(this->get_logger(), "图像尺寸：%dx%d", 
                info_msg->width, info_msg->height);
            RCLCPP_INFO(this->get_logger(), "焦距(fx, fy)：%.2f, %.2f",
                info_msg->k[0], info_msg->k[4]);  // k是相机矩阵[fx,0,cx; 0,fy,cy; 0,0,1]
            RCLCPP_INFO(this->get_logger(), "主点(cx, cy)：%.2f, %.2f",
                info_msg->k[2], info_msg->k[5]);
            info_printed = true;
        }

        // 2. 将ROS图像转换为OpenCV格式（BGR8）
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换失败：%s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;

        // 3. 简单图像处理：边缘检测
        cv::Mat gray, edges;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);       // 转灰度图
        cv::GaussianBlur(gray, gray, cv::Size(7, 7), 1.5);  // 高斯模糊去噪
        cv::Canny(gray, edges, 50, 150);                    // Canny边缘检测

        // 4. 实时显示图像（原始图+边缘检测图）
        cv::imshow("原始图像", frame);
        cv::imshow("边缘检测", edges);
        cv::waitKey(1);  // 必须添加，否则窗口无响应

        // 5. 按's'键保存图像（每2秒最多保存1张，避免刷屏）
        static auto last_save = this->now();
        if (cv::waitKey(1) == 's' && (this->now() - last_save) > 2s) {
            std::string img_path = "turtlebot3_camera_" + std::to_string(save_count_) + ".jpg";
            cv::imwrite(img_path, frame);
            RCLCPP_INFO(this->get_logger(), "图像已保存：%s", img_path.c_str());
            save_count_++;
            last_save = this->now();
        }
    }

    // （可选）压缩图像回调（处理JPEG压缩数据）
    void compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& comp_msg)
    {
        // 转换压缩图像为OpenCV格式
        try {
            cv::Mat frame = cv::imdecode(cv::Mat(comp_msg->data), cv::IMREAD_COLOR);
            if (!frame.empty()) {
                // 此处可添加压缩图像处理逻辑（如缩小尺寸后传输）
                // cv::imshow("压缩图像", frame);
                // cv::waitKey(1);
            }
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "压缩图像解码失败：%s", e.what());
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraProcessor>());
    rclcpp::shutdown();
    cv::destroyAllWindows();  // 关闭OpenCV窗口
    return 0;
}
