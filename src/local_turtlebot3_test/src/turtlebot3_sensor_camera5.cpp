
/*
被迫终止：
    原因：
        $ ros2 launch turtlebot3_gazebo empty_world.launch.py
        $ ros2 topic echo /camera/image_raw/compressedDepth
        gazebo 终端提示错误：
            [image_bridge-5] [ERROR] [1755673323.696608727] [compressed_depth_image_transport]: Compressed Depth Image Transport - Compression requires single-channel 32bit-floating  point or 16bit raw depth images (input format is: rgb8).


自主避障：
1. 订阅深度图像话题，通过cv_bridge将 ROS 图像消息转换为 OpenCV 的cv::Mat
2. 设定安全距离阈值（如 50cm），遍历深度图像像素，筛选出距离小于阈值的区域（判定为障碍物）
3. 通过camera_info计算障碍物在机器人坐标系中的实际位置；
4. 将障碍物位置信息发布到/obstacle_positions话题，结合 Nav2 导航栈规划绕行路径，实现动态避障
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

/camera/camera_info	                sensor_msgs/msg/CameraInfo	                相机内参（焦距、畸变系数、图像尺寸等）
/camera/image_raw	                sensor_msgs/msg/Image	                    原始未压缩图像（RGB 格式，数据量大）
/camera/image_raw/compressed	    sensor_msgs/msg/CompressedImage	JPEG        压缩图像（节省带宽，推荐远程传输）
/camera/image_raw/compressedDepth	sensor_msgs/msg/CompressedImage	            深度图像压缩格式（若相机支持深度）

*/


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
using std::placeholders::_1;

class ObstacleAvoider : public rclcpp::Node{
public:
    ObstacleAvoider() : Node("obstacle_avoider_node"){
        RCLCPP_INFO(this->get_logger(), "obstacle_avoider_node start");

    image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&ObstacleAvoider::image_raw_cb, this, _1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
    const float SAFETY_DISTANCE = 0.5f;
    void image_raw_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received message");
        tran_rosImg2CVmat(msg);
        // depth_callback(msg);
    }
    //通过cv_bridge将 ROS 图像消息转换为 OpenCV 的cv::Mat
    void tran_rosImg2CVmat(const sensor_msgs::msg::Image::SharedPtr msg){
        try{
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            cv::Mat pic = cv_ptr->image;
            cv::imshow("color",pic);
            cv::waitKey(1);

        }catch(cv::Exception& e){
            std::cerr << e.what() << '\n';
        }
    }

    //待定
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg){ 
        try{
            // 将ROS图像消息转换为OpenCV的cv::Mat
            // 注意编码格式必须匹配（32FC1对应浮点型深度数据）
            cv_bridge::CvImagePtr cv_32FC1 = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat cv_img_32FC1= cv_32FC1->image;

            // 遍历图像像素（简化示例：只检查中心区域）
            int center_x = cv_img_32FC1.cols / 2;
            int center_y = cv_img_32FC1.rows / 2;

            // 获取中心像素的距离值
            float distance = cv_img_32FC1.at<float>(center_y, center_x);


            // 判断距离是否有效且小于安全阈值
            if (distance > 0.01f && distance < SAFETY_DISTANCE) {
                RCLCPP_INFO(this->get_logger(), "障碍物距离: %.2f米，小于安全距离！", distance);
                // 执行避障逻辑（如转向）
            } else {
                RCLCPP_INFO(this->get_logger(), "前方安全，距离: %.2f米", distance);
                // 继续前进
            }
        }catch(cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换错误: %s", e.what());
        }
        
        



    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}
