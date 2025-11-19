#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2：：Quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //
#include "tf2_ros/transform_listener.h"             // tf 监听
#include "tf2_ros/buffer.h"                         // tf 监听信息的接收
#include "tf2/utils.h"                              // 4元数 转 欧拉角
#include "chrono"
using namespace std::chrono_literals;               // s ms 表示时间
/*
    监听 tf 
*/

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    TFListener() : Node("tf_listener")
    {
        this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,this);
        // 动态发布
        timer_ = this->create_wall_timer(1s,std::bind(&TFListener::getTransform,this));
    }
    void getTransform(){
        // buffer 中查询坐标关系
        try
        {
            const auto tranform = tf_buffer_ ->lookupTransform(
                "base_link",
                "target_point",
                this->get_clock()->now(),
                rclcpp::Duration::from_seconds(1.0f)
            );
            auto translation = tranform.transform.translation;
            auto rotation = tranform.transform.rotation;
            double y,p,r;
            tf2::getEulerYPR(rotation,y,p,r);
            RCLCPP_INFO(this->get_logger(),"平移： %f,%f,%f",translation.x,translation.y,translation.z);
            RCLCPP_INFO(this->get_logger(), "旋转y,p,r：%f,%f,%f",y,p,r);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            RCLCPP_WARN(this->get_logger(), "%s :",e.what());
        }
        
    }


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFListener>());
    rclcpp::shutdown();
    return 0;
}
