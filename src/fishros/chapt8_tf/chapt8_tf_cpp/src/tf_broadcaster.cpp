#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2：：Quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //
#include "tf2_ros/transform_broadcaster.h"          //
#include "chrono"
using namespace std::chrono_literals;               // s ms 表示时间
/*
    tf 动态发布
*/

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster() : Node("dync_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 动态发布
        timer_ = this->create_wall_timer(100ms,std::bind(&TFBroadcaster::publish_tf,this));
    }

    void publish_tf(){
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 30 * M_PI / 180);
        transform.transform.rotation = tf2::toMsg(q);
        this->tf_broadcaster_->sendTransform(transform);

    }
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
