#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2：：Quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //
#include "tf2_ros/static_transform_broadcaster.h"   // 静态坐标广播类


class StaticTFBroadcaster : public rclcpp::Node
{
public:
    StaticTFBroadcaster() : Node("static_tf_broadcaster")
    {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_tf();
    }

    void publish_tf(){
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "target_point";
        transform.transform.translation.x = 5.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 60 * M_PI / 180);
        transform.transform.rotation = tf2::toMsg(q);
        this->static_tf_broadcaster_->sendTransform(transform);

    }
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
