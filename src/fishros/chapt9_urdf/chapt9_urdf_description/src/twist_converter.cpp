#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <memory>
#include <string>

class SimpleTwistConverter : public rclcpp::Node
{
public:
    SimpleTwistConverter() : Node("simple_twist_converter")
    {
        // 设置参数
        input_topic_ = "/cmd_vel";
        output_topic_ = "/cmd_vel_twist_stamped";
        frame_id_ = "base_footprint";
        
        // 创建订阅者 - 订阅 Twist
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_topic_, 
            10,
            std::bind(&SimpleTwistConverter::twistCallback, this, std::placeholders::_1));
        
        // 创建发布者 - 发布 TwistStamped
        twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            output_topic_, 
            10);
        
        RCLCPP_INFO(this->get_logger(), 
                   "SimpleTwistConverter 已启动");
        RCLCPP_INFO(this->get_logger(), 
                   "订阅: %s (Twist)", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                   "发布: %s (TwistStamped)", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                   "坐标系: %s", frame_id_.c_str());
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 创建 TwistStamped 消息
        auto twist_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        
        // 设置时间戳（使用节点时钟）
        twist_stamped_msg->header.stamp = this->now();
        
        // 设置坐标系
        twist_stamped_msg->header.frame_id = frame_id_;
        
        // 复制 Twist 数据
        twist_stamped_msg->twist = *msg;
        
        // 发布到输出话题
        twist_stamped_pub_->publish(std::move(twist_stamped_msg));
        
        // 打印调试信息
        RCLCPP_DEBUG(this->get_logger(), 
                    "转换: vx=%.3f, wz=%.3f", 
                    msg->linear.x, 
                    msg->angular.z);
    }
    
    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTwistConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}