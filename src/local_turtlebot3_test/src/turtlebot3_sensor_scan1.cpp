#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber() : Node("laser_scan_subscriber")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS(),
            std::bind(&LaserScanSubscriber::scan_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // 将角度（度）转换为弧度
    double deg_to_rad(double deg) const {
        return deg * M_PI / 180.0;
    }

    // 根据目标角度（度）计算对应的ranges索引
    int get_index_by_angle(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double target_deg) const {
        // 1. 将目标角度转换为弧度
        double target_rad = deg_to_rad(target_deg);

        // 2. 检查角度是否在雷达扫描范围内
        if (target_rad < scan_msg->angle_min || target_rad > scan_msg->angle_max) {
            RCLCPP_WARN(this->get_logger(), "角度 %.1f° 超出雷达扫描范围 [%.1f°, %.1f°]",
                      target_deg,
                      scan_msg->angle_min * 180/M_PI,
                      scan_msg->angle_max * 180/M_PI);
            return -1;
        }

        // 3. 计算索引：(目标角度 - 起始角度) / 角度增量
        int index = static_cast<int>((target_rad - scan_msg->angle_min) / scan_msg->angle_increment);

        // 4. 检查索引是否越界
        if (index < 0 || static_cast<size_t>(index) >= scan_msg->ranges.size()) {
            RCLCPP_WARN(this->get_logger(), "计算出的索引 %d 越界（有效范围 0~%zu）",
                      index, scan_msg->ranges.size()-1);
            return -1;
        }

        return index;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // 只打印一次雷达参数（方便调试角度范围）
        static bool params_printed = false;
        if (!params_printed) {
            RCLCPP_INFO(this->get_logger(), "雷达参数："
                      "\n  角度范围：%.1f° ~ %.1f°"
                      "\n  角度增量：%.3f°/点"
                      "\n  总点数：%zu",
                      scan_msg->angle_min * 180/M_PI,
                      scan_msg->angle_max * 180/M_PI,
                      scan_msg->angle_increment * 180/M_PI,
                      scan_msg->ranges.size());
            params_printed = true;
        }

        // 需要检测的方向（单位：度，这些是真正的角度，不是索引）
        const std::vector<int> target_angles_deg = {0, 45, 90, 135, 180, 225, 270, 315};

        for (int angle : target_angles_deg) {
            // 计算该角度对应的索引
            int index = get_index_by_angle(scan_msg, angle);
            if (index == -1) continue;

            // 获取距离值
            float distance = scan_msg->ranges[index];

            // 过滤无效值
            if (std::isnan(distance) || distance < scan_msg->range_min || distance > scan_msg->range_max) {
                RCLCPP_INFO(this->get_logger(), "角度 %d°（索引 %d）：无有效数据", angle, index);
            } else {
                RCLCPP_INFO(this->get_logger(), "角度 %d°（索引 %d）：距离 = %.2f 米", angle, index, distance);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSubscriber>());
    rclcpp::shutdown();
    return 0;
}
