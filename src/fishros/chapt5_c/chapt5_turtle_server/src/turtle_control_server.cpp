#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "cmath"
#include "geometry_msgs/msg/twist.hpp"
#include "chapt5_interface/srv/patrol.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

/**
 * @brief  
 * /turtle1/pose [turtlesim/msg/Pose]
 * /turtle1/cmd_vel [geometry_msgs/msg/Twist]
 * 1. 订阅当前乌龟的位置
 * 2. 计算线性速度
 * 3. 发布
 */

using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
using Pose = turtlesim::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
using Patrol = chapt5_interface::srv::Patrol;
class TurtleControlServer : public rclcpp::Node
{
public:
    TurtleControlServer() : Node("turtle_control_node"),has_reached_goal_(false)
    {
        //1.参数声明
        TurtleControlServer::declare_his_param();
        using std::placeholders::_1;
        using std::placeholders::_2;
        
        //2.订阅乌龟位置信息
        sub_turtle_pose_ = this->create_subscription<Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleControlServer::sub_turtle_cb, this, _1));
            
        // 参数更新后 额外调用我们添加的回调函数，可以做到参数实时更新
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [&](const std::vector<rclcpp::Parameter> & parameters)
            ->rcl_interfaces::msg::SetParametersResult{
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & parameter : parameters) {
                    RCLCPP_INFO(this->get_logger(), "更新参数的值%s=%f",parameter.get_name().c_str(),
                        parameter.as_double());
                    if(parameter.get_name() == "goal_x"){
                        goal_x = parameter.as_double();
                    }
                    else if(parameter.get_name() == "goal_y"){
                        goal_y = parameter.as_double();
                    }
                    else if(parameter.get_name() == "k_linear"){
                        k_linear = parameter.as_double();
                    }
                    else if(parameter.get_name() == "k_angular"){
                        k_angular = parameter.as_double();
                    }
                }
            return result;
        });
        //3. 接收客户端发送的goal信息
        patrol_server_ = this->create_service<Patrol>(
            "patrol_server", std::bind(&TurtleControlServer::patrol_server_cb, this, _1, _2));

        //4. 计算并发布速度信息
        pub_turtle_twist_= this->create_publisher<Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleControlServer::pub_turtle_twist, this));

        //5.
        RCLCPP_INFO(this->get_logger(), "服务器启动...");
    }
     // 提供一个接口供主线程判断是否到达目标点
    bool has_reached_goal() const {
        return has_reached_goal_;
    }
    void declare_his_param(){
        this->declare_parameter<double>("goal_x", 3.0);
        this->declare_parameter<double>("goal_y", 3.0);
        this->declare_parameter<double>("k_linear", 0.9);
        this->declare_parameter<double>("k_angular", 0.6);

        // 获取参数时增加类型检查
        rclcpp::Parameter param_x = this->get_parameter("goal_x");
        rclcpp::Parameter param_y= this->get_parameter("goal_y");
        rclcpp::Parameter param_k_linear = this->get_parameter("k_linear");
        rclcpp::Parameter param_k_angular= this->get_parameter("k_angular");
        if(param_x.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
            goal_x = param_x.get_value<double>();
        } else {
            RCLCPP_ERROR(this->get_logger(), "goal_x 类型错误，使用默认值 3.0");
            goal_x = 3.0; // 默认值
        }
        if(param_y.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
            goal_y = param_y.get_value<double>();
        } else {
            RCLCPP_ERROR(this->get_logger(), "goal_y 类型错误，使用默认值 3.0");
            goal_y = 3.0; // 默认值
        }
        if(param_k_linear.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
            k_linear = param_k_linear.get_value<double>();
        } else {
            RCLCPP_ERROR(this->get_logger(), "k_linear 类型错误，使用默认值 0.9");
            k_linear = 0.9; // 默认值
        }
        if(param_k_angular.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
            k_angular = param_k_angular.get_value<double>();
        } else {
            RCLCPP_ERROR(this->get_logger(), "k_angular 类型错误，使用默认值 0.6");
            k_angular = 0.6; // 默认值
        }
    }
private:

    double goal_x , goal_y;
    double k_linear, k_angular;// 控制线速度

    // 当前位置
    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;
    bool has_reached_goal_;  // 到达目标点的标志位

    rclcpp::Publisher<Twist>::SharedPtr pub_turtle_twist_;
    rclcpp::Subscription<Pose>::SharedPtr sub_turtle_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<Patrol>::SharedPtr patrol_server_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    void pub_turtle_twist()
    {
        if (has_reached_goal_) {
            return;  // 已到达目标，不再发布指令
        }else{
            // RCLCPP_INFO(this->get_logger(), "calculate_twist & publish");
            auto twist = calculate_twist();
            pub_turtle_twist_->publish(twist);
        }
    }

    bool patrol_server_cb(
        const std::shared_ptr<Patrol::Request> request,
        std::shared_ptr<Patrol::Response> response)
    {


        if (request->target_x >0 && request->target_x <10 && 
            request->target_y >0 && request->target_y <10 )
        {
            has_reached_goal_ = false;
            goal_x = request->target_x ;
            goal_y = request->target_y ;
            RCLCPP_INFO(this->get_logger(), "Received request x:%.2f,y:%.2f",goal_x, goal_y);
            response->result = Patrol::Response::SUCCESS; // 设置成功状态
        }else{
            RCLCPP_INFO(this->get_logger(), "参数不符合 x,y(0,10)");
            response->result = Patrol::Response::FAIL; // 设置成功状态
        }
        return true;

    }
    Twist calculate_twist(){
        Twist twist;
        float distance_x = goal_x - current_x;
        float distance_y = goal_y - current_y;

        // 计算两点距离
        float distance_2point = std::sqrt(distance_x * distance_x + distance_y * distance_y);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "两点距离:%2.f",distance_2point);
        
        // 如果距离目标点足够近，则停止移动
        if(distance_2point < 0.1){
            twist.linear.x=0.0;
            twist.angular.z = 0.0;
            RCLCPP_INFO_ONCE(this->get_logger(), "已到达目标点!");
            // has_reached_goal_ = true;  // 设置标志位
            return twist;
        }

        // 计算目标方向角度
        double target_angle = atan2(distance_y , distance_x);
        double angle_diff = target_angle- current_theta;
        // 角度差 归一到 [-pi pi]
        if (angle_diff > M_PI) {
            angle_diff -= 2 * M_PI;
        } else if (angle_diff < -M_PI) {
            angle_diff += 2 * M_PI;
        }

        // 计算速度指令 1  先运动 后转向
        // if (distance_2point > 0.5) {
        //     twist.angular.z = k_angular * angle_diff; // 调整方向 比例控制角速度
        //     twist.linear.x = distance_2point * k_linear;
            
        //     if (twist.linear.x > 1.0) {// 限制最大线速度
        //         twist.linear.x = 1.0;
        //     }
        //     } else {
        //         twist.linear.x = 0.0;
        //         twist.angular.z = 0.0;
                
        //         //与目标点的剩余距离<0.5m,结束导航
        //         // RCLCPP_INFO(this->get_logger(), "已经导航至目标点附近。");
        //     }
        
        // 计算速度指令
        twist.angular.z = k_angular * angle_diff;
        // 当角度偏差较小时才前进，避免在转向时前进
        if (fabs(angle_diff) < 0.9) {  // 约17度的阈值
            twist.linear.x = k_linear * distance_2point;
            // 限制最大线速度
            if (twist.linear.x > 1.5) {
                twist.linear.x = 1.5;
            }
        } else {
            twist.linear.x = 0.0;  // 角度偏差大时只转向不前进
        }
        
        return twist;
    }
    void sub_turtle_cb(const Pose::SharedPtr msg)
    {
        // 更新当前位置信息
        current_x = msg->x;
        current_y = msg->y;
        current_theta = msg->theta;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlServer>();
    while (rclcpp::ok() && !node->has_reached_goal())
    {
        rclcpp::spin_some(node);  // 处理一次回调事件
    }
    rclcpp::shutdown();
    return 0;
}

/*
$ ros2 run chapt5_turtle_server turtle_control  --ros-args --param goal_x:=9.0
$ ros2 run chapt5_turtle_server  turtle_control --ros-args --param goal_x:=9.0 --param goal_y:=9.0
$ ros2 run chapt5_turtle_server  turtle_control --ros-args --param goal_x:=1.0 --param goal_y:=1.0 --param k_linear:=0.3 --param k_angular:=0.3

$ ros2 service call /patrol_server chapt5_interface/srv/Patrol '{target_x: 9, target_y: 9}'

 */