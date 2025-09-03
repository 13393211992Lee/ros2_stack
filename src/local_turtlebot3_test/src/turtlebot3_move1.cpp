#include <memory>
/*
启动：
$ ros2 run local_turtlebot3_test turtlebot31
[INFO] [1754292897.369818046] [turtlebot3_own1]: Turtlebot3控制器节点已启动
[INFO] [1754292897.369963209] [turtlebot3_own1]: 线性速度: 0.30 m/s, 角速度: 0.20 rad/s

订阅：
    $ ros2 topic echo /cmd_vel
查看：  
$ ros2 param  dump /turtlebot3_own1

# 对比速度信息
$ ros2 service call /control_movement std_srvs/srv/SetBool "{'data':true}"
    requester: making request: std_srvs.srv.SetBool_Request(data=True)
    response: std_srvs.srv.SetBool_Response(success=True, message='机器人开始移动')

$ ros2 service call /control_movement std_srvs/srv/SetBool "{'data':false}"
    requester: making request: std_srvs.srv.SetBool_Request(data=False)
    response:std_srvs.srv.SetBool_Response(success=True, message='机器人已停止')

*/

// < >  , m   ->
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"


class turtlebot3_own1 : public rclcpp::Node{
public:
    // 构造函数，初始化节点和参数
    turtlebot3_own1() : Node("turtlebot3_own1"){
        // 声明参数并设置默认值
        declare_parameters();

        // 获取参数值
        get_parameters();

        // 创建速度发布者 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", rclcpp::QoS(10));

        // 创建定时器，控制发布频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0/loop_rate_)),
            std::bind(&turtlebot3_own1::timers_callback, this)
        );

        // 创建控制服务
        control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/control_movement",
            std::bind(&turtlebot3_own1::control_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Turtlebot3控制器节点已启动");
        RCLCPP_INFO(this->get_logger(), "线性速度: %.2f m/s, 角速度: %.2f rad/s",
            linear_speed_, angular_speed_);
    }

private:
    bool is_moving_ = true;
    double linear_speed_;
    double angular_speed_;
    double loop_rate_ ;

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 声明所有参数
    void declare_parameters(){
        this->declare_parameter<double>("linear_speed", 0.3);
        this->declare_parameter<double>("angular_speed", 0.2 );
        this->declare_parameter<double>("loop_rate", 2.0);    // Hz
        this->declare_parameter<bool>("is_moving", true );
    }

    // 获取所有参数
    void get_parameters(){
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        this->get_parameter("is_moving", is_moving_);
        this->get_parameter("loop_rate",loop_rate_);
        // 参数校验
        if (loop_rate_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "无效的循环频率，使用默认值 2.0 Hz");
            loop_rate_ = 2.0;
        }
    }

    // Service回调函数：处理启动/停止请求
    void control_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response){
            is_moving_ = request->data;
            if(is_moving_){
                RCLCPP_INFO(this->get_logger(), "开始移动");
                response->success = true;
                response->message = "机器人开始移动";               
            }else{
                RCLCPP_INFO(this->get_logger(), "停止移动");
                response->success = true;
                response->message = "机器人已停止";
            }
        
    }
    // 定时器回调：周期性发布速度指令
    void timers_callback(){
        auto back_sg = msgCallback();
        cmd_vel_pub_->publish(back_sg);
    }

    geometry_msgs::msg::TwistStamped msgCallback(){
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";
        if(is_moving_){
            twist_msg.twist.angular.z = angular_speed_;
            twist_msg.twist.linear.x = linear_speed_;
        }else{
            twist_msg.twist.angular.z = 0.0;
            twist_msg.twist.linear.x = 0.0;  
        }
       
        return twist_msg;
    }


};
 


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtlebot3_own1>());
    rclcpp::shutdown();
    return 0;
}

