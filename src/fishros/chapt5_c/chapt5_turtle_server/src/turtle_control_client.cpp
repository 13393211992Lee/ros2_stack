#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "chapt5_interface/srv/patrol.hpp"
#include "chrono"
#include "ctime"

using Patrol = chapt5_interface::srv::Patrol;
using namespace std::chrono_literals;

class TurtleControlClient : public rclcpp::Node
{
public:
    TurtleControlClient() : Node("turtle_control_lient")
    {
        srand(time(NULL));
        patrol_client_ = this->create_client<Patrol>("patrol_server");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&TurtleControlClient::timer_cb, this));
    }

private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    void patrol_client_response_callback_name_(rclcpp::Client<Patrol>::SharedFuture result_future){
        auto response = result_future.get();
        if(response->result == Patrol::Response::SUCCESS){
            RCLCPP_INFO(this->get_logger(), "请求巡逻点成功");
        }
        if(response->result == Patrol::Response::FAIL){
            RCLCPP_INFO(this->get_logger(), "请求巡逻点失败");
        }
    }
    void timer_cb()
    {
        // 1. 检测服务是否启动
        while (!this->patrol_client_->wait_for_service(1s))
        {
            if(!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(), "等待服务器上限过程中...rclcpp shutdown...");
            }
        RCLCPP_INFO(this->get_logger(), "等待服务器启动");
        }
        
        //2. 构造请求对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 10;
        request->target_y = rand() % 10;
        RCLCPP_INFO(this->get_logger(), "准备好target %f, %f",request->target_x,request->target_y );

        //3. 发送请求
        this->patrol_client_->async_send_request(
            request, std::bind(&TurtleControlClient::patrol_client_response_callback_name_, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControlClient>());
    rclcpp::shutdown();
    return 0;
}
