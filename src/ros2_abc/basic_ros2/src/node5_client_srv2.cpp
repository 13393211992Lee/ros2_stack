#include "rclcpp/rclcpp.hpp"
#include "basic_interfaces/srv/add_three_ints.hpp"                                       // CHANGE
#include <cstdlib>

using std::placeholders::_1;

class ClientSrv : public rclcpp::Node
{
public:
    ClientSrv(int argc, char * argv[]) : Node("client_srv")
    {
        auto request = std::make_shared<basic_interfaces::srv::AddThreeInts::Request>();
        //解析数据
        parse_arguments(argc,argv,request);
        // 创建client
        client_ = this->create_client<basic_interfaces::srv::AddThreeInts>("add_three_ints");

        // 等待服务器启动
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务时被中断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server 当前不可用继续等待...");
        }

        // 发送请求
        client_->async_send_request(
            request, std::bind(&ClientSrv::client_cb, this, _1));
    }

private:
    long int a_ = 0;  // 存储第一个加数
    long int b_ = 0;  // 存储第二个加数
    long int c_ = 0;  // 存储第3个加数
    rclcpp::Client<basic_interfaces::srv::AddThreeInts>::SharedPtr client_;
    //接受返回数据
    using ServiceResponseFuture = rclcpp::Client<basic_interfaces::srv::AddThreeInts>::SharedFuture;
    void client_cb(ServiceResponseFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "返回的数据为：sum= %.ld",response->sum);
    }

    void parse_arguments(int argc, char * argv[],
        std::shared_ptr<basic_interfaces::srv::AddThreeInts::Request> request)
    {
        if (argc == 4) {
            a_ = std::stol(argv[1]);  // 从第一个参数获取a的值
            b_ = std::stol(argv[2]);  // 从第二个参数获取b的值
            c_ = std::stol(argv[3]);  // 从第3个参数获取b的值

            // 使用从命令行获取的参数
            request->a = a_;
            request->b = b_;
            request->c = c_;
            RCLCPP_INFO(this->get_logger(), "发送请求:a=%ld b=%ld c=%ld",a_,b_,c_);

        }else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client a b c");
            std::exit(1);   // 退出程序，返回非0值表示异常退出
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientSrv>(argc,argv));
    rclcpp::shutdown();
    return 0;
}
