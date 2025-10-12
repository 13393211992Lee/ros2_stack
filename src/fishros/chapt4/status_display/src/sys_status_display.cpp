#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interface/msg/system_status.hpp>
#include <iostream>
#include <memory>

using SystemStatus = status_interface::msg::SystemStatus;


namespace sys_status
{
class SysStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr sub_;
    // QLabel* label_;
    std::unique_ptr<QLabel> label_;
    void cb(const SystemStatus::SharedPtr msg){
        // msg 拆分成 Qstring
        label_->setText(get_qstr_from_msg(msg));
        label_->show();
    }
public:
    SysStatusDisplay(const std::string &node_name) : Node(node_name){
        // label_ = new QLabel();
        label_ = std::make_unique<QLabel>();
        sub_ = this->create_subscription<SystemStatus>("sys_status",10,
            std::bind(&SysStatusDisplay::cb,this,std::placeholders::_1));
    }

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg ){
        std::stringstream show_str;
        show_str<<"-----提供状态可视化显示工具----- \n"
        <<"数据时间：" <<msg->stamp.sec<<" s\n"
        <<"主机名称：" <<msg->hostname<<" \n"
        <<"cpu使用率：" <<msg->cpu_percent<<" %\n"
        <<"内存总大小：" <<msg->memory_total<<" MB\n"
        <<"内存使用率：" <<msg->memory_percent<<"%\n"
        <<"剩余内存：" <<msg->memory_available<<" MB\n"
        <<"网络发送量：" <<msg->net_sent<<" MB\n"
        <<"网络接收量：" <<msg->net_recv<<" MB\n"
        <<"-----------------------";
        return QString::fromStdString(show_str.str());
    }
};
    void spin_thread_func(const std::shared_ptr<sys_status::SysStatusDisplay> &node){
        rclcpp::spin(node);
    }
}  // namespace sys_status



int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sys_status::SysStatusDisplay>("sys_status_sub_node");

    // 两种表达方式：
    // method: lambda
    // std::thread spin_thread([&]() -> void {rclcpp::spin(node);});
    // method2
    std::thread spin_thread(std::bind(&sys_status::spin_thread_func,node));

    spin_thread.detach();
    app.exec(); //执行应用 阻塞代码
    rclcpp::shutdown();
    return 0;
}


/*
1.线程
2.lambda
3.指针 变 智能指针 


*/