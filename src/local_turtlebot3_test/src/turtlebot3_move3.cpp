// 1. 键盘 / 终端指令控制
// 通过键盘输入（如w/s/a/d）或终端输入指令，实时生成速度控制消息并发布，实现机器人的前进、后退、左右转向等运动。
// < >  , m   ->
/**
 * 通过键盘输入控制机器人运动的功能
 * 启动
 * $ ros2 run local_turtlebot3_test turtlebot3m3 

 * turtlesi 映射话题
    $ ros2 run turtlesim turtlesim_node --ros-args --remap /turtle1/cmd_vel:=/cmd_vel
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("keyboard_controller")
    {
        // 创建发布者，发布Twist消息到cmd_vel话题
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 设置循环频率为10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardController::timer_callback, this));
        
        // 显示操作说明
        print_usage();
    }

private:
    // 定时器回调函数，用于发布速度消息
    void timer_callback()
    {
        int key = get_key();
        if (key != -1)
        {
            handle_key(key);
        }
        
        // 发布速度消息
        publisher_->publish(twist_msg_);
    }

    // 处理按键输入
    void handle_key(int key)
    {
        // 默认速度值
        const double linear_speed = 0.5;  // 线速度(m/s)
        const double angular_speed = 1.0; // 角速度(rad/s)
        
        // 停止所有运动
        if (key == 'x' || key == 'X')
        {
            twist_msg_.linear.x = 0.0;
            twist_msg_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "停止运动");
        }
        // 前进
        else if (key == 'w' || key == 'W')
        {
            twist_msg_.linear.x = linear_speed;
            twist_msg_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "前进");
        }
        // 后退
        else if (key == 's' || key == 'S')
        {
            twist_msg_.linear.x = -linear_speed;
            twist_msg_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "后退");
        }
        // 左转
        else if (key == 'a' || key == 'A')
        {
            twist_msg_.linear.x = 0.0;
            twist_msg_.angular.z = angular_speed;
            RCLCPP_INFO(this->get_logger(), "左转");
        }
        // 右转
        else if (key == 'd' || key == 'D')
        {
            twist_msg_.linear.x = 0.0;
            twist_msg_.angular.z = -angular_speed;
            RCLCPP_INFO(this->get_logger(), "右转");
        }
        // 打印帮助信息
        else if (key == 'h' || key == 'H')
        {
            print_usage();
        }
    }

    // 打印使用说明
    void print_usage()
    {
        RCLCPP_INFO(this->get_logger(), "键盘控制机器人运动:");
        RCLCPP_INFO(this->get_logger(), "------------------------");
        RCLCPP_INFO(this->get_logger(), "w/W: 前进");
        RCLCPP_INFO(this->get_logger(), "s/S: 后退");
        RCLCPP_INFO(this->get_logger(), "a/A: 左转");
        RCLCPP_INFO(this->get_logger(), "d/D: 右转");
        RCLCPP_INFO(this->get_logger(), "x/X: 停止");
        RCLCPP_INFO(this->get_logger(), "h/H: 显示帮助");
        RCLCPP_INFO(this->get_logger(), "Ctrl+C: 退出程序");
    }

    // 非阻塞获取键盘输入
    int get_key()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;

        // 获取终端设置
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        // 禁用规范模式和回显
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        // 设置非阻塞模式
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if(ch != EOF)
        {
            ungetc(ch, stdin);
            return getchar();
        }

        return -1;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardController>());
    rclcpp::shutdown();
    return 0;
}



