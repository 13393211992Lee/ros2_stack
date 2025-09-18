
#include "rclcpp/rclcpp.hpp"

class Person :public rclcpp::Node
{
private:
    std::string na_;
    int age_;

public:
    Person(const std::string &node_na, const std::string &na ,const int &age ):
    Node(node_na){ //调用父类为i的构造函数
        this->na_ = na;
        this->age_ = age;
    }

    void eat(const std::string &food_na){
        RCLCPP_INFO(this->get_logger(), "我是%s,今年%d岁,爱吃%s",
        this->na_.c_str(),this->age_ ,food_na.c_str()
        );
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Person>("ci_node", "cici", 3);
    node->eat("berry");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
