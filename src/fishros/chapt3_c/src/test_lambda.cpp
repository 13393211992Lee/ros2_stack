#include <algorithm> //lambda函数
#include <memory>       //智能指针
#include <iostream>     //std
int main()
{
    auto add = [](int a, int b) -> int {return a+b;};
    int sum = add(200,300);
    auto print_sum = [sum]()->void{
        std::cout<<sum<<std::endl;
    };
    print_sum();
    return 0;
}
