#include <algorithm> //lambda函数
#include <memory>       //智能指针
#include <iostream>     //std
#include <vector>

// 普通函数
int add2(int c , int d){
    return c+d;
};
int main()
{
    // lambda
    int c = 10;
    //按值捕获（直接写变量名）只能读取变量，不能修改，副本默认是const
    auto add1 = [c](int a ,int b)-> int {
        // c = 1;   
        return a+b;
    };
    std::cout<< "add1和 " << add1(1,2) << std::endl;
    std::cout<< "add2和 " << add2(3,4) << std::endl;

    // 引用捕获（&变量名） 可读可修改变量
    int count = 0;
    auto increment = [&count]() { count++; };
    increment();
    increment();
    std::cout << "Count: " << count << std::endl;

    // 在算法中使用Lambda
    std::vector<int> numbers = {1, 2, 3, 4, 5};
    std::for_each(numbers.begin(), numbers.end(), 
                  [](int g) { std::cout << g * 2 << " "; });


    // 与const 的区别
    const int x = 10;  // 定义为const变量
    auto func = [x]() {
        // x = 20;  // 编译错误：x是const变量，无论如何都不能修改
    };
    // x = 30;  // 编译错误：在外部也不能修改const变量

    
    return 0;
}
/*
[]是捕获列表
add1: add1是一个Lambda 表达式的变量名
-> int :可以省略，编译器会自动推断返回类型

*/