#include <memory>
#include <iostream>
int main()
{
    // auto 用法
    // std::shared_ptr<std::string> ptr1 = std::make_shared<std::string>("this is a string");
    auto ptr1 = std::make_shared<std::string>("this is a string");
    std::cout<<"ptr1引用计数"<<ptr1.use_count() << ",指向的内存地址 " <<ptr1.get() << std::endl;

    // ptr2.use_count() 计数指的是 该指针指向内存地址 该地址被引用的次数
    auto ptr2 = ptr1;
    std::cout<<"ptr1引用计数"<<ptr1.use_count() << ",指向的内存地址 " <<ptr1.get() << std::endl;
    std::cout<<"ptr2引用计数"<<ptr2.use_count() << ",指向的内存地址 " <<ptr2.get() << std::endl;


    ptr1.reset(); //释放引用
    std::cout<<"ptr1引用计数"<<ptr1.use_count() << ",指向的内存地址 " <<ptr1.get() << std::endl;
    std::cout<<"ptr2引用计数"<<ptr2.use_count() << ",指向的内存地址 " <<ptr2.get() << std::endl;

    return 0;
}
