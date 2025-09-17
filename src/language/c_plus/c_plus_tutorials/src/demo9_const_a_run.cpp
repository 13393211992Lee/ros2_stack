#include "demo9_const_a.h"
#include <iostream>


void someFunction() {
    std::cout << "global_const_a: " <<global_const_a << std::endl; // 使用 const 变量
    std::cout << "global_non_const_a: " << global_non_const_a << std::endl; // 使用非 const 变量

    global_non_const_a = 120;
    std::cout << "非const变量修改值global_non_const_a: " << global_non_const_a << std::endl;
}
int main(int argc, char const *argv[])
{
    (void)argc;
    (void)argv;
    someFunction();
    return 0;
}


