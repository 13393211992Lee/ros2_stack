#include<iostream>
using namespace std;
//#include "stu.h"  // 类模板中的成员函数，不会一开始就执行创建，分文件会链接不到。
#include "demo22_9.hpp"  // 推荐方法2：把实现和声明都放在一个.hpp的头文件中导入即可
int main(void)
{
	Students10<string, int> stu("张三", 19);
	stu.show();
	return EXIT_SUCCESS;
}
