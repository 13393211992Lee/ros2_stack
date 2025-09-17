#include<iostream>
using namespace std;

// 1. 值传递（无法真正交换外部变量）
void swap1(int a, int b) {
    int temp = a;
    a = b;
    b = temp;
}

// 2. 指针传递（通过指针操作实际内存）
void swap2(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

void swap3(int &a, int &b) {  // int &a = a; int &b = b;
	// 3、引用传递
	int temp = a;
	a = b;
	b = temp;
}
// int& fun1() {
// 	//int c = 10;
// 	//return c;  // 返回局部变量的引用
// }
int& fun2() {
	static int d = 10;  // static静态可以把变量的生命周期设置为程序的结束
	return d;  // 返回局部变量的引用
}
int main()
{
	int a = 10;
	int b = 20;
	swap1(a,b);
	//swap2(&a,&b);	
	//swap3(a,b);
	// 
	// 
	//注意事项
	// 1、引用必须是一块合法的内存空间
	//int& a1 = 10;   // 10是常量，不能直接这样写

	// 2、不要返回局部变量的引用
	//int &ref = fun1();
	//int &ref = fun2();
	//cout << "ref:" << ref << endl;

	// 3、函数返回值如果是引用，函数的调用可以作为左值出现

	//int& ref = fun2(); // 把a起别名叫ref
	//fun2() = 600; // a = 600 
	//cout << "ref:" << ref << endl;  // 600 因为a的别名是ref
 	return 0;
}
