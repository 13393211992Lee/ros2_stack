//函数重载
#include<iostream>
using namespace std;


void fun4(int a,double b) {
	cout << "int a,double b:  "<<a << "," << b << endl;
}

void fun4(double b,int a) {
	cout << "double b ,int a:  "<<b << "," << a << endl;
}

// 函数返回值不能作为函数重载的条件，因为会产生二义性
// int fun4(int a, double b) {
// 	return 1;
// 	cout << "int a,double b:" << endl;
// }

// 函数重载碰上默认参数，也会产生二义性，不能重载
// void fun4(double b=3.14, int a = 6) {
// 	cout << "默认double b,int a:" << endl;
// }

int main()
{
  fun4(1.2, 5);
  fun4(5, 1.2);
  return EXIT_SUCCESS;
  // return 0;
}

/*
函数重载实现原理


不同的编译器可能会产生不同的内部名。
void func(){}
void func(int x){}
void func(int x,char y){}

以上三个函数在linux下生成的编译之后的函数名为:
_Z4funcv //v 代表void,无参数
_Z4funci //i 代表参数为int类型
_Z4funcic //i 代表第一个参数为int类型，第二个参数为char类型

*/